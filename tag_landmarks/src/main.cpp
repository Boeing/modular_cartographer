#include <tag_detection_aruco/tag_tracker.h>
#include <tag_interface/DetectTags.h>

#include <cartographer_ros_msgs/LandmarkList.h>

#include <camera_interface/GetImage.h>

#include <tf2_ros/transform_broadcaster.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

#include <atomic>
#include <string>
#include <thread>

template <typename T> T get_param_with_default_warn(const std::string& param_name, const T& default_val)
{
    if (ros::param::has(param_name))
    {
        T param_val;
        if (ros::param::get(param_name, param_val))
        {
            return param_val;
        }
    }
    ROS_WARN_STREAM("Using default value for " << param_name);
    return default_val;
}

template <typename T> T get_param_or_throw(const std::string& param_name)
{
    if (ros::param::has(param_name))
    {
        T param_val;
        if (ros::param::get(param_name, param_val))
        {
            return param_val;
        }
    }
    throw std::runtime_error("Must specify: " + param_name);
}

Eigen::Isometry3d convert(const geometry_msgs::Pose& tr)
{
    return Eigen::Translation3d(tr.position.x, tr.position.y, tr.position.z) *
           Eigen::Quaterniond(tr.orientation.w, tr.orientation.x, tr.orientation.y, tr.orientation.z);
}

Eigen::Isometry3d convert(const geometry_msgs::Transform& tr)
{
    return Eigen::Translation3d(tr.translation.x, tr.translation.y, tr.translation.z) *
           Eigen::Quaterniond(tr.rotation.w, tr.rotation.x, tr.rotation.y, tr.rotation.z);
}

class RosWrapper
{
  public:
    RosWrapper(const cv::Ptr<opencv_aruco::aruco::Dictionary>& dictionary,
               const cv::Ptr<opencv_aruco::aruco::DetectorParameters>& detector_params,
               const tag_detection_aruco::DetectorType detection_type, const int rows, const int cols,
               const double unit_size, const double marker_size)
        : dictionary_(dictionary), detector_params_(detector_params), detection_type_(detection_type),
          marker_size_(marker_size), board_rows_(rows), board_cols_(cols), board_unit_size_(unit_size), running_(true),
          nh_("~"), tf_listener_(tf_buffer_)
    {
        get_image_srv_ = nh_.serviceClient<camera_interface::GetImage>("/camera/get_image");
        landmark_publisher_ = nh_.advertise<cartographer_ros_msgs::LandmarkList>("/landmark", 1);

        run_thread_ = std::thread(&RosWrapper::run, this);
    }

    ~RosWrapper()
    {
        running_ = false;
        run_thread_.join();
    }

  private:
    void run()
    {
        ros::Rate rate(1.0);
        int i = 0;
        while (running_)
        {
            cartographer_ros_msgs::LandmarkList ls;
            ls.header.stamp = ros::Time::now();
            ls.header.frame_id = "base_link";
            ls.header.seq = i++;

            ROS_INFO("Detecting Tags");
            try
            {
                camera_interface::GetImageRequest req;
                req.timeout = 10.0;
                camera_interface::GetImageResponse res;
                if (!get_image_srv_.call(req, res) || !res.success)
                    throw std::runtime_error("Failed to call: " + get_image_srv_.getService());

                tag_detection_aruco::DetectResult result;
                cv_bridge::CvImagePtr im = tag_detection_aruco::loadImage(res.image);
                if (detection_type_ == tag_detection_aruco::DetectorType::ARUCO)
                {
                    result = tag_detection_aruco::detectTags(dictionary_, detector_params_, im->image, res.camera_info,
                                                             marker_size_);
                }
                else if (detection_type_ == tag_detection_aruco::DetectorType::CHARUCO)
                {
                    result =
                        tag_detection_aruco::detectBoards(dictionary_, detector_params_, im->image, res.camera_info,
                                                          marker_size_, board_rows_, board_cols_, board_unit_size_);
                }
                else
                {
                    throw std::runtime_error("Unknown detection type");
                }

                const geometry_msgs::TransformStamped tr = tf_buffer_.lookupTransform(
                    "base_link", res.image.header.frame_id, res.image.header.stamp, ros::Duration(0.1));
                const Eigen::Isometry3d tracking_to_camera = convert(tr.transform);

                for (const tag_interface::Tag& tag : result.tags)
                {
                    cartographer_ros_msgs::LandmarkEntry landmark;
                    landmark.id = std::to_string(tag.marker_id);

                    const Eigen::Isometry3d camera_to_tag = convert(tag.pose.pose);
                    Eigen::Isometry3d t = (tracking_to_camera * camera_to_tag);
                    //                    t = t.inverse();

                    landmark.tracking_from_landmark_transform.position.x = t.translation().x();
                    landmark.tracking_from_landmark_transform.position.y = t.translation().y();
                    landmark.tracking_from_landmark_transform.position.z = t.translation().z();
                    const Eigen::Quaterniond qt(t.linear());
                    landmark.tracking_from_landmark_transform.orientation.w = qt.w();
                    landmark.tracking_from_landmark_transform.orientation.x = qt.x();
                    landmark.tracking_from_landmark_transform.orientation.y = qt.y();
                    landmark.tracking_from_landmark_transform.orientation.z = qt.z();

                    landmark.translation_weight = 1e3;
                    landmark.rotation_weight = 1e3;

                    ls.landmarks.push_back(landmark);
                }

                landmark_publisher_.publish(ls);
            }
            catch (const std::exception& e)
            {
                ROS_ERROR_STREAM(e.what());
            }

            rate.sleep();
        }
    }

    const cv::Ptr<opencv_aruco::aruco::Dictionary> dictionary_;
    const cv::Ptr<opencv_aruco::aruco::DetectorParameters> detector_params_;
    const tag_detection_aruco::DetectorType detection_type_;

    const double marker_size_;
    const int board_rows_;
    const int board_cols_;
    const double board_unit_size_;

    std::atomic<bool> running_;
    std::thread run_thread_;

    ros::NodeHandle nh_;
    ros::ServiceClient get_image_srv_;
    ros::Publisher landmark_publisher_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "tag_detection_aruco");

    ros::NodeHandle nh("~");

    // Detector params
    const tag_detection_aruco::DetectorType detector_type = tag_detection_aruco::getDetectorTypeFromString(
        get_param_with_default_warn<std::string>("~detector_type", "aruco"));

    // Marker params
    const double marker_size = get_param_or_throw<double>("~marker_size");
    const int board_rows = get_param_with_default_warn<int>("~board_rows", 8);
    const int board_cols = get_param_with_default_warn<int>("~board_cols", 7);
    const double board_unit_size = get_param_with_default_warn<double>("~unit_size", 0.04);

    cv::Ptr<opencv_aruco::aruco::Dictionary> dictionary =
        opencv_aruco::aruco::getPredefinedDictionary(opencv_aruco::aruco::PREDEFINED_DICTIONARY_NAME(7));

    cv::Ptr<opencv_aruco::aruco::DetectorParameters> params = opencv_aruco::aruco::DetectorParameters::create();
    params->adaptiveThreshWinSizeMin = 3;
    params->adaptiveThreshWinSizeMax = 23;
    params->adaptiveThreshWinSizeStep = 10;
    params->adaptiveThreshConstant = 7;
    params->minMarkerPerimeterRate = 0.03;
    params->maxMarkerPerimeterRate = 4.0;
    params->polygonalApproxAccuracyRate = 0.05;
    params->minDistanceToBorder = 3;
    params->minMarkerDistanceRate = 0.05;
    params->cornerRefinementMethod = opencv_aruco::aruco::CornerRefineMethod::CORNER_REFINE_APRILTAG;
    params->cornerRefinementWinSize = 5;
    params->cornerRefinementMaxIterations = 1000;
    params->cornerRefinementMinAccuracy = 0.01;
    params->markerBorderBits = 1;
    params->perspectiveRemovePixelPerCell = 8;
    params->perspectiveRemoveIgnoredMarginPerCell = 0.13;
    params->maxErroneousBitsInBorderRate = 0.04;
    params->minOtsuStdDev = 5.0;
    params->errorCorrectionRate = 0.6;

    RosWrapper ros_wrapper(dictionary, params, detector_type, board_rows, board_cols, board_unit_size, marker_size);

    ros::spin();

    return EXIT_SUCCESS;
}

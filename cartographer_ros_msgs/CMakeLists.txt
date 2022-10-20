cmake_minimum_required(VERSION 3.8)

project(cartographer_ros_msgs)

# find_package(catkin REQUIRED COMPONENTS std_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# Package deps
find_package(geometry_msgs REQUIRED)
find_package(graph_map REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
    "msg/BagfileProgress.msg"
    "msg/HistogramBucket.msg"
    "msg/LandmarkEntry.msg"
    "msg/LandmarkList.msg"
    "msg/Metric.msg"
    "msg/MetricFamily.msg"
    "msg/MetricLabel.msg"
    "msg/StatusCode.msg"
    "msg/StatusResponse.msg"
    "msg/SubmapEntry.msg"
    "msg/SubmapList.msg"
    "msg/SubmapTexture.msg"
    "msg/SystemState.msg"
    "msg/TrajectoryStates.msg"
    "srv/FinishTrajectory.srv"
    "srv/GetTrajectoryStates.srv"
    "srv/LoadState.srv"
    "srv/ReadMetrics.srv"
    "srv/StartLocalisation.srv"
    "srv/StartMapping.srv"
    "srv/SubmapQuery.srv"
    "srv/TrajectoryQuery.srv"
    "srv/WriteState.srv"
    DEPENDENCIES
    geometry_msgs
    graph_map
    nav_msgs
    sensor_msgs
    std_msgs
)

ament_package()

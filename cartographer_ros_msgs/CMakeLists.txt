cmake_minimum_required(VERSION 2.8.3)

project(cartographer_ros_msgs)

set(PACKAGE_DEPENDENCIES
    geometry_msgs
    std_msgs
    sensor_msgs
    nav_msgs
)

find_package(catkin REQUIRED COMPONENTS message_generation ${PACKAGE_DEPENDENCIES})

add_message_files(
  DIRECTORY msg
  FILES
    BagfileProgress.msg
    HistogramBucket.msg
    LandmarkEntry.msg
    LandmarkList.msg
    MetricFamily.msg
    MetricLabel.msg
    Metric.msg
    StatusCode.msg
    StatusResponse.msg
    SubmapEntry.msg
    SubmapList.msg
    SubmapTexture.msg
    TrajectoryStates.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    FinishTrajectory.srv
    GetTrajectoryStates.srv
    LoadState.srv
    ReadMetrics.srv
    StartLocalisation.srv
    StartMapping.srv
    SubmapQuery.srv
    TrajectoryQuery.srv
    WriteState.srv
)

generate_messages(
  DEPENDENCIES
    ${PACKAGE_DEPENDENCIES}
)

catkin_package(
  CATKIN_DEPENDS
    ${PACKAGE_DEPENDENCIES}
    message_runtime
)

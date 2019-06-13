cmake_minimum_required(VERSION 2.8.3)

project(cartographer_ros_msgs)

find_package(catkin REQUIRED COMPONENTS 
  geometry_msgs
  message_generation
  nav_msgs
  sensor_msgs
  std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
    BagfileProgress.msg
    HistogramBucket.msg
    LandmarkEntry.msg
    LandmarkList.msg
    Metric.msg
    MetricFamily.msg
    MetricLabel.msg
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
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    nav_msgs
    sensor_msgs
    std_msgs
)

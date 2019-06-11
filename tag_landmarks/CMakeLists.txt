cmake_minimum_required(VERSION 2.8.3)
project(tag_landmarks)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    tag_detection_aruco
    roscpp
    tf2_ros
    camera_interface
    cartographer_ros_msgs
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
    CATKIN_DEPENDS
        tag_detection_aruco
        roscpp
        tf2_ros
        camera_interface
        cartographer_ros_msgs
    INCLUDE_DIRS
    LIBRARIES
    DEPENDS
        EIGEN3
)

include_directories(include)

include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
    src/main.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

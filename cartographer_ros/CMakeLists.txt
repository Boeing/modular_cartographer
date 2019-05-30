cmake_minimum_required(VERSION 2.8.12)

project(cartographer_ros)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(ExternalProject)

set(PROTOBUF_PROJECT_NAME protobuf)
set(PROTOBUF_PROJECT_SRC_DIR ${CMAKE_CURRENT_BINARY_DIR}/${PROTOBUF_PROJECT_NAME}/src/${PROTOBUF_PROJECT_NAME})
set(PROTOBUF_PROJECT_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/${PROTOBUF_PROJECT_NAME}/src/${PROTOBUF_PROJECT_NAME}-build)
set(PROTOBUF_INSTALL_DIR ${PROTOBUF_PROJECT_BUILD_DIR}/install)
set(PROTOBUF_INCLUDE_DIRS ${PROTOBUF_INSTALL_DIR}/include)
set(PROTOBUF_LIBRARIES ${PROTOBUF_INSTALL_DIR}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}protobuf${CMAKE_STATIC_LIBRARY_SUFFIX})
externalproject_add(${PROTOBUF_PROJECT_NAME}
    PREFIX ${PROTOBUF_PROJECT_NAME}
    URL ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/protobuf-3.4.1.tar.gz
    UPDATE_COMMAND ""
    CONFIGURE_COMMAND ${CMAKE_COMMAND} ${PROTOBUF_PROJECT_SRC_DIR}/cmake -GNinja -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_BUILD_TYPE=Release -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX=install
    CMAKE_GENERATOR "Ninja"
    BUILD_COMMAND "ninja"
    INSTALL_COMMAND ninja install
    BUILD_BYPRODUCTS "${PROTOBUF_LIBRARIES}"
)

set(CARTOGRAPHER_PROJECT_NAME cartographer)
set(CARTOGRAPHER_PROJECT_SRC_DIR ${CMAKE_CURRENT_BINARY_DIR}/${CARTOGRAPHER_PROJECT_NAME}/src/${CARTOGRAPHER_PROJECT_NAME})
set(CARTOGRAPHER_PROJECT_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/${CARTOGRAPHER_PROJECT_NAME}/src/${CARTOGRAPHER_PROJECT_NAME}-build)
set(CARTOGRAPHER_INSTALL_DIR ${CARTOGRAPHER_PROJECT_BUILD_DIR}/install)
set(CARTOGRAPHER_INCLUDE_DIRS ${CARTOGRAPHER_INSTALL_DIR}/include)
set(CARTOGRAPHER_LIBRARIES ${CARTOGRAPHER_INSTALL_DIR}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}cartographer${CMAKE_STATIC_LIBRARY_SUFFIX})
externalproject_add(${CARTOGRAPHER_PROJECT_NAME}
    PREFIX ${CARTOGRAPHER_PROJECT_NAME}
    URL ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/bcd5486025df4f601c3977c44a5e00e9c80b4975.tar.gz
    UPDATE_COMMAND ""
    DEPENDS ${PROTOBUF_PROJECT_NAME}
    CMAKE_ARGS -DCMAKE_PREFIX_PATH=${PROTOBUF_INSTALL_DIR} -DCMAKE_INSTALL_PREFIX=install
    CMAKE_GENERATOR "Ninja"
    BUILD_COMMAND "ninja"
    INSTALL_COMMAND ninja install
    BUILD_BYPRODUCTS "${CARTOGRAPHER_LIBRARIES}"
)

set(PACKAGE_DEPENDENCIES
    ${PROJECT_NAME}_msgs
    eigen_conversions
    geometry_msgs
    message_runtime
    nav_msgs
    pcl_conversions
    pluginlib
    rosbag
    roscpp
    roslib
    rviz
    sensor_msgs
    std_msgs
    tf2
    tf2_eigen
    tf2_ros
    urdf
    visualization_msgs
)

enable_testing()
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(Boost REQUIRED COMPONENTS system iostreams)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Abseil REQUIRED)
find_package(LuaGoogle REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(GMock REQUIRED)
find_package(urdfdom_headers REQUIRED)
if(DEFINED urdfdom_headers_VERSION)
    if(${urdfdom_headers_VERSION} GREATER 0.4.1)
        add_definitions(-DURDFDOM_HEADERS_HAS_SHARED_PTR_DEFS)
    endif()
endif()
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
find_package(PkgConfig)
pkg_search_module(CAIRO REQUIRED cairo>=1.12.16)

set(GTEST_FOUND TRUE)
set(GTEST_INCLUDE_DIRS ${GMOCK_INCLUDE_DIRS})
set(GTEST_LIBRARIES ${GMOCK_LIBRARIES})

catkin_package(
    CATKIN_DEPENDS
        ${PACKAGE_DEPENDENCIES}
    DEPENDS
        ABSEIL
        Boost
        CERES
        EIGEN3
        PCL
        urdfdom_headers
    INCLUDE_DIRS
    LIBRARIES ${PROJECT_NAME}
)

include_directories(include)

include_directories(SYSTEM
    ${ABSEIL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${CAIRO_INCLUDE_DIRS}
    ${CARTOGRAPHER_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${LUA_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${PROTOBUF_INCLUDE_DIRS}
    ${urdfdom_headers_INCLUDE_DIRS}
)

set(CMAKE_AUTOMOC ON)
include_directories(${Qt5Widgets_INCLUDE_DIRS})
add_definitions(-DQT_NO_KEYWORDS)

file(GLOB_RECURSE ALL_SRCS "src/*.cc" "include/${PROJECT_NAME}/rviz/*.h")
file(GLOB_RECURSE ALL_TESTS "src/*_test.cc")
file(GLOB_RECURSE ALL_EXECUTABLES "src/*_main.cc")
list(REMOVE_ITEM ALL_SRCS ${ALL_TESTS})
list(REMOVE_ITEM ALL_SRCS ${ALL_EXECUTABLES})

message(STATUS "ALL_SRCS: ${ALL_SRCS}")

add_library(${PROJECT_NAME} ${ALL_SRCS})
add_dependencies(${PROJECT_NAME} cartographer)

set(LIBS
    ${Boost_LIBRARIES}
    ${CAIRO_LIBRARIES}
    ${CARTOGRAPHER_LIBRARIES}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
    ${LUA_LIBRARIES}
    ${PCL_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    gflags
    glog
    Qt5::Core
    Qt5::Widgets
    standalone_absl
)

add_executable(cartographer_assets_writer
    src/assets_writer_main.cc
    src/ros_map_writing_points_processor.cc
)

add_executable(cartographer_node
    src/node_main.cc
)

add_executable(cartographer_offline_node
    src/offline_node_main.cc
)

add_executable(cartographer_occupancy_grid_node
    src/occupancy_grid_node_main.cc
)

add_executable(${PROJECT_NAME}bag_validate
    src/rosbag_validate_main.cc
)

add_executable(cartographer_pbstream_to_ros_map
    src/pbstream_to_ros_map_main.cc
)

add_executable(cartographer_pbstream_map_publisher
    src/pbstream_map_publisher_main.cc
)

add_executable(cartographer_dev_pbstream_trajectories_to_rosbag
    src/dev/pbstream_trajectories_to_rosbag_main.cc
)

add_executable(cartographer_dev_rosbag_publisher
    src/dev/rosbag_publisher_main.cc
)

add_executable(cartographer_dev_trajectory_comparison
    src/dev/trajectory_comparison_main.cc
)

target_link_libraries(${PROJECT_NAME} PUBLIC ${LIBS})
target_link_libraries(cartographer_assets_writer PUBLIC ${PROJECT_NAME} ${LIBS})
target_link_libraries(cartographer_node PUBLIC ${PROJECT_NAME} ${LIBS})
target_link_libraries(cartographer_offline_node PUBLIC ${PROJECT_NAME} ${LIBS})
target_link_libraries(cartographer_occupancy_grid_node PUBLIC ${PROJECT_NAME} ${LIBS})
target_link_libraries(${PROJECT_NAME}bag_validate PUBLIC ${PROJECT_NAME} ${LIBS})
target_link_libraries(cartographer_pbstream_to_ros_map PUBLIC ${PROJECT_NAME} ${LIBS})
target_link_libraries(cartographer_pbstream_map_publisher PUBLIC ${PROJECT_NAME} ${LIBS})
target_link_libraries(cartographer_dev_pbstream_trajectories_to_rosbag PUBLIC ${PROJECT_NAME} ${LIBS})
target_link_libraries(cartographer_dev_rosbag_publisher PUBLIC ${PROJECT_NAME} ${LIBS})
target_link_libraries(cartographer_dev_trajectory_comparison PUBLIC ${PROJECT_NAME} ${LIBS})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} cartographer)

if (CATKIN_ENABLE_TESTING)
    foreach(TEST_SOURCE_FILENAME ${ALL_TESTS})
        get_filename_component(TEST_NAME ${TEST_SOURCE_FILENAME} NAME_WE)
        catkin_add_gtest(${TEST_NAME} ${TEST_SOURCE_FILENAME})
        add_dependencies(${TEST_NAME} ${catkin_EXPORTED_TARGETS})
        target_link_libraries(${TEST_NAME}
            ${PROJECT_NAME}
        )
    endforeach()
endif()

install(TARGETS
    cartographer_assets_writer
    cartographer_dev_pbstream_trajectories_to_rosbag
    cartographer_dev_rosbag_publisher
    cartographer_dev_trajectory_comparison
    cartographer_node
    cartographer_occupancy_grid_node
    #cartographer_offline_node
    cartographer_pbstream_map_publisher
    cartographer_pbstream_to_ros_map
    ${PROJECT_NAME}
    ${PROJECT_NAME}bag_validate

    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES rviz_plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY ogre_media
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


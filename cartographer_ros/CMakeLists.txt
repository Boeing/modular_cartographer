cmake_minimum_required(VERSION 2.8.12)

project(cartographer_ros)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(ExternalProject)

# Abseil
set(ABSEIL_PROJECT_NAME abseil)
set(ABSEIL_PROJECT_SRC_DIR ${CMAKE_CURRENT_BINARY_DIR}/${ABSEIL_PROJECT_NAME}/src/${ABSEIL_PROJECT_NAME})
set(ABSEIL_PROJECT_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/${ABSEIL_PROJECT_NAME}/src/${ABSEIL_PROJECT_NAME}-build)
set(ABSEIL_INCLUDE_DIRS ${ABSEIL_PROJECT_SRC_DIR})
# ensure absl_synchonization is first
set(ABSEIL_LIBRARIES
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/synchronization/${CMAKE_STATIC_LIBRARY_PREFIX}absl_synchronization${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/debugging/${CMAKE_STATIC_LIBRARY_PREFIX}absl_symbolize${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/time/${CMAKE_STATIC_LIBRARY_PREFIX}absl_time${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/strings/${CMAKE_STATIC_LIBRARY_PREFIX}str_format_internal${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/strings/${CMAKE_STATIC_LIBRARY_PREFIX}str_format_extension_internal${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/strings/${CMAKE_STATIC_LIBRARY_PREFIX}absl_str_format${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/strings/${CMAKE_STATIC_LIBRARY_PREFIX}absl_strings${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/hash/${CMAKE_STATIC_LIBRARY_PREFIX}absl_hash${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/algorithm/${CMAKE_STATIC_LIBRARY_PREFIX}absl_algorithm${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/base/${CMAKE_STATIC_LIBRARY_PREFIX}absl_base${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/base/${CMAKE_STATIC_LIBRARY_PREFIX}absl_dynamic_annotations${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/base/${CMAKE_STATIC_LIBRARY_PREFIX}absl_internal_malloc_internal${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/base/${CMAKE_STATIC_LIBRARY_PREFIX}absl_internal_spinlock_wait${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/base/${CMAKE_STATIC_LIBRARY_PREFIX}absl_internal_throw_delegate${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/container/${CMAKE_STATIC_LIBRARY_PREFIX}absl_container${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/container/${CMAKE_STATIC_LIBRARY_PREFIX}test_instance_tracker_lib${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/debugging/${CMAKE_STATIC_LIBRARY_PREFIX}absl_debugging${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/debugging/${CMAKE_STATIC_LIBRARY_PREFIX}absl_examine_stack${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/debugging/${CMAKE_STATIC_LIBRARY_PREFIX}absl_failure_signal_handler${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/debugging/${CMAKE_STATIC_LIBRARY_PREFIX}absl_leak_check${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/debugging/${CMAKE_STATIC_LIBRARY_PREFIX}absl_stack_consumption${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/debugging/${CMAKE_STATIC_LIBRARY_PREFIX}absl_stacktrace${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/memory/${CMAKE_STATIC_LIBRARY_PREFIX}absl_memory${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/meta/${CMAKE_STATIC_LIBRARY_PREFIX}absl_meta${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/numeric/${CMAKE_STATIC_LIBRARY_PREFIX}absl_int128${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/numeric/${CMAKE_STATIC_LIBRARY_PREFIX}absl_numeric${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/types/${CMAKE_STATIC_LIBRARY_PREFIX}absl_any${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/types/${CMAKE_STATIC_LIBRARY_PREFIX}absl_bad_any_cast${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/types/${CMAKE_STATIC_LIBRARY_PREFIX}absl_bad_optional_access${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/types/${CMAKE_STATIC_LIBRARY_PREFIX}absl_optional${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/types/${CMAKE_STATIC_LIBRARY_PREFIX}absl_span${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/types/${CMAKE_STATIC_LIBRARY_PREFIX}absl_variant${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/utility/${CMAKE_STATIC_LIBRARY_PREFIX}absl_utility${CMAKE_STATIC_LIBRARY_SUFFIX}"
)
message(STATUS "${ABSEIL_LIBRARIES}")
externalproject_add(${ABSEIL_PROJECT_NAME}
    PREFIX ${ABSEIL_PROJECT_NAME}
    URL ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/abseil-cpp-7b46e1d31a6b08b1c6da2a13e7b151a20446fa07.tar.gz
    URL_MD5 86b452e655610381675a09742244b307
    INSTALL_COMMAND  ""
    BUILD_COMMAND    ${CMAKE_COMMAND} --build "${ABSEIL_PROJECT_BUILD_DIR}"
    CMAKE_CACHE_ARGS 
        "-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON"
        "-DBUILD_TESTING:BOOL=OFF"
        "-DCMAKE_BUILD_TYPE:STRING=Release"
    BUILD_BYPRODUCTS "${ABSEIL_LIBRARIES}"
)

# Protobuf
set(PROTOBUF_PROJECT_NAME protobuf)
set(PROTOBUF_PROJECT_SRC_DIR ${CMAKE_CURRENT_BINARY_DIR}/${PROTOBUF_PROJECT_NAME}/src/${PROTOBUF_PROJECT_NAME})
set(PROTOBUF_PROJECT_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/${PROTOBUF_PROJECT_NAME}/src/${PROTOBUF_PROJECT_NAME}-build)
set(PROTOBUF_INSTALL_DIR ${PROTOBUF_PROJECT_BUILD_DIR}/install)
set(PROTOBUF_INCLUDE_DIRS ${PROTOBUF_INSTALL_DIR}/include)
set(PROTOBUF_LIBRARIES ${PROTOBUF_INSTALL_DIR}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}protobuf${CMAKE_STATIC_LIBRARY_SUFFIX})
externalproject_add(${PROTOBUF_PROJECT_NAME}
    PREFIX ${PROTOBUF_PROJECT_NAME}
    URL ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/protobuf-3.4.1.tar.gz
    URL_MD5 31b19dcfd6567095fdb66a8c07347222
    UPDATE_COMMAND ""
    CONFIGURE_COMMAND ${CMAKE_COMMAND} ${PROTOBUF_PROJECT_SRC_DIR}/cmake -GNinja -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_BUILD_TYPE=Release -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX=install
    CMAKE_GENERATOR "Ninja"
    BUILD_COMMAND "ninja"
    INSTALL_COMMAND ninja install
    BUILD_BYPRODUCTS "${PROTOBUF_LIBRARIES}"
)

# Cartographer
set(CARTOGRAPHER_PROJECT_NAME cartographer)
set(CARTOGRAPHER_PROJECT_SRC_DIR ${CMAKE_CURRENT_BINARY_DIR}/${CARTOGRAPHER_PROJECT_NAME}/src/${CARTOGRAPHER_PROJECT_NAME})
set(CARTOGRAPHER_PROJECT_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/${CARTOGRAPHER_PROJECT_NAME}/src/${CARTOGRAPHER_PROJECT_NAME}-build)
set(CARTOGRAPHER_INSTALL_DIR ${CARTOGRAPHER_PROJECT_BUILD_DIR}/install)
set(CARTOGRAPHER_INCLUDE_DIRS ${CARTOGRAPHER_INSTALL_DIR}/include)
set(CARTOGRAPHER_LIBRARIES ${CARTOGRAPHER_INSTALL_DIR}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}cartographer${CMAKE_STATIC_LIBRARY_SUFFIX})
externalproject_add(${CARTOGRAPHER_PROJECT_NAME}
    PREFIX ${CARTOGRAPHER_PROJECT_NAME}
    GIT_REPOSITORY https://git.web.boeing.com/brta-robotics/cartographer.git
    #GIT_TAG master
    UPDATE_COMMAND ""
    DEPENDS ${PROTOBUF_PROJECT_NAME} ${ABSEIL_PROJECT_NAME}
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=install
    CMAKE_CACHE_ARGS
        "-DCMAKE_PREFIX_PATH:STRING=${PROTOBUF_INSTALL_DIR}"
        "-DABSEIL_INCLUDE_DIRS:STRING=${ABSEIL_INCLUDE_DIRS}"
        "-DABSEIL_LIBRARIES:STRING=${ABSEIL_LIBRARIES}"
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
        Boost
        CERES
        EIGEN3
        PCL
        urdfdom_headers
    INCLUDE_DIRS
    LIBRARIES ${PROJECT_NAME}
)

include_directories(include)

#catkin_lint: ignore_once missing_directory
include_directories(SYSTEM
    ${ABSEIL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${CAIRO_INCLUDE_DIRS}
    ${CARTOGRAPHER_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${LUA_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${PROTOBUF_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${urdfdom_headers_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${GTEST_INCLUDE_DIRS}
)

set(SRC_FILES
    src/assets_writer.cc
    src/map_builder_bridge.cc
    src/metrics/family_factory.cc
    src/metrics/internal/family.cc
    src/metrics/internal/histogram.cc
    src/metrics/internal/metrics_test.cc
    src/msg_conversion.cc
    src/node.cc
    src/node_constants.cc
    src/node_options.cc
    src/playable_bag.cc
    src/proto_sstream.cc
    src/ros_log_sink.cc
    src/ros_map.cc
    src/ros_map_writing_points_processor.cc
    src/sensor_bridge.cc
    src/submap.cc
    src/tf_bridge.cc
    src/time_conversion.cc
    src/trajectory_options.cc
    src/urdf_reader.cc
)

add_library(${PROJECT_NAME} ${SRC_FILES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${CARTOGRAPHER_PROJECT_NAME})

set(RVIZ_SRC_FILES
    include/${PROJECT_NAME}/rviz/drawable_submap.h
    include/${PROJECT_NAME}/rviz/ogre_slice.h
    include/${PROJECT_NAME}/rviz/submaps_display.h
    src/rviz/drawable_submap.cc
    src/rviz/ogre_slice.cc
    src/rviz/submaps_display.cc
)

qt5_wrap_cpp(RVIZ_SRC_FILES_MOC ${RVIZ_SRC_FILES})
add_library(cartographer_rviz_plugin ${RVIZ_SRC_FILES_MOC} ${RVIZ_SRC_FILES})

set(LIBS
    ${ABSEIL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${CAIRO_LIBRARIES}
    ${CARTOGRAPHER_LIBRARIES}
    ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
    ${LUA_LIBRARIES}
    ${PCL_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${GTEST_LIBRARIES}
    gflags
    glog
    Qt5::Core
    Qt5::Widgets
)

add_executable(cartographer_assets_writer
    src/dev/assets_writer_main.cc
    src/ros_map_writing_points_processor.cc
)

add_executable(cartographer_node
    src/node_main.cc
)

add_executable(cartographer_offline_node
    src/dev/offline_node_main.cc
)

add_executable(cartographer_occupancy_grid_node
    src/dev/occupancy_grid_node_main.cc
)

add_executable(${PROJECT_NAME}bag_validate
    src/dev/rosbag_validate_main.cc
)

add_executable(cartographer_pbstream_to_ros_map
    src/dev/pbstream_to_ros_map_main.cc
)

add_executable(cartographer_pbstream_map_publisher
    src/dev/pbstream_map_publisher_main.cc
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
target_link_libraries(cartographer_rviz_plugin PUBLIC ${PROJECT_NAME} ${LIBS})
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

# add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${CARTOGRAPHER_PROJECT_NAME})

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
    cartographer_offline_node
    cartographer_pbstream_map_publisher
    cartographer_pbstream_to_ros_map
    ${PROJECT_NAME}
    ${PROJECT_NAME}bag_validate
    cartographer_rviz_plugin

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

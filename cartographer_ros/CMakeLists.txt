cmake_minimum_required(VERSION 3.8)

project(cartographer_ros)

# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17) #Replaced 14 for 17 for is_same_v compatibility
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Abseil error https://stackoverflow.com/questions/65536494/why-am-i-keeping-get-this-error-message-xxx-so-undefined-reference-to-abslst
# Boost error https://stackoverflow.com/questions/39591067/undefined-reference-to-boostprogram-options-in-xubuntu
# Fixes abseil binding, but breaks everything else
# add_definitions("-D_GLIBCXX_USE_CXX11_ABI=0") 

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
externalproject_add(
    ${ABSEIL_PROJECT_NAME}
    PREFIX ${ABSEIL_PROJECT_NAME}
    URL ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/abseil-cpp-7b46e1d31a6b08b1c6da2a13e7b151a20446fa07.tar.gz
    URL_MD5 86b452e655610381675a09742244b307
    INSTALL_COMMAND ""
    BUILD_COMMAND ${CMAKE_COMMAND} --build "${ABSEIL_PROJECT_BUILD_DIR}"
    CMAKE_CACHE_ARGS "-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON" "-DBUILD_TESTING:BOOL=OFF"
                     "-DCMAKE_BUILD_TYPE:STRING=Release" "CMAKE_CXX_STANDARD:STRING=17"
    BUILD_BYPRODUCTS "${ABSEIL_LIBRARIES}"
)

# Protobuf
set(PROTOBUF_PROJECT_NAME protobuf)
if(NOT DEFINED PROTOBUF_INSTALL_DIR)
    set(PROTOBUF_PROJECT_SRC_DIR ${CMAKE_CURRENT_BINARY_DIR}/${PROTOBUF_PROJECT_NAME}/src/${PROTOBUF_PROJECT_NAME})
    set(PROTOBUF_PROJECT_BUILD_DIR
        ${CMAKE_CURRENT_BINARY_DIR}/${PROTOBUF_PROJECT_NAME}/src/${PROTOBUF_PROJECT_NAME}-build
    )
    set(PROTOBUF_INSTALL_DIR ${PROTOBUF_PROJECT_BUILD_DIR}/install)
    externalproject_add(
        ${PROTOBUF_PROJECT_NAME}
        PREFIX ${PROTOBUF_PROJECT_NAME}
        URL ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/protobuf-3.4.1.tar.gz
        URL_MD5 31b19dcfd6567095fdb66a8c07347222
        UPDATE_COMMAND ""
        CONFIGURE_COMMAND
            ${CMAKE_COMMAND} ${PROTOBUF_PROJECT_SRC_DIR}/cmake -GNinja -DCMAKE_POSITION_INDEPENDENT_CODE=ON
            -DCMAKE_BUILD_TYPE=Release -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX=install
        CMAKE_GENERATOR "Ninja"
        BUILD_COMMAND "ninja"
        INSTALL_COMMAND ninja install
        BUILD_BYPRODUCTS "${PROTOBUF_LIBRARIES}"
    )
endif()
set(PROTOBUF_INCLUDE_DIRS ${PROTOBUF_INSTALL_DIR}/include)
set(PROTOBUF_LIBRARIES ${PROTOBUF_INSTALL_DIR}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}protobuf${CMAKE_STATIC_LIBRARY_SUFFIX})

# Cartographer
set(CARTOGRAPHER_PROJECT_NAME cartographer)
# set(CARTOGRAPHER_INSTALL_DIR /root/ros2_ws/aux/cartographer/build/install)
if(NOT DEFINED CARTOGRAPHER_INSTALL_DIR)
    set(CARTOGRAPHER_PROJECT_SRC_DIR
        ${CMAKE_CURRENT_BINARY_DIR}/${CARTOGRAPHER_PROJECT_NAME}/src/${CARTOGRAPHER_PROJECT_NAME}
    )
    set(CARTOGRAPHER_PROJECT_BUILD_DIR
        ${CMAKE_CURRENT_BINARY_DIR}/${CARTOGRAPHER_PROJECT_NAME}/src/${CARTOGRAPHER_PROJECT_NAME}-build
    )
    set(CARTOGRAPHER_INSTALL_DIR ${CARTOGRAPHER_PROJECT_BUILD_DIR}/install)
    externalproject_add(
        ${CARTOGRAPHER_PROJECT_NAME}
        PREFIX ${CARTOGRAPHER_PROJECT_NAME}
        # GIT_REPOSITORY https://git.web.boeing.com/brta-robotics/cartographer.git
        # GIT_TAG 8d17b2fadb3a754f52f7730fc2e1a39e4803bcb1
        SOURCE_DIR /root/ros2_ws/src/cartographer 
        UPDATE_COMMAND ""
        DEPENDS ${PROTOBUF_PROJECT_NAME} ${ABSEIL_PROJECT_NAME}
        CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=install
        CMAKE_CACHE_ARGS
            "-DCMAKE_PREFIX_PATH:STRING=${PROTOBUF_INSTALL_DIR}" "-DABSEIL_INCLUDE_DIRS:STRING=${ABSEIL_INCLUDE_DIRS}"
            "-DABSEIL_LIBRARIES:STRING=${ABSEIL_LIBRARIES}"
        CMAKE_GENERATOR "Ninja"
        BUILD_COMMAND "ninja"
        INSTALL_COMMAND ninja install
        BUILD_BYPRODUCTS "${CARTOGRAPHER_LIBRARIES}"
    )
endif()
set(CARTOGRAPHER_INCLUDE_DIRS ${CARTOGRAPHER_INSTALL_DIR}/include)
set(CARTOGRAPHER_LIBRARIES
    ${CARTOGRAPHER_INSTALL_DIR}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}cartographer${CMAKE_STATIC_LIBRARY_SUFFIX}
)

# set(PACKAGE_DEPENDENCIES
#     ${PROJECT_NAME}_msgs
#     eigen_conversions
#     geometry_msgs
#     # googletest
#     message_runtime
#     nav_msgs
#     pcl_conversions
#     pluginlib
#     rosbag
#     rclcpp
#     roslib
#     rviz
#     sensor_msgs
#     std_msgs
#     tf2
#     tf2_eigen
#     tf2_ros
#     urdf
#     visualization_msgs
# )

# enable_testing()
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)

find_package(ament_cmake REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})
find_package(cartographer_ros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system iostreams program_options)
find_package(PCL REQUIRED COMPONENTS common)
# find_package(LuaGoogle REQUIRED)
find_package(Lua REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(urdfdom_headers REQUIRED)
if(DEFINED urdfdom_headers_VERSION)
    if(${urdfdom_headers_VERSION} GREATER 0.4.1)
        add_definitions(-DURDFDOM_HEADERS_HAS_SHARED_PTR_DEFS)
    endif()
endif()
# find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
find_package(gflags REQUIRED)
find_package(std_srvs REQUIRED)
find_package(urdf REQUIRED)
# find_package(tinyxml REQUIRED)
# find_package(absl REQUIRED PATHS /root/ros2_ws/aux/abseil-install)
# find_package(protobuf REQUIRED PATHS /root/ros2_ws/aux/protobuf/build/install)
# set(ABSEIL_INCLUDE_DIRS /root/ros2_ws/aux/abseil-install/include)
# set(ABSEIL_LIBRARIES /root/ros2_ws/aux/abseil-install/lib)
# set(PROTOBUF_INCLUDE_DIRS /root/ros2_ws/aux/protobuf/build/install/include)
# set(PROTOBUF_LIBRARIES /root/ros2_ws/aux/protobuf/build/install/lib)

find_package(PkgConfig)
# include(FindPkgConfig)

pkg_search_module(CAIRO REQUIRED cairo>=1.12.16)

# catkin_package(
#     CATKIN_DEPENDS
#     ${PACKAGE_DEPENDENCIES}
#     DEPENDS
#     Boost
#     CERES
#     EIGEN3
#     PCL
#     urdfdom_headers
#     INCLUDE_DIRS
#     LIBRARIES
#     ${PROJECT_NAME}
# )

# include_directories(include)

# # Need to stop cmake_format from adding space between # and catkin_lint
# # cmake-format: off
# #catkin_lint: ignore_once missing_directory
# # cmake-format: on
# include_directories(
#     SYSTEM
#     ${ABSEIL_INCLUDE_DIRS}
#     ${Boost_INCLUDE_DIRS}
#     ${CAIRO_INCLUDE_DIRS}
#     ${CARTOGRAPHER_INCLUDE_DIRS}
#     ${CERES_INCLUDE_DIRS}
#     ${EIGEN3_INCLUDE_DIR}
#     ${LUA_INCLUDE_DIR}
#     ${PCL_INCLUDE_DIRS}
#     ${PROTOBUF_INCLUDE_DIRS}
#     ${catkin_INCLUDE_DIRS}
#     ${urdfdom_headers_INCLUDE_DIRS}
#     ${Qt5Widgets_INCLUDE_DIRS}
# )


include_directories(
  include
  "."
  # ${cartographer_INCLUDE_DIRS}
  ${cartographer_ros_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  # From previous CMakeLists
  SYSTEM
  ${ABSEIL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CAIRO_INCLUDE_DIRS}
  ${CARTOGRAPHER_INCLUDE_DIRS} 
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${LUA_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)


set(SRC_FILES
    src/assets_writer.cc # Use of rosbag
    src/map_builder_bridge.cc
    src/metrics/family_factory.cc
    src/metrics/internal/family.cc
    src/metrics/internal/histogram.cc
    ## src/metrics/internal/metrics_test.cc # Remove to avoid fail with gtest
    src/msg_conversion.cc
    src/node.cc
    src/node_constants.cc
    src/node_options.cc
    src/playable_bag.cc # Use of rosbag
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

set(LIBS
    ${ABSEIL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${CAIRO_LIBRARIES}
    ${CARTOGRAPHER_LIBRARIES}
    # ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
    ${LUA_LIBRARIES}
    ${PCL_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    gflags
    glog
    tinyxml
    png16
    # Qt5::Core
    # Qt5::Widgets
)

add_library(${PROJECT_NAME} ${SRC_FILES})

target_link_libraries(${PROJECT_NAME} PUBLIC 
  # ${LIBS}
  ${CARTOGRAPHER_LIBRARIES}
  ${ABSEIL_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${CERES_LIBRARIES}
  ${LUA_LIBRARIES}
  ${CAIRO_LIBRARIES}
  glog
  gflags
  png16
)

# Lua
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${LUA_INCLUDE_DIR})
# PCL
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCL_LIBRARIES})
set(BLACKLISTED_PCL_DEFINITIONS " -march=native -msse4.2 -mfpmath=sse ")
foreach(DEFINITION ${PCL_DEFINITIONS})
  list (FIND BLACKLISTED_PCL_DEFINITIONS "${DEFINITION}" DEFINITIONS_INDEX)
  if (${DEFINITIONS_INDEX} GREATER -1)
    continue()
  endif()
  set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${DEFINITION}")
endforeach()
# Eigen
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "${EIGEN3_INCLUDE_DIR}")
# YAML
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${YAMLCPP_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${YAMLCPP_LIBRARIES})

# URDFDOM
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${urdfdom_headers_INCLUDE_DIRS})

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# set(RVIZ_SRC_FILES
#     include/${PROJECT_NAME}/rviz/drawable_submap.h include/${PROJECT_NAME}/rviz/ogre_slice.h
#     include/${PROJECT_NAME}/rviz/submaps_display.h src/rviz/drawable_submap.cc src/rviz/ogre_slice.cc
#     src/rviz/submaps_display.cc
# )

# qt5_wrap_cpp(RVIZ_SRC_FILES_MOC ${RVIZ_SRC_FILES})
# add_library(cartographer_rviz_plugin ${RVIZ_SRC_FILES_MOC} ${RVIZ_SRC_FILES})

add_executable(cartographer_node
  src/node_main.cc)
target_include_directories(cartographer_node SYSTEM PUBLIC ${LUA_INCLUDE_DIR})
target_link_libraries(cartographer_node ${PROJECT_NAME})
ament_target_dependencies(cartographer_node
  "cartographer_ros_msgs"
  "geometry_msgs"
  "nav_msgs"
  "pcl_conversions"
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "tf2"
  "tf2_msgs"
  "tf2_ros"
  "visualization_msgs"
  "std_srvs"
)

add_executable(occupancy_grid_node
  src/dev/occupancy_grid_node_main.cc)
target_include_directories(occupancy_grid_node SYSTEM PUBLIC ${LUA_INCLUDE_DIR})
target_link_libraries(occupancy_grid_node ${PROJECT_NAME})
ament_target_dependencies(occupancy_grid_node
  "cartographer_ros_msgs"
  "nav_msgs"
  "rclcpp"
  "std_msgs"
)

add_executable(cartographer_assets_writer src/dev/assets_writer_main.cc src/ros_map_writing_points_processor.cc)
target_link_libraries(cartographer_assets_writer ${PROJECT_NAME} cairo)
ament_target_dependencies(cartographer_assets_writer
  "rclcpp"
  "rosbag2_cpp"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "urdf"
)

# Linking problem with abseil
# add_executable(cartographer_offline_node src/dev/offline_node_main.cc)
# target_link_libraries(cartographer_offline_node ${PROJECT_NAME} ${LIBS})
# ament_target_dependencies(cartographer_offline_node
#   "cartographer_ros_msgs"
#   "nav_msgs"
#   "rclcpp"
#   "rosbag2_cpp"
#   "rosbag2_storage"
#   "std_srvs"
#   "tf2"
#   "tf2_ros"
#   "urdf"
#   "visualization_msgs"
# )

# Depends on bagfiles
add_executable(${PROJECT_NAME}bag_validate src/dev/rosbag_validate_main.cc )
target_link_libraries(${PROJECT_NAME}bag_validate ${PROJECT_NAME} ${LIBS})
ament_target_dependencies(${PROJECT_NAME}bag_validate
  "nav_msgs"
  "rclcpp"
  "rosbag2_cpp"
  "sensor_msgs"
  "tf2_msgs"
)

# Linking problems with abseil
# add_executable(cartographer_pbstream_to_ros_map src/dev/pbstream_to_ros_map_main.cc)
# target_link_libraries(cartographer_pbstream_to_ros_map PUBLIC ${PROJECT_NAME} cairo)

add_executable(cartographer_pbstream_map_publisher src/dev/pbstream_map_publisher_main.cc)
target_link_libraries(cartographer_pbstream_map_publisher ${PROJECT_NAME} ${LIBS})
ament_target_dependencies(cartographer_pbstream_map_publisher
  "rclcpp"
  "nav_msgs"
)

# Depends on bagfiles
# add_executable(cartographer_dev_pbstream_trajectories_to_rosbag src/dev/pbstream_trajectories_to_rosbag_main.cc)
# target_link_libraries(cartographer_dev_pbstream_trajectories_to_rosbag PUBLIC ${PROJECT_NAME} ${LIBS})

# Depends on bagfiles
# add_executable(cartographer_dev_rosbag_publisher src/dev/rosbag_publisher_main.cc)
# target_link_libraries(cartographer_dev_rosbag_publisher PUBLIC ${PROJECT_NAME} ${LIBS})

# Depends on bagfiles
# add_executable(cartographer_dev_trajectory_comparison src/dev/trajectory_comparison_main.cc)
# target_link_libraries(cartographer_dev_trajectory_comparison PUBLIC ${PROJECT_NAME} ${LIBS})

# Depends on bagfiles
# add_executable(test_find_constraint src/dev/test_find_constraint.cc)
# target_link_libraries(test_find_constraint PUBLIC ${PROJECT_NAME} ${LIBS})

# Linking problem with boost
# add_executable(sdf_to_pbstream src/dev/sdf_to_pbstream.cc)
# target_link_libraries(sdf_to_pbstream ${PROJECT_NAME} ${ABSEIL_LIBRARIES} ${Boost_LIBRARIES} tinyxml)
# ament_target_dependencies(sdf_to_pbstream
#   "cartographer_ros_msgs"
#   "nav_msgs"
#   "rclcpp"
#   "std_msgs"
# )

install(TARGETS
  cartographer_node
  occupancy_grid_node
  cartographer_pbstream_map_publisher
  DESTINATION lib/${PROJECT_NAME})

# target_link_libraries(${PROJECT_NAME} PUBLIC ${LIBS})
# target_link_libraries(cartographer_rviz_plugin PUBLIC ${PROJECT_NAME} ${LIBS})

# # if(CATKIN_ENABLE_TESTING)
# #     foreach(TEST_SOURCE_FILENAME ${ALL_TESTS})
# #         get_filename_component(TEST_NAME ${TEST_SOURCE_FILENAME} NAME_WE)
# #         catkin_add_gtest(${TEST_NAME} ${TEST_SOURCE_FILENAME})
# #         add_dependencies(${TEST_NAME} ${catkin_EXPORTED_TARGETS})
# #         target_link_libraries(${TEST_NAME} ${PROJECT_NAME})
# #     endforeach()
# # endif()

# install(
#     TARGETS
#         cartographer_assets_writer
#         cartographer_dev_pbstream_trajectories_to_rosbag
#         cartographer_dev_rosbag_publisher
#         cartographer_dev_trajectory_comparison
#         cartographer_node
#         cartographer_occupancy_grid_node
#         cartographer_offline_node
#         cartographer_pbstream_map_publisher
#         cartographer_pbstream_to_ros_map
#         ${PROJECT_NAME}
#         # ${PROJECT_NAME}bag_validate
#         cartographer_rviz_plugin
#         sdf_to_pbstream
#         test_find_constraint
#     ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(FILES rviz_plugin_description.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# install(DIRECTORY ogre_media DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

ament_package()

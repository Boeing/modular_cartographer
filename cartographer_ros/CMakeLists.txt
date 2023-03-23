cmake_minimum_required(VERSION 3.8)

project(cartographer_ros)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17) # Replaced 14 for 17 for is_same_v compatibility
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ~~~
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
set(ABSEIL_INSTALL_DIR ${ABSEIL_PROJECT_BUILD_DIR}/install)

set(ABSEIL_LIBRARIES
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/synchronization/${CMAKE_STATIC_LIBRARY_PREFIX}absl_synchronization${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/synchronization/${CMAKE_STATIC_LIBRARY_PREFIX}absl_graphcycles_internal${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/strings/${CMAKE_STATIC_LIBRARY_PREFIX}absl_cordz_functions${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/strings/${CMAKE_STATIC_LIBRARY_PREFIX}absl_cord_internal${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/strings/${CMAKE_STATIC_LIBRARY_PREFIX}absl_cord${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/strings/${CMAKE_STATIC_LIBRARY_PREFIX}absl_cordz_info${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/strings/${CMAKE_STATIC_LIBRARY_PREFIX}absl_strings_internal${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/strings/${CMAKE_STATIC_LIBRARY_PREFIX}absl_cordz_handle${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/strings/${CMAKE_STATIC_LIBRARY_PREFIX}absl_str_format_internal${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/strings/${CMAKE_STATIC_LIBRARY_PREFIX}absl_strings${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/strings/${CMAKE_STATIC_LIBRARY_PREFIX}absl_cordz_sample_token${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/base/${CMAKE_STATIC_LIBRARY_PREFIX}absl_base${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/base/${CMAKE_STATIC_LIBRARY_PREFIX}absl_scoped_set_env${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/base/${CMAKE_STATIC_LIBRARY_PREFIX}absl_raw_logging_internal${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/base/${CMAKE_STATIC_LIBRARY_PREFIX}absl_malloc_internal${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/base/${CMAKE_STATIC_LIBRARY_PREFIX}absl_log_severity${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/base/${CMAKE_STATIC_LIBRARY_PREFIX}absl_strerror${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/base/${CMAKE_STATIC_LIBRARY_PREFIX}absl_throw_delegate${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/base/${CMAKE_STATIC_LIBRARY_PREFIX}absl_spinlock_wait${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/status/${CMAKE_STATIC_LIBRARY_PREFIX}absl_statusor${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/status/${CMAKE_STATIC_LIBRARY_PREFIX}absl_status${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/time/${CMAKE_STATIC_LIBRARY_PREFIX}absl_time${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/time/${CMAKE_STATIC_LIBRARY_PREFIX}absl_civil_time${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/time/${CMAKE_STATIC_LIBRARY_PREFIX}absl_time_zone${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/hash/${CMAKE_STATIC_LIBRARY_PREFIX}absl_hash${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/hash/${CMAKE_STATIC_LIBRARY_PREFIX}absl_city${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/hash/${CMAKE_STATIC_LIBRARY_PREFIX}absl_low_level_hash${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/flags/${CMAKE_STATIC_LIBRARY_PREFIX}absl_flags_parse${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/flags/${CMAKE_STATIC_LIBRARY_PREFIX}absl_flags_usage${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/flags/${CMAKE_STATIC_LIBRARY_PREFIX}absl_flags_program_name${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/flags/${CMAKE_STATIC_LIBRARY_PREFIX}absl_flags_reflection${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/flags/${CMAKE_STATIC_LIBRARY_PREFIX}absl_flags_internal${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/flags/${CMAKE_STATIC_LIBRARY_PREFIX}absl_flags_marshalling${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/flags/${CMAKE_STATIC_LIBRARY_PREFIX}absl_flags_commandlineflag_internal${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/flags/${CMAKE_STATIC_LIBRARY_PREFIX}absl_flags_commandlineflag${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/flags/${CMAKE_STATIC_LIBRARY_PREFIX}absl_flags_config${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/flags/${CMAKE_STATIC_LIBRARY_PREFIX}absl_flags_private_handle_accessor${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/flags/${CMAKE_STATIC_LIBRARY_PREFIX}absl_flags${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/flags/${CMAKE_STATIC_LIBRARY_PREFIX}absl_flags_usage_internal${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/types/${CMAKE_STATIC_LIBRARY_PREFIX}absl_bad_variant_access${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/types/${CMAKE_STATIC_LIBRARY_PREFIX}absl_bad_optional_access${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/types/${CMAKE_STATIC_LIBRARY_PREFIX}absl_bad_any_cast_impl${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/numeric/${CMAKE_STATIC_LIBRARY_PREFIX}absl_int128${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/debugging/${CMAKE_STATIC_LIBRARY_PREFIX}absl_symbolize${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/debugging/${CMAKE_STATIC_LIBRARY_PREFIX}absl_demangle_internal${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/debugging/${CMAKE_STATIC_LIBRARY_PREFIX}absl_stacktrace${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/debugging/${CMAKE_STATIC_LIBRARY_PREFIX}absl_examine_stack${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/debugging/${CMAKE_STATIC_LIBRARY_PREFIX}absl_debugging_internal${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/debugging/${CMAKE_STATIC_LIBRARY_PREFIX}absl_failure_signal_handler${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/debugging/${CMAKE_STATIC_LIBRARY_PREFIX}absl_leak_check${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/profiling/${CMAKE_STATIC_LIBRARY_PREFIX}absl_periodic_sampler${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/profiling/${CMAKE_STATIC_LIBRARY_PREFIX}absl_exponential_biased${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/container/${CMAKE_STATIC_LIBRARY_PREFIX}absl_hashtablez_sampler${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/container/${CMAKE_STATIC_LIBRARY_PREFIX}absl_raw_hash_set${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/random/${CMAKE_STATIC_LIBRARY_PREFIX}absl_random_seed_sequences${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/random/${CMAKE_STATIC_LIBRARY_PREFIX}absl_random_internal_randen_hwaes_impl${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/random/${CMAKE_STATIC_LIBRARY_PREFIX}absl_random_internal_randen_hwaes${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/random/${CMAKE_STATIC_LIBRARY_PREFIX}absl_random_seed_gen_exception${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/random/${CMAKE_STATIC_LIBRARY_PREFIX}absl_random_internal_platform${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/random/${CMAKE_STATIC_LIBRARY_PREFIX}absl_random_internal_pool_urbg${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/random/${CMAKE_STATIC_LIBRARY_PREFIX}absl_random_internal_seed_material${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/random/${CMAKE_STATIC_LIBRARY_PREFIX}absl_random_internal_randen_slow${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/random/${CMAKE_STATIC_LIBRARY_PREFIX}absl_random_internal_randen${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/random/${CMAKE_STATIC_LIBRARY_PREFIX}absl_random_distributions${CMAKE_STATIC_LIBRARY_SUFFIX}"
    "${ABSEIL_PROJECT_BUILD_DIR}/absl/random/${CMAKE_STATIC_LIBRARY_PREFIX}absl_random_internal_distribution_test_util${CMAKE_STATIC_LIBRARY_SUFFIX}"
)
message(STATUS "${ABSEIL_LIBRARIES}")

set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)

externalproject_add(
    ${ABSEIL_PROJECT_NAME}
    PREFIX ${ABSEIL_PROJECT_NAME}
    GIT_REPOSITORY https://github.com/abseil/abseil-cpp
    GIT_TAG 215105818dfde3174fe799600bb0f3cae233d0bf # 20211102.0
    SOURCE_DIR ${ABSEIL_PROJECT_SRC_DIR}
    BINARY_DIR ${ABSEIL_PROJECT_BUILD_DIR}
    INSTALL_COMMAND ""
    CMAKE_ARGS
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON
        -DBUILD_TESTING=OFF
        -DCMAKE_BUILD_TYPE=Release
        -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
        -DCMAKE_CXX_STANDARD=17
        CMAKE_CXX_STANDARD=17
        -DABSL_PROPAGATE_CXX_STD:BOOL=ON
    BUILD_BYPRODUCTS "${ABSEIL_LIBRARIES}"
    BUILD_ALWAYS ON
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
        GIT_REPOSITORY https://gitlab.com/mtdi/pi9419/boeing/cartographer.git
        GIT_TAG fix_cpp_version
        GIT_SHALLOW ON
        UPDATE_COMMAND ""
        DEPENDS ${PROTOBUF_PROJECT_NAME} ${ABSEIL_PROJECT_NAME}
        CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=install -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER} -DCMAKE_CXX_STANDARD=17
                   CMAKE_CXX_STANDARD=17
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

# enable_testing() set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    ament_cmake
    cartographer_ros_msgs
    geometry_msgs
    nav_msgs
    pcl_conversions
    rclcpp
    sensor_msgs
    std_msgs
    tf2
    tf2_ros
    tf2_eigen
    tf2_msgs
    visualization_msgs
    rosbag2_cpp
    gflags
    std_srvs
    urdf
)

foreach(dep_pkg ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${dep_pkg} REQUIRED)
endforeach()

find_package(PCL REQUIRED COMPONENTS common)
find_package(Lua REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(urdfdom_headers REQUIRED)

if(DEFINED urdfdom_headers_VERSION)
    if(${urdfdom_headers_VERSION} GREATER 0.4.1)
        add_definitions(-DURDFDOM_HEADERS_HAS_SHARED_PTR_DEFS)
    endif()
endif()


find_package(PkgConfig)
# include(FindPkgConfig)

pkg_search_module(CAIRO REQUIRED cairo>=1.12.16)

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
    ${std_srvs_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${tf2_msgs_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_eigen_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${ros2bag_cpp_INCLUDE_DIRS}
    # From previous CMakeLists
    SYSTEM
    ${ABSEIL_INCLUDE_DIRS}
    # ${Boost_INCLUDE_DIRS}
    ${CAIRO_INCLUDE_DIRS}
    ${CARTOGRAPHER_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${LUA_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${PROTOBUF_INCLUDE_DIRS}
    ${urdfdom_headers_INCLUDE_DIRS}
    # ${Qt5Widgets_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS}
)

set(SRC_FILES
    src/assets_writer.cc # Use of rosbag
    src/map_builder_bridge.cc
    src/metrics/family_factory.cc
    src/metrics/internal/family.cc
    src/metrics/internal/histogram.cc
    # src/metrics/internal/metrics_test.cc # Remove to avoid fail with gtest
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
    ${CARTOGRAPHER_LIBRARIES}
    ${ABSEIL_LIBRARIES}
    # ${Boost_LIBRARIES}
    ${CAIRO_LIBRARIES}
    # ${catkin_LIBRARIES}
    ${CERES_LIBRARIES}
    ${LUA_LIBRARIES}
    ${PCL_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    gflags
    glog
    tinyxml
    png16
    # Qt5::Core Qt5::Widgets
)

add_library(${PROJECT_NAME} ${SRC_FILES})

target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

# Add dependency to cartographer external_project
add_dependencies(${PROJECT_NAME} ${CARTOGRAPHER_PROJECT_NAME})

###################################


# Lua
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${LUA_INCLUDE_DIR})
# PCL
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
set(BLACKLISTED_PCL_DEFINITIONS " -march=native -msse4.2 -mfpmath=sse ")
foreach(DEFINITION ${PCL_DEFINITIONS})
    list(FIND BLACKLISTED_PCL_DEFINITIONS "${DEFINITION}" DEFINITIONS_INDEX)
    if(${DEFINITIONS_INDEX} GREATER -1)
        continue()
    endif()
    set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${DEFINITION}")
endforeach()
# Eigen
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${EIGEN3_INCLUDE_DIR}")
# YAML
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${YAMLCPP_INCLUDE_DIRS})

# URDFDOM
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${urdfdom_headers_INCLUDE_DIRS})

target_include_directories(
    ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}> $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
                           $<INSTALL_INTERFACE:include>
)
#########################################


set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

ament_target_dependencies(${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  ${PCL_LIBRARIES}
  ${YAMLCPP_LIBRARIES}
)

ament_export_dependencies(${CARTOGRAPHER_PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})


install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)


##########################################################
## End target PROJECT_NAME
##########################################################

##########################################################
# RVIZ plugin

# set(OGRE_INCLUDE_DIRS /usr/include/OGRE)

# set(RVIZ_SRC_FILES include/${PROJECT_NAME}/rviz/drawable_submap.h include/${PROJECT_NAME}/rviz/ogre_slice.h
# include/${PROJECT_NAME}/rviz/submaps_display.h src/rviz/drawable_submap.cc src/rviz/ogre_slice.cc
# src/rviz/submaps_display.cc )

# qt5_wrap_cpp(RVIZ_SRC_FILES_MOC ${RVIZ_SRC_FILES}) add_library(cartographer_rviz_plugin ${RVIZ_SRC_FILES})
# target_include_directories(cartographer_rviz_plugin SYSTEM PUBLIC ${OGRE_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS})

##########################################################
# cartographer_node
##########################################################


add_executable(cartographer_node src/node_main.cc)
target_include_directories(cartographer_node SYSTEM PUBLIC ${LUA_INCLUDE_DIR})
target_link_libraries(cartographer_node ${PROJECT_NAME})
ament_target_dependencies(
    cartographer_node
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

ament_package()

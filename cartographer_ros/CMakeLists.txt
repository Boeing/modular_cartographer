cmake_minimum_required(VERSION 2.8.12)

project(cartographer_ros)

include(ExternalProject)

set(PROTOBUF_PROJECT_NAME protobuf)
set(PROTOBUF_PROJECT_SRC_DIR ${CMAKE_CURRENT_BINARY_DIR}/${PROTOBUF_PROJECT_NAME}/src/${PROTOBUF_PROJECT_NAME})
set(PROTOBUF_PROJECT_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/${PROTOBUF_PROJECT_NAME}/src/${PROTOBUF_PROJECT_NAME}-build)
set(PROTOBUF_INSTALL_DIR ${PROTOBUF_PROJECT_BUILD_DIR}/install)
set(PROTOBUF_INCLUDE_DIRS ${PROTOBUF_INSTALL_DIR}/include)
set(PROTOBUF_LIBRARIES ${PROTOBUF_INSTALL_DIR}/lib/${CMAKE_STATIC_LIBRARY_PREFIX}protobuf${CMAKE_STATIC_LIBRARY_SUFFIX})
ExternalProject_Add(${PROTOBUF_PROJECT_NAME}
    PREFIX ${PROTOBUF_PROJECT_NAME}
    URL ${CMAKE_CURRENT_SOURCE_DIR}/dependencies/protobuf-3.4.1.tar.gz
    UPDATE_COMMAND ""
    SOURCE_SUBDIR "cmake"
    CMAKE_ARGS -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_BUILD_TYPE=Release -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_INSTALL_PREFIX=install
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
ExternalProject_Add(${CARTOGRAPHER_PROJECT_NAME}
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
    cartographer_ros_msgs
    eigen_conversions
    geometry_msgs
    message_runtime
    nav_msgs
    pcl_conversions
    rosbag
    roscpp
    roslib
    sensor_msgs
    std_msgs
    tf2
    tf2_eigen
    tf2_ros
    urdf
    visualization_msgs
    rviz
)


find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

find_package(Boost REQUIRED COMPONENTS system iostreams)
find_package(PCL REQUIRED COMPONENTS common)

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/functions.cmake")

option(BUILD_GRPC "build features that require Cartographer gRPC support" false)

google_initialize_cartographer_project()
google_enable_testing()

include(FindPkgConfig)
PKG_SEARCH_MODULE(CAIRO REQUIRED cairo>=1.12.16)

find_package(Abseil REQUIRED)
find_package(LuaGoogle REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(LuaGoogle REQUIRED)

find_package(urdfdom_headers REQUIRED)
if(DEFINED urdfdom_headers_VERSION)
  if(${urdfdom_headers_VERSION} GREATER 0.4.1)
    add_definitions(-DURDFDOM_HEADERS_HAS_SHARED_PTR_DEFS)
  endif()
endif()

include_directories(
    ${urdfdom_headers_INCLUDE_DIRS}
    ${ABSEIL_INCLUDE_DIRS}
    ${CARTOGRAPHER_INCLUDE_DIRS}
    ${PROTOBUF_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${LUA_INCLUDE_DIR}
    ${CAIRO_INCLUDE_DIRS}
)

# Override Catkin's GTest configuration to use GMock.
set(GTEST_FOUND TRUE)
set(GTEST_INCLUDE_DIRS ${GMOCK_INCLUDE_DIRS})
set(GTEST_LIBRARIES ${GMOCK_LIBRARIES})

catkin_package(
    CATKIN_DEPENDS
        ${PACKAGE_DEPENDENCIES}
    DEPENDS
        PCL
        EIGEN3
        Boost
        urdfdom_headers
        ABSEIL
    INCLUDE_DIRS "."
    LIBRARIES ${PROJECT_NAME}
)

file(GLOB_RECURSE ALL_SRCS "cartographer_ros/*.cc" "cartographer_ros/*.h")
file(GLOB_RECURSE ALL_TESTS "cartographer_ros/*_test.cc")
file(GLOB_RECURSE ALL_EXECUTABLES "cartographer_ros/*_main.cc")
file(GLOB_RECURSE ALL_GRPC_FILES "cartographer_ros/cartographer_grpc/*")
list(REMOVE_ITEM ALL_SRCS ${ALL_TESTS})
list(REMOVE_ITEM ALL_SRCS ${ALL_EXECUTABLES})
if (NOT ${BUILD_GRPC})
  list(REMOVE_ITEM ALL_SRCS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM ALL_TESTS ${ALL_GRPC_FILES})
  list(REMOVE_ITEM ALL_EXECUTABLES ${ALL_GRPC_FILES})
endif()

add_library(${PROJECT_NAME} STATIC ${ALL_SRCS})
add_dependencies(${PROJECT_NAME} cartographer)
add_subdirectory("cartographer_ros")

target_link_libraries(${PROJECT_NAME}
    PUBLIC
    ${CARTOGRAPHER_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    standalone_absl
    glog
    gflags
    ${CERES_LIBRARIES}
    ${LUA_LIBRARIES}
    ${CAIRO_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${LUA_INCLUDE_DIR})

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

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${EIGEN3_INCLUDE_DIR}")

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${Boost_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME} PUBLIC ${Boost_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

if (CATKIN_ENABLE_TESTING)
  foreach(TEST_SOURCE_FILENAME ${ALL_TESTS})
    get_filename_component(TEST_NAME ${TEST_SOURCE_FILENAME} NAME_WE)
    catkin_add_gtest(${TEST_NAME} ${TEST_SOURCE_FILENAME})
    # catkin_add_gtest uses a plain (i.e. no PUBLIC/PRIVATE/INTERFACE) call to
    # target_link_libraries. That forces us to do the same.
    target_include_directories(${TEST_NAME} SYSTEM PUBLIC ${LUA_INCLUDE_DIR})
    target_link_libraries(${TEST_NAME} ${LUA_LIBRARIES})
    target_include_directories(${TEST_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
    target_link_libraries(${TEST_NAME} ${catkin_LIBRARIES})
    add_dependencies(${TEST_NAME} ${catkin_EXPORTED_TARGETS})

    target_link_libraries(${TEST_NAME}
        ${CARTOGRAPHER_LIBRARIES}
        ${PROTOBUF_LIBRARIES}
        standalone_absl
        glog
        gflags
        ${CERES_LIBRARIES}
        ${LUA_LIBRARIES}
        ${CAIRO_LIBRARIES}
    )

    target_link_libraries(${TEST_NAME} ${PROJECT_NAME})
    set_target_properties(${TEST_NAME} PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})
    # Needed for dynamically linked GTest on Windows.
    if (WIN32)
      target_compile_definitions(${TEST_NAME} PUBLIC -DGTEST_LINKED_AS_SHARED_LIBRARY)
    endif()
  endforeach()
endif()

install(DIRECTORY launch urdf configuration_files
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/tf_remove_frames.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Install source headers.
file(GLOB_RECURSE HDRS "cartographer_ros/*.h")
foreach(HDR ${HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(INSTALL_DIR ${REL_FIL} DIRECTORY)
  install(
    FILES
      ${HDR}
    DESTINATION
      include/${INSTALL_DIR}
  )
endforeach()

set(PROJECT_NAME_RVIZ cartographer_rviz)
file(GLOB_RECURSE ALL_SRCS_RVIZ "cartographer_rviz/*.cc" "cartographer_rviz/*.h")
set(CMAKE_AUTOMOC ON)

message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Core Qt5::Widgets)
include_directories(${Qt5Widgets_INCLUDE_DIRS})

add_definitions(-DQT_NO_KEYWORDS)
add_library(${PROJECT_NAME_RVIZ} ${ALL_SRCS_RVIZ})
target_link_libraries(${PROJECT_NAME_RVIZ} PUBLIC ${QT_LIBRARIES})

target_include_directories(${PROJECT_NAME_RVIZ} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(${PROJECT_NAME_RVIZ} SYSTEM PUBLIC "${EIGEN3_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME_RVIZ} PUBLIC ${EIGEN3_LIBRARIES})

target_include_directories(${PROJECT_NAME_RVIZ} SYSTEM PUBLIC "${Boost_INCLUDE_DIRS}")
target_link_libraries(${PROJECT_NAME_RVIZ} PUBLIC ${Boost_LIBRARIES})

target_include_directories(${PROJECT_NAME_RVIZ} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME_RVIZ} PUBLIC ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME_RVIZ} ${catkin_EXPORTED_TARGETS})

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
set_target_properties(${PROJECT_NAME_RVIZ} PROPERTIES COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

target_link_libraries(${PROJECT_NAME_RVIZ} PUBLIC ${CARTOGRAPHER_LIBRARIES} standalone_absl)

install(TARGETS ${PROJECT_NAME_RVIZ}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES rviz_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY ogre_media
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


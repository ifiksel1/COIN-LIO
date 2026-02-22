cmake_minimum_required(VERSION 3.10)
project(coin_lio)

SET(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions -fdiagnostics-color=always")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ARM64 (Jetson Orin) optimizations
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(aarch64)|(ARM64)")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8.2-a+fp16 -mtune=native")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv8.2-a+fp16 -mtune=native")
  message("ARM64 detected: enabling armv8.2-a+fp16, tuning for native CPU")
endif()

# Multi-processing: enabled on all platforms with sufficient cores
message("Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")
include(ProcessorCount)
ProcessorCount(N)
message("Processor count: ${N}")
if(N GREATER 4)
  add_definitions(-DMP_EN)
  add_definitions(-DMP_PROC_NUM=3)
  message("Multi-processing enabled: 3 cores")
elseif(N GREATER 3)
  add_definitions(-DMP_EN)
  add_definitions(-DMP_PROC_NUM=2)
  message("Multi-processing enabled: 2 cores")
else()
  add_definitions(-DMP_PROC_NUM=1)
  message("Multi-processing disabled: insufficient cores")
endif()

find_package(OpenMP QUIET)
if(OpenMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  message("OpenMP found: ${OpenMP_CXX_FLAGS}")
else()
  message("OpenMP not found, continuing without it")
endif()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  diagnostic_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  message_generation
  eigen_conversions
  cv_bridge
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include)

add_library(${PROJECT_NAME}
  include/ikd-Tree/ikd_Tree.cpp
  src/preprocess.cpp
  src/image_processing.cpp
  src/imu_processing.cpp
  src/feature_manager.cpp
  src/timing.cpp
  src/use_ikfom.cpp
  src/projector.cpp
)

add_message_files(
  FILES
  Pose6D.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(coin_lio_mapping src/laserMapping.cpp)
target_link_libraries(coin_lio_mapping ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE ${PYTHON_INCLUDE_DIRS})

add_executable(coin_lio_calibration src/calibration.cpp)
target_link_libraries(coin_lio_calibration ${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(coin_lio_vision_bridge src/vision_bridge.cpp)
target_link_libraries(coin_lio_vision_bridge ${catkin_LIBRARIES})
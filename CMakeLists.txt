cmake_minimum_required(VERSION 2.8.3)
project(visual_livox)

SET(CMAKE_BUILD_TYPE "Debug")

#set(CMAKE_CXX_COMPILER "/usr/bin/clang++")

#set(CMAKE_C_COMPILER "/usr/bin/clang")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -std=c++0x -std=c++14 -fexceptions -Wno-unused-local-typedefs")

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver
  roslib
  geometry_msgs
  cv_bridge
  pcl_conversions
  visualization_msgs
  # tf2
  tf2_ros
  tf2_geometry_msgs
  # tf2_msgs
  message_filters
  # pcl_ros
  pcl_conversions
  nav_msgs
  )

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV 3.2 REQUIRED)
find_package(Ceres REQUIRED)
#find_package(OpenMP REQUIRED)
include_directories(
	${catkin_INCLUDE_DIRS} 
        ${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS
)

######################
### visual odometry
######################

file(GLOB source_files
    "src/livox_mapping/*.h"
    "src/*.h"
)

add_executable(visual_odometry
        ${source_files}
        src/imageFactory.cpp
        src/pointcloudFactory.cpp
        src/visualOdometry.cpp
        )

target_compile_options(visual_odometry PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(visual_odometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS})






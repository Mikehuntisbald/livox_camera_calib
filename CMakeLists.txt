cmake_minimum_required(VERSION 3.0.2)
project(livox_camera_calib)
#set(OpenCV_DIR /home/sam/opencv-installed/usr/local/share/OpenCV)
#list(APPEND CMAKE_PREFIX_PATH "/home/sam/opencv-installed/usr/local")

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
#SET(CMAKE_BUILD_TYPE "Debug")
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
#SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
#set(CMAKE_PREFIX_PATH /home/sam/opencv-installed/usr/local)
#list(APPEND CMAKE_PREFIX_PATH "/home/sam/opencv-installed/usr/local")
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads)
find_package(Ceres REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(lidar_camera_calib src/lidar_camera_calib.cpp include/lidar_camera_calib.hpp)
add_executable(lidar_camera_multi_calib src/lsd.cpp src/lidar_camera_multi_calib.cpp include/lidar_camera_calib.hpp)
add_executable(bag_to_pcd src/bag_to_pcd.cpp)
add_executable(visualize src/visualize.cpp)

message("OpenCV libraries: ${OpenCV_DIR}")
message("OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")

target_link_libraries(lidar_camera_calib 
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES}
)

target_link_libraries(lidar_camera_multi_calib 
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${CERES_LIBRARIES}
)

target_link_libraries(bag_to_pcd 
  ${catkin_LIBRARIES}  ${PCL_LIBRARIES}
)

target_link_libraries(visualize 
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES}
)
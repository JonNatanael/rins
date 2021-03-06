cmake_minimum_required(VERSION 2.8.3)
project(rins)

set (OpenCV_DIR /usr/share/OpenCV)
## Find system dependencies
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include(dlib/cmake)
set(CMAKE_BUILD_TYPE Release)

## Find catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  genmsg
  nodelet
  nodelet_topic_tools
  pcl_conversions
  pcl_msgs
  pluginlib
  rosbag
  rosconsole
  roscpp
  rospy
  roslib
  sensor_msgs
  std_msgs
  message_generation
  OpenCV
  cv_bridge
)

## Add include directories
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Add link directories
link_directories(
  ${Boost_LIBRARY_DIRS}
  ${Eigen_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)

link_libraries(${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_message_files(
  FILES
  StampedString.msg
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare the catkin package
catkin_package(
  LIBRARIES
    pcl_ros_filters
    pcl_ros_io
    pcl_ros_tf
  CATKIN_DEPENDS
    dynamic_reconfigure
    pcl_conversions
    pcl_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    message_runtime
  DEPENDS
    Boost
    Eigen
    PCL
)



add_executable (example src/example.cpp)

add_executable (detect src/detect.cpp)
TARGET_LINK_LIBRARIES(detect dlib ${catkin_LIBRARIES} ${OpenCV_LIBS})


install(PROGRAMS
        scripts/faces.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


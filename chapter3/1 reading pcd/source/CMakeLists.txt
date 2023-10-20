cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pcd_read)
find_package(PCL 1.10 REQUIRED)
find_package(catkin REQUIRED COMPONENTS sensor_msgs)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    pcl_conversions
    pcl_ros
    sensor_msgs
  # other dependencies
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(pcd_read pcd_read.cpp)
target_link_libraries(pcd_read 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
)

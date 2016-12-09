cmake_minimum_required(VERSION 2.8.3)
project(vulcano_base_gazebo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  gazebo_ros
  roscpp 
  message_generation 
  gazebo_msgs 
  nodelet 
  std_srvs 
  geometry_msgs 
  sensor_msgs 
  nav_msgs 
  urdf 
  tf     
)


find_package(Boost REQUIRED COMPONENTS system)

include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(omnidrive_plugin src/omnidrive_plugin.cpp)
target_link_libraries(omnidrive_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(
  INCLUDE_DIRS include
  DEPENDS 
    roscpp
    gazebo_ros
  )

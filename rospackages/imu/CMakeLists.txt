cmake_minimum_required(VERSION 2.8.3)
project(imu)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(catkin_deps rospy roslib std_msgs sensor_msgs)
find_package(catkin REQUIRED ${catkin_deps})
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include 
    CATKIN_DEPENDS ${catkin_deps}
    DEPENDS 
    LIBRARIES ${PROJECT_NAME} 
)



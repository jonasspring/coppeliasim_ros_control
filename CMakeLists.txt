cmake_minimum_required(VERSION 2.8.3)
project(coppeliasim_ros_control)

find_package(catkin REQUIRED COMPONENTS roscpp vrep_common vrep_skeleton_msg_and_srv controller_manager transmission_interface control_toolbox joint_limits_interface urdf hardware_interface
  franka_hw franka_simulation)

################################################
# List sources.
################################################
set(${PROJECT_NAME}_headers
    include/simConst.h
    include/simLib.h
    include/simTypes.h

    include/simExtRosControl_server.h
    include/simExtRosControl_plugin.h
    include/MyRobot_simHW.h
)

set(${PROJECT_NAME}_sources
    src/ControlLoop/simLib.cpp
    src/ControlLoop/simExtRosControl_server.cpp
    src/ControlLoop/simExtRosControl_plugin.cpp
    src/ControlLoop/MyRobot_simHW.cpp
)

find_package(Franka 0.7.0 REQUIRED)

catkin_package(
               CATKIN_DEPENDS roscpp rosconsole std_msgs roslib
               DEPENDS roscpp rosconsole std_msgs roslib Franka franka_simulation franka_hw)

include_directories(include ${catkin_INCLUDE_DIRS})

link_directories("/opt/ros/melodic/lib")

add_library(simExtRosControl ${${PROJECT_NAME}_headers} ${${PROJECT_NAME}_sources})

add_dependencies(simExtRosControl ${catkin_EXPORTED_TARGETS})
target_link_libraries(simExtRosControl
    pthread
    dl
    roslib
    rosconsole
    rostime
    rospack
    roscpp_serialization
    roscpp
    boost_system
   ${catkin_LIBRARIES}
)

target_link_libraries(simExtRosControl
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
  ${franka_simulation_LIBRARIES}
  ${franka_hw_LIBRARIES}
)

## Installation
#install(TARGETS simExtRosControl
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

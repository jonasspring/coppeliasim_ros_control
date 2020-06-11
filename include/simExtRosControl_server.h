#pragma once

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

// API services:
#include "vrep_skeleton_msg_and_srv/displayText.h"

namespace MR
{
class MyRobot_simHW;
}

namespace controller_manager
{
class ControllerManager;
}

namespace ros
{
class CallbackQueue;
}

class ROS_server
{
public:
    static bool initialize();
    static void shutDown();

    static void instancePass();
    static void mainScriptAboutToBeCalled();

    static void simulationAboutToStart();
    static void simulationEnded();

private:
    ROS_server() {} // Make this class non-instantiable.

    static ros::NodeHandle* sm_node;
    static void spinOnce();

    // Control.
    static ros::CallbackQueue * sm_rosControlCallbackQueue;
    static MR::MyRobot_simHW * sm_myRobotHw;
    static controller_manager::ControllerManager * sm_ctrlManager;
    static ros::AsyncSpinner * sm_spinner;
};

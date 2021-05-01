#include <iostream>
#include <ros/ros.h>

#include <quadruped_ctrl/QuadMPCAction.h>
#include <actionlib/client/simple_action_client.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "MPC Controller");

    actionlib::SimpleActionClient<quadruped_ctrl::QuadMPCAction.h> client("QuadMPC", true);

    client.waitForServer();

    quadruped_ctrl::QuadMPCAction ctrl;
    ctrl.

    return 0;
}


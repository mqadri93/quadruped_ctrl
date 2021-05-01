#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <sstream>

#include "GaitCtrller.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "adaptiveGaitMPC");
  ros::NodeHandle nh;
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(10);
  int count = 0;

  // GaitCtrller(double freq, double* PIDParam);
  double stand_kp, stand_kd, joint_kp, joint_kd;
  int freq;
  // getParam(nh, "my_param1", myParam1);
  ros::param::get("/simulation/freq", freq);
  ros::param::get("/simulation/stand_kp", stand_kp);
  ros::param::get("/simulation/stand_kd", stand_kd);
  ros::param::get("/simulation/joint_kp", joint_kp);
  ros::param::get("/simulation/joint_kd", joint_kd);

  double pid_coefficients[] = {stand_kp, stand_kd, joint_kp, joint_kd};
  auto cpp_gait_ctrller = GaitCtrller(nh, freq, pid_coefficients);

  while (ros::ok()) {
    std_msgs::String msg;
    cpp_gait_ctrller.pubDemo(count);

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}

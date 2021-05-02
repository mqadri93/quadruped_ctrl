#ifndef GAIT_CTRLLER_H
#define GAIT_CTRLLER_H

#include <math.h>
#include <zmq.hpp>
//#include <ros/ros.h>
//#include <std_msgs/Float32.h>
//#include <std_msgs/Float64MultiArray.h>
//#include <std_msgs/Int32.h>
//#include <std_msgs/String.h>
//#include <time.h>

#include <iostream>
#include <string>

#include "Controllers/ContactEstimator.h"
#include "Controllers/ControlFSMData.h"
#include "Controllers/DesiredStateCommand.h"
#include "Controllers/OrientationEstimator.h"
#include "Controllers/PositionVelocityEstimator.h"
#include "Controllers/RobotLegState.h"
#include "Controllers/SafetyChecker.h"
#include "Controllers/StateEstimatorContainer.h"
#include "Dynamics/MiniCheetah.h"
#include "MPC_Ctrl/ConvexMPCLocomotion.h"
#include "Utilities/IMUTypes.h"
#include "calculateTool.h"

struct JointEff {
  double eff[12];
};

class GaitCtrller {
 public:
  GaitCtrller(double freq, double* PIDParam);
  ~GaitCtrller();
  void SetIMUData(double* imuData);
  void SetLegData(double* motorData);
  void PreWork(double* imuData, double* motorData);
  void SetGaitType(int gaitType);
  void SetRobotMode(int mode);
  void SetRobotVel(double* vel);
  void ToqueCalculator(double* imuData, double* motorData, double* effort);
  /*
  void pubDemo(int val) {
    std_msgs::Float64MultiArray array;
    array.data.clear();
    // for (int k = 0; k < 4; k++) array.data.push_back(ctrlParam(k));
    for (int k = 0; k < 4; k++) array.data.push_back((k + 1) * val);
    pub_pid.publish(array);
    convexMPC->pubCvx(val);
  }
  */

 private:
  int _gaitType = 0;
  int _robotMode = 0;
  bool _safetyCheck = true;
  std::vector<double> _gamepadCommand;
  Vec4<float> ctrlParam;

  //ros::NodeHandle nh;
  //ros::Publisher pub_pid;

  Quadruped<float> _quadruped;
  ConvexMPCLocomotion* convexMPC;
  LegController<float>* _legController;
  StateEstimatorContainer<float>* _stateEstimator;
  LegData _legdata;
  LegCommand legcommand;
  ControlFSMData<float> control_data;
  VectorNavData _vectorNavData;
  CheaterState<double>* cheaterState;
  StateEstimate<float> _stateEstimate;
  RobotControlParameters* controlParameters;
  DesiredStateCommand<float>* _desiredStateCommand;
  SafetyChecker<float>* safetyChecker;
};

extern "C" {

GaitCtrller* gCtrller = NULL;
JointEff jointEff;

// first step, init the controller
/*
void init_controller(ros::NodeHandle& nh_, double freq, double PIDParam[]) {
  if (NULL != gCtrller) {
    delete gCtrller;
  }
  gCtrller = new GaitCtrller(nh_, freq, PIDParam);
}
*/

void init_controller(double freq, double PIDParam[]) {
  if (NULL != gCtrller) {
    delete gCtrller;
  }
  gCtrller = new GaitCtrller(freq, PIDParam);
}

// the kalman filter need to work second
void pre_work(double imuData[], double legData[]) {
  gCtrller->PreWork(imuData, legData);
}

// gait type can be set in any time
void set_gait_type(int gaitType) { gCtrller->SetGaitType(gaitType); }

// set robot mode, 0: High performance model, 1: Low power mode
void set_robot_mode(int mode) { gCtrller->SetRobotMode(mode); }

// robot vel can be set in any time
void set_robot_vel(double vel[]) { gCtrller->SetRobotVel(vel); }

// after init controller and pre work, the mpc calculator can work
JointEff* toque_calculator(double imuData[], double motorData[]) {
  double eff[12] = {0.0};
  gCtrller->ToqueCalculator(imuData, motorData, eff);
  for (int i = 0; i < 12; i++) {
    jointEff.eff[i] = eff[i];
  }
  return &jointEff;
}
}

#endif

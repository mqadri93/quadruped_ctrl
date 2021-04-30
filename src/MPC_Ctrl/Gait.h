#ifndef PROJECT_GAIT_H
#define PROJECT_GAIT_H

#include <string>
#include <queue>

#include "Utilities/cppTypes.h"
#include "MPC_Ctrl/utils.h"
#include "MPC_Ctrl/SwStXtd.h"
#include "Controllers/FootSwingTrajectory.h"
#include "Controllers/ControlFSMData.h"
#include "SparseCMPC.h"

class Gait {
public:
  virtual ~Gait() = default;

  virtual void setGaitParam(int nSegment, Vec4<int> offsets, Vec4<int> durations, const std::string& name = "walk") = 0;
  virtual Vec4<float> getContactState() = 0;
  virtual Vec4<float> getSwingState() = 0;
  virtual int* getMpcTable() = 0;
  virtual void setIterations(int iterationsBetweenMPC, int currentIteration) = 0;
  virtual float getCurrentStanceTime(float dtMPC, int leg) = 0;
  virtual float getCurrentSwingTime(float dtMPC, int leg) = 0;
  virtual float getCurrentGaitPhase() = 0;
  virtual int getGaitHorizon() = 0;
  virtual void debugPrint() { }
  std::vector<bool>  leg_command_in = {1, 1, 1, 1};
  std::vector<float> x_swingOnset = {0., 0., 0., 0.};
  std::vector<float> touchdown_pos_world = {0., 0., 0., 0.};
  float swingTimeRemaining_lookahead[4];
  int counter = 0;
  int h_mpc;
  float vb;
  float vx_des;
  Vec4<float> swingTimes;
  float dt;
  std::vector<float> x_fh;
  Sw_St_Xtd_out _gait;

protected:
  std::string _name;
};

using Eigen::Array4f;
using Eigen::Array4i;

class OffsetDurationGait : public Gait {
public:
  OffsetDurationGait(int nSegment, Vec4<int> offset, Vec4<int> durations, const std::string& name);
  ~OffsetDurationGait();
  void setGaitParam(int nSegment, Vec4<int> offset, Vec4<int> durations, const std::string& name);
  Vec4<float> getContactState();
  Vec4<float> getSwingState();
  int* getMpcTable();
  void setIterations(int iterationsBetweenMPC, int currentIteration);
  float getCurrentStanceTime(float dtMPC, int leg);
  float getCurrentSwingTime(float dtMPC, int leg);
  float getCurrentGaitPhase();
  int getGaitHorizon();
  void debugPrint();

private:
  int* _mpc_table = NULL;
  Array4i _offsets; // offset in mpc segments
  Array4i _durations; // duration of step in mpc segments
  Array4f _offsetsFloat; // offsets in phase (0 to 1)
  Array4f _durationsFloat; // durations in phase (0 to 1)
  // int _stance;
  // int _swing;
  int _iteration;
  int _nIterations;
  float _phase;
};



class MixedFrequncyGait : public Gait {
public:
  MixedFrequncyGait(int nSegment, Vec4<int> periods, float duty_cycle, const std::string& name);
  ~MixedFrequncyGait();
  Vec4<float> getContactState();
  Vec4<float> getSwingState();
  int* getMpcTable();
  void setIterations(int iterationsBetweenMPC, int currentIteration);
  float getCurrentStanceTime(float dtMPC, int leg);
  float getCurrentSwingTime(float dtMPC, int leg);
  float getCurrentGaitPhase();
  int getGaitHorizon();
  void debugPrint();

private:
  float _duty_cycle;
  int* _mpc_table;
  Array4i _periods;
  Array4f _phase;
  int _iteration;
  int _nIterations;
};

class AdaptiveGait : public Gait {
public:
  AdaptiveGait(int nSegment, Vec4<int> offset, Vec4<int> durations, const std::string& name);
  ~AdaptiveGait();
  void setGaitParam(int nSegment, Vec4<int> offset, Vec4<int> durations, const std::string& name);
  Vec4<float> getContactState();
  Vec4<float> getSwingState();
  int* getMpcTable();
  void setIterations(int iterationsBetweenMPC, int currentIteration);
  float getCurrentStanceTime(float dtMPC, int leg);
  float getCurrentSwingTime(float dtMPC, int leg);
  float getCurrentGaitPhase();
  int getGaitHorizon();
  void debugPrint();

private:
  int* _mpc_table = NULL;
  Array4i _offsets; // offset in mpc segments
  Array4i _durations; // duration of step in mpc segments
  Array4f _offsetsFloat; // offsets in phase (0 to 1)
  Array4f _durationsFloat; // durations in phase (0 to 1)
  // int _stance;
  // int _swing;
  int _iteration;
  int _nIterations;
  float _phase;
};


#endif //PROJECT_GAIT_H

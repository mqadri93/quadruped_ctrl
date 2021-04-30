#include "Gait.h"
#include <iostream>

// Offset - Duration Gait
OffsetDurationGait::OffsetDurationGait(int nSegment, Vec4<int> offsets, Vec4<int> durations, const std::string &name = "walk") :
  _offsets(offsets.array()),
  _durations(durations.array()),
  _nIterations(nSegment)  //10
{

  // _name = name;
  // // allocate memory for MPC gait table
  // _mpc_table = new int[nSegment * 4];

  // _offsetsFloat = offsets.cast<float>() / (float) nSegment;
  // _durationsFloat = durations.cast<float>() / (float) nSegment;

  // _stance = durations[0];
  // _swing = nSegment - durations[0];
  setGaitParam(nSegment, offsets, durations, name);
}

AdaptiveGait::AdaptiveGait(int nSegment, Vec4<int> offsets, Vec4<int> durations, const std::string &name = "walk") :
  _offsets(offsets.array()),
  _durations(durations.array()),
  _nIterations(nSegment)  //10
{
  setGaitParam(nSegment, offsets, durations, name);
}

void OffsetDurationGait::setGaitParam(int nSegment, Vec4<int> offsets, Vec4<int> durations, const std::string& name = "walk") {

  _offsets = offsets.array();
  _durations = durations.array();
  _nIterations = nSegment;

  _name = name;
  // allocate memory for MPC gait table
  if(NULL != _mpc_table) {
    delete[] _mpc_table;
  }
  _mpc_table = new int[nSegment * 4];

  _offsetsFloat = offsets.cast<float>() / (float) nSegment;
  _durationsFloat = durations.cast<float>() / (float) nSegment;

  // _stance = durations[0];
  // _swing = nSegment - durations[0];
}

MixedFrequncyGait::MixedFrequncyGait(int nSegment, Vec4<int> periods, float duty_cycle, const std::string &name) {
  _name = name;
  _duty_cycle = duty_cycle;
  _mpc_table = new int[nSegment * 4];
  _periods = periods;
  _nIterations = nSegment;
  _iteration = 0;
  _phase.setZero();
}

void AdaptiveGait::setGaitParam(int nSegment, Vec4<int> offsets, Vec4<int> durations, const std::string& name = "walk") {

  _offsets = offsets.array();
  _durations = durations.array();
  _nIterations = nSegment;

  _name = name;
  // allocate memory for MPC gait table
  if(NULL != _mpc_table) {
    delete[] _mpc_table;
  }
  _mpc_table = new int[nSegment * 4];

  _offsetsFloat = offsets.cast<float>() / (float) nSegment;
  _durationsFloat = durations.cast<float>() / (float) nSegment;
  
}

OffsetDurationGait::~OffsetDurationGait() {
  delete[] _mpc_table;
}

MixedFrequncyGait::~MixedFrequncyGait() {
  delete[] _mpc_table;
}

AdaptiveGait::~AdaptiveGait() {
  delete[] _mpc_table;
}

Vec4<float> OffsetDurationGait::getContactState() {
  Array4f progress = _phase - _offsetsFloat;

  for(int i = 0; i < 4; i++)
  {
    if(progress[i] < 0) progress[i] += 1.;
    if(progress[i] > _durationsFloat[i])
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / _durationsFloat[i];
    }
  }

  //printf("contact state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
  return progress.matrix();
}

Vec4<float> MixedFrequncyGait::getContactState() {
  Array4f progress = _phase;

  for(int i = 0; i < 4; i++) {
    if(progress[i] < 0) progress[i] += 1.;
    if(progress[i] > _duty_cycle) {
      progress[i] = 0.;
    } else {
      progress[i] = progress[i] / _duty_cycle;
    }
  }

  //printf("contact state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
  return progress.matrix();
}

Vec4<float> AdaptiveGait::getContactState() {
  Array4f progress = _phase - _offsetsFloat;

  for(int i = 0; i < 4; i++)
  {
    if(progress[i] < 0) progress[i] += 1.;
    if(progress[i] > _durationsFloat[i])
    {
      progress[i] = 0.;
    }
    else
    {
      progress[i] = progress[i] / _durationsFloat[i];
    }
  }

  //printf("contact state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
  return progress.matrix();
}


Vec4<float> OffsetDurationGait::getSwingState()
{
  Array4f swing_offset = _offsetsFloat + _durationsFloat;
  for(int i = 0; i < 4; i++)
    if(swing_offset[i] > 1) swing_offset[i] -= 1.;
  Array4f swing_duration = 1. - _durationsFloat;

  Array4f progress = _phase - swing_offset;

  for(int i = 0; i < 4; i++)
  {
    if(progress[i] < 0) progress[i] += 1.f;
    if(progress[i] > swing_duration[i])
    {
      progress[i] = 0.;
    }
    else
    {
      if(swing_duration[i] < 0.0000000001) {
        progress[i] = 0.0;
      } else {
        progress[i] = progress[i] / swing_duration[i];
      }
    }
  }
  return progress.matrix();
}

Vec4<float> MixedFrequncyGait::getSwingState() {

  float swing_duration = 1.f - _duty_cycle;
  Array4f progress = _phase - _duty_cycle;
  for(int i = 0; i < 4; i++) {
    if(progress[i] < 0) {
      progress[i] = 0;
    } else {
      progress[i] = progress[i] / swing_duration;
    }
  }

  //printf("swing state: %.3f %.3f %.3f %.3f\n", progress[0], progress[1], progress[2], progress[3]);
  return progress.matrix();
}

Vec4<float> AdaptiveGait::getSwingState()
{
  Array4f swing_offset = _offsetsFloat + _durationsFloat;
  for(int i = 0; i < 4; i++)
    if(swing_offset[i] > 1) swing_offset[i] -= 1.;
  Array4f swing_duration = 1. - _durationsFloat;

  Array4f progress = _phase - swing_offset;

  for(int i = 0; i < 4; i++)
  {
    if(progress[i] < 0) progress[i] += 1.f;
    if(progress[i] > swing_duration[i])
    {
      progress[i] = 0.;
    }
    else
    {
      if(swing_duration[i] < 0.0000000001) {
        progress[i] = 0.0;
      } else {
        progress[i] = progress[i] / swing_duration[i];
      }
    }
  }
  return progress.matrix();
}


int* OffsetDurationGait::getMpcTable()
{

  //printf("OffsetDurationGait::getMpcTable:\n");
  // printf("value is: %d", _nIterations);   _nIterations = 10
  for(int i = 0; i < _nIterations; i++)
  {
    int iter = (i + _iteration + 1) % _nIterations;
    Array4i progress = iter - _offsets;
    for(int j = 0; j < 4; j++)
    {
      if(progress[j] < 0) progress[j] += _nIterations;
      if(progress[j] < _durations[j])
        _mpc_table[i*4 + j] = 1;
      else
        _mpc_table[i*4 + j] = 0;

      //printf("%d ", _mpc_table[i*4 + j]);
    }
    //printf("\n");
  }
  //printf("\n");

  return _mpc_table;
}

int* MixedFrequncyGait::getMpcTable() {
  // printf("MPC table (%d):\n", _iteration);
  for(int i = 0; i < _nIterations; i++) {
    for(int j = 0; j < 4; j++) {
      int progress = (i + _iteration + 1) % _periods[j];  // progress
      if(progress < (_periods[j] * _duty_cycle)) {
        _mpc_table[i*4 + j] = 1;
      } else {
        _mpc_table[i*4 + j] = 0;
      }
      // printf("%d %d (%d %d) | ", _mpc_table[i*4 + j], progress, _periods[j], (int)(_periods[j] * _duty_cycle));
    }

    //printf("%d %d %d %d (%.3f %.3f %.3f %.3f)\n", _mpc_table[i*4], _mpc_table[i*4 + 1], _mpc_table[i*4 + ])
    //printf("\n");
  }
  return _mpc_table;
}

// This is just a placeholder to match the base class functions


int* AdaptiveGait::getMpcTable()
{
  int NUM_LEGS = 4;
  //float vx_des = gamepadCommand[0];

  for(int i=0; i<NUM_LEGS; i++){
    _mpc_table[i] = _gait.leg_command[i];
  }
  
  //float vb = _stateEstimator.getResult().vBody[0];
  Sw_St_Xtd_out _gait_mpc = _gait;
  
  //cout << "**************************" << endl;
  //cout << "leg command in MPC" << endl;
  //print_vector(_gait.leg_command);
  for(int iter = 1; iter < h_mpc; iter++) {
    leg_command_in = _gait_mpc.leg_command;

    for(int leg = 0; leg < NUM_LEGS; leg++) {
      if(_gait_mpc.leg_command[leg] == 1) {
        x_fh[leg] = x_fh[leg] - vb*dt;
      } else {
        x_fh[leg] = x_swingOnset[leg] + 
                    (_gait_mpc.x_td_out[leg]-x_swingOnset[leg])*swingTimeRemaining_lookahead[leg]/swingTimes[leg];
        if(x_fh[leg] >= _gait_mpc.x_td_out[leg])
          leg_command_in[leg] = 1;
      }
    }
    
    //Calls to this function are expensive. Needs to shorten the horizon time
    Sw_St_Xtd_out _gait_mpc = generate_data(x_fh, vb, vx_des, leg_command_in);
    for(int leg = 0; leg < NUM_LEGS; leg++) {
      if(_gait_mpc.swing_state_flag[leg]) {
        x_swingOnset[leg] = x_fh[leg];
        swingTimeRemaining_lookahead[leg] = swingTimes[leg];
      }
      if(_gait_mpc.leg_command[leg] == 0 && !_gait_mpc.swing_state_flag[leg]) {
        swingTimeRemaining_lookahead[leg] -= dt;
      }
      //cout << "i " << iter*NUM_LEGS + leg  << " " << _gait_mpc.leg_command[leg] << endl;;
      _mpc_table[iter*NUM_LEGS + leg] = _gait_mpc.leg_command[leg];
    }

  }

  return _mpc_table;
}

/*
int* AdaptiveGait::getAdaptiveMpcTable() {
  return _mpc_table;
}
*/

void OffsetDurationGait::setIterations(int iterationsPerMPC, int currentIteration)
{
  _iteration = (currentIteration / iterationsPerMPC) % _nIterations;
  _phase = (float)(currentIteration % (iterationsPerMPC * _nIterations)) / (float) (iterationsPerMPC * _nIterations);
  // std::cout << "currentIteration = " << currentIteration  << " iterationsPerMPC = " << iterationsPerMPC << std::endl;
  // std::cout << "_nIterations = " << _nIterations << " _iteration = " << _iteration << " _phase = " << _phase << std::endl;
}

void MixedFrequncyGait::setIterations(int iterationsBetweenMPC, int currentIteration) {
  _iteration = (currentIteration / iterationsBetweenMPC);// % _nIterations;
  for(int i = 0; i < 4; i++) {
    int progress_mult = currentIteration % (iterationsBetweenMPC * _periods[i]);
    _phase[i] = ((float)progress_mult) / ((float) iterationsBetweenMPC * _periods[i]);
    //_phase[i] = (float)(currentIteration % (iterationsBetweenMPC * _periods[i])) / (float) (iterationsBetweenMPC * _periods[i]);
  }

  //printf("phase: %.3f %.3f %.3f %.3f\n", _phase[0], _phase[1], _phase[2], _phase[3]);

}

void AdaptiveGait::setIterations(int iterationsPerMPC, int currentIteration)
{
  _iteration = (currentIteration / iterationsPerMPC) % _nIterations;
  _phase = (float)(currentIteration % (iterationsPerMPC * _nIterations)) / (float) (iterationsPerMPC * _nIterations);
  // std::cout << "currentIteration = " << currentIteration  << " iterationsPerMPC = " << iterationsPerMPC << std::endl;
  // std::cout << "_nIterations = " << _nIterations << " _iteration = " << _iteration << " _phase = " << _phase << std::endl;
}

float OffsetDurationGait::getCurrentGaitPhase() {
  return _phase;
}

float MixedFrequncyGait::getCurrentGaitPhase() {
  return 0;
}

float AdaptiveGait::getCurrentGaitPhase() {
  return _phase;
}


float OffsetDurationGait::getCurrentSwingTime(float dtMPC, int leg) {
  (void)leg;
  // return dtMPC * _swing;
  return dtMPC * (_nIterations - _durations[leg]);
}

float MixedFrequncyGait::getCurrentSwingTime(float dtMPC, int leg) {
  return dtMPC * (1. - _duty_cycle) * _periods[leg];
}

float AdaptiveGait::getCurrentSwingTime(float dtMPC, int leg) {
  (void)leg;
  // return dtMPC * _swing;
  return dtMPC * (_nIterations - _durations[leg]);
}


float OffsetDurationGait::getCurrentStanceTime(float dtMPC, int leg) {
  (void) leg;
  // return dtMPC * _stance;
  return dtMPC * _durations[leg];
}

float MixedFrequncyGait::getCurrentStanceTime(float dtMPC, int leg) {
  return dtMPC * _duty_cycle * _periods[leg];
}

float AdaptiveGait::getCurrentStanceTime(float dtMPC, int leg) {
  (void) leg;
  // return dtMPC * _stance;
  return dtMPC * _durations[leg];
}

int OffsetDurationGait::getGaitHorizon() {
  return _nIterations;
}

int MixedFrequncyGait::getGaitHorizon() {
  return _nIterations;
}

int AdaptiveGait::getGaitHorizon() {
  return _nIterations;
}

void OffsetDurationGait::debugPrint() {

}

void MixedFrequncyGait::debugPrint() {

}

void AdaptiveGait::debugPrint() {

}
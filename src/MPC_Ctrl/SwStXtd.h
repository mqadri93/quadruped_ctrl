#ifndef SWSTXTD_H_
#define SWSTXTD_H_
#include "utils.h"



/*
The things we need to find are:

struct Sw_St_Xtd_out {
	vector<bool> swing_state_flag;
	vector<bool> leg_command;
	vector<float> x_td_out;
};

Sw_St_Xtd_out generate_data(
	[ ] ecat_data: current foot positions relative to the hip (x position) aka in hip frame
	[x] vb: body velocity expressed in body frame (x velocity) 
	[ ] vx_des : comes from the joystick
	[ ] t : current time 
	[ ] leg_command_in: at time t: which legs are in stance and which are in swing (line 396 convexMPC)
)
*/

Sw_St_Xtd_out Sw_St_Xtd(std::map<std::string, FIA> pFIS_stack,
						std::map<int, FA> FFS_stack,
						vector<float> nodes,
						vector<float> ecat_data,
						int NUM_LEGS,
						float vb,
						float vx_des,
						float marginalUtility_threshold,
						float wkspace_margin,
						float x_swing, // user defined
						float x_td_nom, // user defined
						vector<float> x_td_old, // output of last call
						float internode_dx, // distance between 2 nodes
						int trouble,
						int t,
						vector<bool> leg_command_in, 
						int lesion,
						vector<bool> lesion_legs,
						vector<float> lesion_leg_xtd,
						vector<float> lesion_leg_xswing);

void load_data_FFS(std::map<int, FA> *FFS_stack);
void load_data_FIS_forward(std::map<string, FIA>* pFIS_stack);
void load_data_FIS_backward(std::map<string, FIA>* pFIS_stack) ;

void generate_data();

#endif
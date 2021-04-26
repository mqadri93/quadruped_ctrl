#ifndef SWSTXTD_H_
#define SWSTXTD_H_
#include "utils.h"

Sw_St_Xtd_out Sw_St_Xtd(std::map<std::string, FIA> pFIS_stack,
						std::map<int, FA> FFS_stack,
						vector<float> nodes,
						vector<float> ecat_data,
						int NUM_LEGS,
						float vb,
						float vx_des,
						float marginalUtility_threshold,
						float wkspace_margin,
						float x_swing,
						float x_td_nom,
						vector<float> x_td_old,
						float internode_dx,
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
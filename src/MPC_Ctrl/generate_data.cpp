#include "generate_data.h"



int main()
{
    /*
    cnpy::NpyArray pFIS_stack = cnpy::npy_load("../python_src/FFS_stack.npy");
    string row = to_string(pFIS_stack.shape[0]);
    string col = to_string(pFIS_stack.shape[1]);
    //int row=1;
    cout << pFIS_stack.shape[0] << endl;
    cout << pFIS_stack.shape[1] << endl;
    */
    std::map<int, FA> FFS_stack;
    std::map<std::string, FIA> pFIS_stack;

    float h = 0.25;
    float mu = 0.3;
    float l1 = 0.21;
    float l2 = 0.19;
    int tau_max = 17;
    float wkspace_margin = 0.01;

    leg_ik_out Xlims;
    Xlims.Max_X = 0.3122;
    Xlims.Min_X = -0.3122;

    vector<float> nodes = linspace(Xlims.Min_X+wkspace_margin, Xlims.Max_X-wkspace_margin, 10);

    load_data_FFS(&FFS_stack);
    load_data_FIS_forward(&pFIS_stack);
    load_data_FIS_backward(&pFIS_stack);

    
    vector<float> ecat_data = {0.1, -0.16, -0.1, 0.1};
    float marginalUtility_threshold = 0.6; 
    float x_swing = -0.15;
    float x_td_nom = 0.15;
    int NUM_LEGS = 4;
    
    float vb = 0.6;
    float vx_des = 0.6;
    float internode_dx = mean(diff(nodes));

    int trouble = 0;
    int t = 1;
    vector<bool> leg_command_in(NUM_LEGS, 0);
    int lesion = 0;

    vector<float> x_td_old(NUM_LEGS, x_td_nom);
    vector<bool> lesion_legs (NUM_LEGS, false);
    vector<float> lesion_leg_xtd(NUM_LEGS, x_td_nom);
    vector<float> lesion_leg_xswing(NUM_LEGS, x_swing);

    // Tests
    //lesion_leg_xswing[0] = lesion_leg_xswing[0]*2;
    //lesion_legs[0] = true;
    lesion = 1;

    leg_command_in[0] = 1;
    leg_command_in[3] = 1;

    Sw_St_Xtd_out _gait = Sw_St_Xtd(pFIS_stack,
                                    FFS_stack,
                                    nodes,
                                    ecat_data,
                                    NUM_LEGS,
                                    vb,
                                    vx_des,
                                    marginalUtility_threshold,
                                    wkspace_margin,
                                    x_swing,
                                    x_td_nom,
                                    x_td_old,
                                    internode_dx,
                                    trouble,
                                    t,
                                    leg_command_in,
                                    lesion,
                                    lesion_legs,
                                    lesion_leg_xtd,
                                    lesion_leg_xswing);
    
    cout << "=====leg_command=====" << endl;
    print_vector(_gait.leg_command);
    cout << "=====swing_state_flag=====" << endl;
    print_vector(_gait.swing_state_flag);
    cout << "=====x_td_out=====" << endl;
    print_vector(_gait.x_td_out);
}

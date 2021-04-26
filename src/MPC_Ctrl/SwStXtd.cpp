#include "SwStXtd.h"

// static double t_liftoff = 0;


void generate_data() {
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

    std::vector<float> nodes = linspace(Xlims.Min_X+wkspace_margin, Xlims.Max_X-wkspace_margin, 10);

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
						vector<float> lesion_leg_xswing)
{
	Sw_St_Xtd_out output;
	vector<bool> swing_transition_flag(NUM_LEGS, false);
	vector<int> leg_command = {8};
	vector<float> x_td_out = {1.5};

	// ----------------------- Check direction of travel ------------------
	string WAY = "backward";
	if(vb >= 0)
	{
		WAY = "forward";
	}

	bool doFacetEnumeration = 0;
	float swingTime = 0.25;
	
	// --------------------------------------------------------------------
	// ------------------ Definitions -----------------------------------
	//static VectorXd t_liftoff = VectorXd::Zero(NUM_LEGS);
	//VectorXd x_swingVec = x_swing*VectorXd::Ones(NUM_LEGS);
	//VectorXd x_td = x_td_old;
	//VectorXd swing_transition_flag = VectorXd::Zero(NUM_LEGS);
	// t_liftoff += 1;

	float t_for_trouble = -1.;
	vector<float> t_liftoff(NUM_LEGS, 1);
	bool can_enter_trouble_response;
	vector<float> waiting_time(4, 0);
	if(t_for_trouble < 0) {
		fill(t_liftoff.begin(), t_liftoff.end(), 1); 
		t_for_trouble = 2*swingTime;
		can_enter_trouble_response = 1;
		fill(waiting_time.begin(), waiting_time.end(), 0); 
	}
	
	vector<float> x_td = x_td_old;
	if (t - t_for_trouble >= swingTime) 
	{
		can_enter_trouble_response = 1;
	}
	vector<float> x_swingVec(NUM_LEGS, x_swing);

	if(lesion && any(lesion_legs)) {
		// These two cases seem to be the same in the matlab code
		if(WAY == "forward") {
			for(int i =0; i<lesion_legs.size();i++) {
		    	if(lesion_legs[i] == true) {
					x_swingVec[i] = lesion_leg_xswing[i];
				}
			}
		}
		else{
			for(int i =0; i<lesion_legs.size();i++) {
				if(lesion_legs[i] == true) {
					x_swingVec[i] = lesion_leg_xswing[i];
				}
			}			
		}
	}	
	int num_rows = MAX_VERTICES;
	vector<vector<vector<double>>> pFIS_single_for_MU;
	pFIS_single_for_MU.push_back({{0,0}});
	pFIS_single_for_MU.push_back({{0,0}});
	pFIS_single_for_MU.push_back({{0,0}});
	pFIS_single_for_MU.push_back({{0,0}});
	vector<float> entry2 = {0,0};
	vector<float> pFIS_interNode ={0,0};
	vector<int> check_leg_in_stance(NUM_LEGS, 0);
	vector<float> x_fh = ecat_data;
	vector<bool> leg_state = leg_command_in;
	check_leg_in_stance[2] = 10;
		
	int N_in_contact = 0;
	for(size_t leg=1; leg<=NUM_LEGS; leg++) {
		if(leg_state[leg-1] && x_fh[leg-1] > nodes[0] + wkspace_margin) {
			N_in_contact = N_in_contact+1;
			check_leg_in_stance[N_in_contact-1] = leg-1;
		}
	}
	vector<float> marginalUtility(NUM_LEGS, 0);
	

	//===============================================
	//=================== Computing the Marginal Utility
	//===============================================
	double vol_of_pFIS_tot_for_MU;
	Polygon_2 cgal_pFIS_tot_for_MU;
	if(N_in_contact) {
		getpFIS_at_x_from_pFIS_stack_out pFIS_tot_for_MU =  getpFIS_at_x_from_pFIS_stack(pFIS_stack,
																						x_fh[check_leg_in_stance[0]],
																						nodes,
																						WAY);		
		if(check_leg_in_stance[0] == 0) {
			pFIS_single_for_MU[0] = pFIS_tot_for_MU.pFIS;
		}
		else if (check_leg_in_stance[0] == 1) {
			pFIS_single_for_MU[1] = pFIS_tot_for_MU.pFIS;
		}
		else if (check_leg_in_stance[0] == 2) {
			pFIS_single_for_MU[2] = pFIS_tot_for_MU.pFIS;
		}
		else if (check_leg_in_stance[0] == 3) {
			pFIS_single_for_MU[3] = pFIS_tot_for_MU.pFIS;
		}
		else {
			pFIS_single_for_MU[0] = pFIS_tot_for_MU.pFIS;
		}
		//for(int i=0; i<pFIS_single_for_MU[0].size(); i++) {
		//	print_vector(pFIS_single_for_MU[0][i]);
		//}
		cgal_pFIS_tot_for_MU = FA_to_cgalPolygon(pFIS_tot_for_MU.pFIS);
	}

	if(N_in_contact > 1) {
		double cVol = 0;
		for(int legs = 0; legs < NUM_LEGS; legs ++ )
		{
			if (std::find(check_leg_in_stance.begin() + 1, check_leg_in_stance.end(), legs) != check_leg_in_stance.end())
			{
				getpFIS_at_x_from_pFIS_stack_out tmp =  getpFIS_at_x_from_pFIS_stack(pFIS_stack,
																					x_fh[legs],
																					nodes,
																					WAY);
				Polygon_2 cgal_tmp = FA_to_cgalPolygon(tmp.pFIS);
				//cout << "============= " << legs << "--------------------" << endl;
				//for(int i=0; i<tmp.pFIS.size(); i++) {
				//	print_vector(tmp.pFIS[i]);
				//}
				//cout << cgal_tmp << endl;
				//cout << cgal_pFIS_tot_for_MU << endl; 
				Polygon_with_holes_2  sum = CGAL::minkowski_sum_by_full_convolution_2(cgal_pFIS_tot_for_MU, cgal_tmp);
				std::vector<Point_2> mink_out;
				std::vector<Point_2> result;
				for(size_t i=0; i< sum.outer_boundary().size(); i++){
					mink_out.push_back(sum.outer_boundary()[i]);
				} 
				CGAL::convex_hull_2( mink_out.begin(), mink_out.end(), std::back_inserter(result) );
			    cgal_pFIS_tot_for_MU = cgalPoint2_to_Polygon(result);
				cVol = cgal_pFIS_tot_for_MU.area();
				if(legs == 0) {
					pFIS_single_for_MU[0] = tmp.pFIS;
				}
				else if (legs == 1) {
					pFIS_single_for_MU[1] = tmp.pFIS;
				}
				else if (legs == 2) {
					pFIS_single_for_MU[2] = tmp.pFIS;
				}
				else if (legs == 3) {
					pFIS_single_for_MU[3] = tmp.pFIS;
				}
				else {
					pFIS_single_for_MU[0] = tmp.pFIS;
				}
			
			}
		}
		vol_of_pFIS_tot_for_MU = cVol;
	}
	else if(N_in_contact == 1) {
		vol_of_pFIS_tot_for_MU = cgal_pFIS_tot_for_MU.area(); 
	}
	else {
		vol_of_pFIS_tot_for_MU = 0;
	}

	/*
	for(int i=0; i<pFIS_single_for_MU[0].size(); i++) {
		print_vector(pFIS_single_for_MU[0][i]);
	}
	cout << "==============================" << endl;
	for(int i=0; i<pFIS_single_for_MU[1].size(); i++) {
		print_vector(pFIS_single_for_MU[1][i]);
	}	
	cout << "==============================" << endl;
	for(int i=0; i<pFIS_single_for_MU[2].size(); i++) {
		print_vector(pFIS_single_for_MU[2][i]);
	}
	cout << "==============================" << endl;
	for(int i=0; i<pFIS_single_for_MU[3].size(); i++) {
		print_vector(pFIS_single_for_MU[3][i]);
	}
	*/
	// ===================================

	Polygon_2 cgal_tmp;
	double tmp_vol = 0;
	for(int legs =0; legs<NUM_LEGS; legs++){
		if(!leg_state[legs]) continue;
		vector<int> tmp_ind;
		vector<vector<double>> tmp;
		vector<vector<double>> entry2;
		for(size_t i=0; i<check_leg_in_stance.size(); i++) {
			if(i==legs) continue;
			tmp_ind.push_back(check_leg_in_stance[i]);
		}
		if(tmp_ind.size() > 0) {
			if(tmp_ind[0] == 0) {
				tmp = pFIS_single_for_MU[0];
			} 
			else if(tmp_ind[0] == 1){
				tmp = pFIS_single_for_MU[1];
			}
			else if(tmp_ind[0] == 2){
				tmp = pFIS_single_for_MU[2];
			}
			else if(tmp_ind[0] == 3){
				tmp = pFIS_single_for_MU[3];
			}
			else{
				tmp = {{0, 0}};
				cout << "Entered otherwise statement when evaluating tmp_ind(1)" << endl;
			}
			cgal_tmp = FA_to_cgalPolygon(tmp);
			if(tmp_ind.size() > 1) {
				int k = 1;
				while(k < tmp_ind.size()) {
					if(tmp_ind[k] == 0) {
						entry2 = pFIS_single_for_MU[0];
					} 
					else if(tmp_ind[k] == 1){
						entry2 = pFIS_single_for_MU[1];
					}
					else if(tmp_ind[k] == 2){
						entry2 = pFIS_single_for_MU[2];
					}
					else if(tmp_ind[k] == 3){
						entry2 = pFIS_single_for_MU[3];
					}
					else{
						entry2 = pFIS_single_for_MU[0];
						cout << "Entered otherwise statement when evaluating tmp_ind(k)" << endl;
					}
					Polygon_2 cgal_entry2 = FA_to_cgalPolygon(entry2);

 					Polygon_with_holes_2  sum = CGAL::minkowski_sum_by_full_convolution_2(cgal_tmp, cgal_entry2);
					std::vector<Point_2> mink_out;
					std::vector<Point_2> result;
					for(size_t i=0; i< sum.outer_boundary().size(); i++){
						mink_out.push_back(sum.outer_boundary()[i]);
					} 
					CGAL::convex_hull_2( mink_out.begin(), mink_out.end(), std::back_inserter(result) );
					cgal_tmp = cgalPoint2_to_Polygon(result);
					tmp_vol = cgal_tmp.area();
					k++;
				}
			}
			else {
				std::vector<Point_2> result;
				vector<Point_2> tmp_point2 = FA_to_cgalPoint2(tmp);
				CGAL::convex_hull_2( tmp_point2.begin(), tmp_point2.end(), std::back_inserter(result) );
				cgal_tmp = cgalPoint2_to_Polygon(result);
				tmp_vol = cgal_tmp.area();
			}
		} 
		else {
			tmp_vol = 0;
		}
		marginalUtility[legs] = 1 - tmp_vol/vol_of_pFIS_tot_for_MU;
	}

	//print_vector(marginalUtility);


	fill(t_liftoff.begin(), t_liftoff.end(), t);
	vector<float> x_fh_at_liftoff(NUM_LEGS, x_td_nom);

	for(size_t i =0; i<NUM_LEGS; i++) {
		vector<bool> cond_to_swing;
		bool x_marg_safety_cond;
		vector<int> neighborLegs;
		if(vb >= 0) {
			for(size_t k =0; k<x_fh.size(); k++) {
				if(x_fh[k] < x_swingVec[k]){
					cond_to_swing.push_back(true);
				}
				else{
					cond_to_swing.push_back(false);
				}
			}
			x_marg_safety_cond = x_marg_safety_cond_f(x_fh[i], wkspace_margin, nodes);
		}
		else {
			for(size_t k =0; k<x_fh.size(); k++) {
				if(x_fh[k] > x_swingVec[k]){
					cond_to_swing.push_back(true);
				}
				else{
					cond_to_swing.push_back(false);
				}
			}
			x_marg_safety_cond = x_marg_safety_cond_b(x_fh[i], wkspace_margin, nodes);
		}
	
	
		if(cond_to_swing[i] && leg_state[i]){
			if(marginalUtility[i] <= marginalUtility_threshold || x_marg_safety_cond || lesion_legs[i]){
				if(i == 1 || i == 4) {
					neighborLegs.push_back(2);
					neighborLegs.push_back(3);
				}
				else if(i==2 || i==3) {
					neighborLegs.push_back(1);
					neighborLegs.push_back(4);					
				}
				else {
					neighborLegs.push_back(1);
					neighborLegs.push_back(4);	
				}
			}
			if(all(leg_state, true)) {
				leg_state[i]=0;
				swing_transition_flag[i] = 1;
				t_liftoff[i] = t;
				x_fh_at_liftoff[i] = x_fh[i];
			}
			else {
				for(auto& neighour:neighborLegs) {
					if(leg_state[neighour] == 0) {
						if(x_marg_safety_cond || (lesion_legs[i] && lesion)) {
							if(x_marg_safety_cond){
								leg_state[neighour-1] = 1;
								leg_state[i] = 0;
								swing_transition_flag[i] = 1;
								t_liftoff[i] = t;
								x_fh_at_liftoff[i] = x_fh[i];
							}
							else {
								waiting_time[i] = waiting_time[i] + 1;
								if(waiting_time[i] >= 5) {
									waiting_time[i] =0;
									leg_state[neighour-1] = 0;
									swing_transition_flag[i] = 1;
									t_liftoff[i] = t;
									x_fh_at_liftoff[i] = x_fh[i];
								}
								else {
									leg_state[neighour-1] = 1;
									leg_state[i] = 1;
								}
							}
						}
					}
				}
			}
		}
		if(leg_state[i] == 0) {
			if(abs(t - t_liftoff[i]) < 3e-3){
				x_td[i] = x_td_nom;
				if(lesion && lesion_legs[i]) {
					if(WAY == "forward") {
						x_td[i] = lesion_leg_xtd[i];
					}
					else {
						x_td[i] = -lesion_leg_xtd[i];
					}
				}
				if(trouble) {
					cout << "trouble = true. Not implemented" << endl;
				}
			}
		}
		else {
			// Unused variable declaration in matlab code
		}
	}

	output.leg_command = leg_state;
	output.swing_state_flag = swing_transition_flag;
	output.x_td_out = x_td;	
	return output;
}




void load_data_FFS(std::map<int, FA> *FFS_stack) 
{
    FA arr1 = {{0,0},
               {13.518890266326359,45.062967554421199},
               {-22.442254107910166,74.807513693033883}};
    (*FFS_stack).insert(std::pair<int, FA> {0, arr1});

    FA arr2 = {{0,0},
               {16.447195743888440,54.823985812961467},
               {-24.529526557380347,98.400805241559269},
               {-29.926447562629864,99.754825208766206}};
    
    (*FFS_stack).insert(std::pair<int, FA> {1, arr2});

    FA arr3 = {{0,0},
               {20.994859007815137,69.982863359383785},
               {7.591682130783782,89.937972017028059}};
    (*FFS_stack).insert(std::pair<int, FA> {2, arr3});

    FA arr4 = {{0,0},
               {25.711015261042427,85.703384203474769},
               {-30.971453803429018,1.032381793447634e+02}};

    (*FFS_stack).insert(std::pair<int, FA> {3, arr4});

    FA arr5 = {{0,0},
               {26.202412727563132,87.341375758543776},
               {-36.060583015843427,1.202019433861448e+02}};

    (*FFS_stack).insert(std::pair<int, FA> {4, arr5});

    FA arr6 = {{0,0},
               {28.710077496679766,95.700258322265881},
                {-45.772856574741986,1.654626609308416e+02},
                {-46.968538962000650,1.565617965400022e+02}};

    (*FFS_stack).insert(std::pair<int, FA> {5, arr6});

    FA arr7 = {{0,0},
               {35.978002646187711,1.199266754872924e+02},
               {3.805884868233449,1.781784333607509e+02},
               {-29.018497684539728,96.728325615132434}};

    (*FFS_stack).insert(std::pair<int, FA> {6, arr7});

    FA arr8 = {{0,0},
               {54.887925220854711,1.829597507361824e+02},
               {-20.994859007815140,69.982863359383799}};

    (*FFS_stack).insert(std::pair<int, FA> {7, arr8});

    FA arr9 = {{0,0},
                {31.858422570712733,1.061947419023758e+02},
                {-16.447195743888440,54.823985812961467}};

    (*FFS_stack).insert(std::pair<int, FA> {8, arr9});

    FA arr10 = {{0,0},
                {22.442254107910173,74.807513693033911},
                {-13.518890266326361,45.062967554421199}};

    (*FFS_stack).insert(std::pair<int, FA> {9, arr10});

}

void load_data_FIS_forward(std::map<string, FIA>* pFIS_stack) 
{

    FIA forward;
    FA forward_arr16 = {{0, 0},
          {276.751055048981,         922.503516829935},
          {244.578937271026,          980.755274703394},
          {231.175760393995,          1000.71038336104},
          {190.199038092726 ,         1044.28720278964},
          {115.716104021304,          1114.04960539821},
          {79.7549596470679,          1143.79415153682},
          {17.4919639036613,          1176.65471916443},
         {-39.1905051608101,          1194.18951430571},
         {-44.5874261660597,          1195.54353427292},
         {-81.5982209143295,          1203.66927098085},
         {-117.559365288566,          1173.92472484223},
         {-165.864983603167,          1122.55396875282},
         {-241.747767831837,          1009.57708137602},
         { -274.57215038461,          928.126973630403},
         {-275.767832771869,          919.226109239563}};
    std::tuple<FA, double> forward_fia16 = make_tuple(forward_arr16, 3.546701082217108e+05);
    forward.push_back(forward_fia16);

    FA forward_arr15 = {{0, 0},
          {254.30880094107,          847.696003136901},
          {222.136683163116,           905.94776101036},
          {208.733506286085,          925.902869668004},
          {167.756783984816,          969.479689096602},
          {93.2738499133942,          1039.24209170518},
          {57.3127055391577,          1068.98663784379},
         {-4.95029020424887,          1101.84720547139},
         {-61.6327592687203,          1119.38200061268},
         {-67.0296802739698,          1120.73602057989},
          {-104.04047502224,          1128.86175728781},
         {-152.346093336841,           1077.4910011984},
         {-228.228877565511,            964.5141138216},
         {-261.053260118284,           883.064006075982},
         {-262.248942505543,          874.163141685142}};

    std::tuple<FA, double> forward_fia15 = make_tuple(forward_arr15, 3.099689963110345e+05);
    forward.push_back(forward_fia15);

    FA forward_arr14 = {{0, 0},
          {222.450378370358,          741.501261234526},
          {190.278260592403,         799.753019107984},
          {176.875083715372,        819.708127765629},
          {135.898361414103,       863.284947194226},
         {61.4154273426815,          933.047349802802},
         {  25.454282968445,          962.791895941415},
         {-36.8087127749616,          995.652463569016},
         { -93.491181839433,           1013.1872587103},
         {-98.8881028446825,          1014.54127867751},
         {-135.898897592952,          1022.66701538544},
         {-211.781681821622,          909.690128008638},
         {-244.606064374395,           828.24002026302},
         {-245.801746761654,           819.33915587218}};
    std::tuple<FA, double> forward_fia14= make_tuple(forward_arr14, 2.518405989747852e+05);
    forward.push_back(forward_fia14);

    FA forward_arr13 = {{0, 0},
          {167.562453149503 ,         558.541510498343},
          {135.390335371549 ,         616.793268371802},
          {121.987158494517 ,         636.748377029446},
          {81.0104361932486 ,         680.325196458044},
          { 6.5275021218268 ,          750.08759906662},
         {-29.4336422524097 ,         779.832145205232},
         {-91.6966379958163 ,         812.692712832833},
         {-148.379107060288 ,         830.227507974122},
         {-153.776028065537 ,         831.581527941329},
         {-190.786822813807 ,         839.707264649254},
         { -223.61120536658 ,         758.257156903636},
         {-224.806887753839 ,         749.356292512796}};

    std::tuple<FA, double> forward_fia13= make_tuple(forward_arr13, 1.627255582313354e+05);
    forward.push_back(forward_fia13);

    FA forward_arr11 = {{0, 0},
             {131.584450503315,          438.614835011051},
          {118.181273626284 ,         458.569943668695},
          {77.2045513250151,          502.146763097293},
         { 2.72161725359335,          571.909165705869},
         {-33.2395271206432,          601.653711844481},
         {-95.5025228640497,          634.514279472082},
         {-152.184991928521,          652.049074613371},
         {-157.581912933771,          653.403094580578},
         {-194.592707682041,          661.528831288503},
         {-195.788390069299,          652.627966897664}};

    std::tuple<FA, double> forward_fia11= make_tuple(forward_arr11, 9.763914993739699e+04);
    forward.push_back(forward_fia11);


    FA forward_arr9 = {{0, 0},
          {102.874373006635,          342.914576688785},
         { 89.4711961296041,          362.869685346429},
          {48.4944738283353 ,         406.446504775027},
          {12.5333294540988,           436.19105091364},
         {-49.7296662893078,          469.051618541241},
         {-106.412135353779,          486.586413682529},
         {-111.809056359029,          487.940433649736},
         {-148.819851107299 ,         496.066170357662}};

    std::tuple<FA, double> forward_fia9= make_tuple(forward_arr9, 5.719687841746338e+04);
    forward.push_back(forward_fia9);


    FA forward_arr8 = {{0, 0},
          {76.6719602790724 ,         255.573200930241},
           {63.268783402041,          275.528309587885},
          {22.2920611007722 ,         319.105129016483},
         {-13.6690832734643,          348.849675155096},
         {-70.3515523379358,          366.384470296385},
         {-75.7484733431853,          367.738490263591},
         {-112.759268091455,          375.864226971517}};

    std::tuple<FA, double> forward_fia8= make_tuple(forward_arr8, 3.277604548224890e+04);
    forward.push_back(forward_fia8);
    
    FA forward_arr7 = {{0, 0},
          {50.9609450180299,          169.869816726766},
          {37.5577681409986,          189.824925384411},
         {-3.41895416027021,          233.401744813008},
         {-39.3800985345067,          263.146290951621},
         {-44.7770195397563,          264.500310918828},
         {-81.7878142880261,          272.626047626754}};

    std::tuple<FA, double> forward_fia7= make_tuple(forward_arr7, 1.589642754174412e+04);
    forward.push_back(forward_fia7);

    FA forward_arr5 = {{0, 0},
          { 29.9660860102148,          99.8869533673827},
          {-11.010636291054,           143.46377279598},
         {-46.9717806652905,          173.208318934593},
           {-52.36870167054,            174.5623389018}};

    std::tuple<FA, double> forward_fia5= make_tuple(forward_arr5, 5.550836310672081e+03);
    forward.push_back(forward_fia5);

    FA forward_arr3 = {{0, 0},
          { 13.5188902663264,          45.0629675544212},
         {-22.4422541079102 ,         74.8075136930339}};

    std::tuple<FA, double> forward_fia3= make_tuple(forward_arr3, 1.011314568712832e+03);
    forward.push_back(forward_fia3);

    //cout << forward.size() << endl;
    (*pFIS_stack).insert(std::pair<string, FIA> {"forward", forward});
}


void load_data_FIS_backward(std::map<string, FIA>* pFIS_stack) 
{

    FIA backward;
    FA backward_arr16 = {{0, 0},
          {276.751055048981,          922.503516829935},
          {244.578937271026,          980.755274703394},
          {231.175760393995,          1000.71038336104},
          {190.199038092726,          1044.28720278964},
          {115.716104021304,          1114.04960539821},
          {79.7549596470679,          1143.79415153682},
          {17.4919639036614,          1176.65471916443},
         {-39.1905051608101,          1194.18951430571},
         {-44.5874261660596,          1195.54353427292},
         {-81.5982209143295,          1203.66927098085},
         {-117.559365288566,          1173.92472484223},
         {-165.864983603167,          1122.55396875282},
         {-241.747767831837,          1009.57708137602},
          {-274.57215038461,          928.126973630403},
         {-275.767832771869,          919.226109239563}};
    std::tuple<FA, double> backward_fia16 = make_tuple(backward_arr16, 3.546701082217109e+05);
    backward.push_back(backward_fia16);


    FA backward_arr15 = {{0, 0},
          { 263.232164782654,          877.440549275514},
            {231.0600470047,          935.692307148973},
          {217.656870127669,          955.647415806617},
            {176.6801478264,          999.224235235215},
          {102.197213754978,          1068.98663784379},
          {39.9342180115715,          1101.84720547139},
         {-16.7482510528999,          1119.38200061268},
         {-22.1451720581494,          1120.73602057989},
         {-59.1559668064193,          1128.86175728781},
         {-95.1171111806559,           1099.1172111492},
         {-143.422729495257,          1047.74645505979},
         {-219.305513723927,          934.769567682987},
           {-252.1298962767,          853.319459937369},
         {-253.325578663959,          844.418595546529}};

    std::tuple<FA, double> backward_fia15 = make_tuple(backward_arr15, 3.121770010955952e+05);
    backward.push_back(backward_fia15);


    FA backward_arr13 = {{0, 0},
          {246.784969038766,          822.616563462553},
          {214.612851260812,          880.868321336011},
           {201.20967438378,         900.823429993655},
          {126.726740312358,          970.585832602231},
          {64.4637445689519,          1003.44640022983},
           {7.7812755044804,          1020.98119537112},
         {-29.2295192437895,          1029.10693207905},
          {-65.190663618026,          999.362385940434},
         {-113.496281932627,          947.991629851019},
         {-189.379066161297,          835.014742474221},
          {-222.20344871407,          753.564634728602},
         {-223.399131101329,          744.663770337763}};

    std::tuple<FA, double> backward_fia13 = make_tuple(backward_arr13, 2.592449052401225e+05);
    backward.push_back(backward_fia13);

    FA backward_arr11 = {{0, 0},
             {225.7901,  752.6337},
            {193.6180,  810.8855},
            {119.1351,  880.6479},
            {56.8721,  913.5084},
            {0.1896,  931.0432},
            {-35.7716,  901.2987},
            {-84.0772,  849.9279},
            {-159.9600,  736.9510},
            {-192.7843,  655.5009},
            {-193.9800,  646.6001}};

    std::tuple<FA, double> backward_fia11 = make_tuple(backward_arr11, 2.076789501085621e+05);
    backward.push_back(backward_fia11);

    FA backward_arr10 = {{0, 0},
            {200.0791,  666.9303},
            {167.9070,  725.1821},
            {93.4240,  794.9445},
            {31.1610,  827.8050},
            {-4.8001,  798.0605},
            {-53.1057,  746.6897},
            {-128.9885,  633.7129},
            {-161.8129,  552.2627},
            {-163.0086,  543.3619}};

    std::tuple<FA, double> backward_fia10 = make_tuple(backward_arr10, 1.575561553267955e+05);
    backward.push_back(backward_fia10);

    FA backward_arr9 = {{0, 0},
            {173.8767,  579.5889},
            {141.7046,  637.8407},
            {67.2216, 707.6031},
            {31.2605,  677.8586},
            {-17.0451,  626.4878},
            {-92.9279,  513.5109},
            {-125.7523,  432.0608},
            {-126.9480,  423.1599}};

    std::tuple<FA, double> backward_fia9 = make_tuple(backward_arr9, 1.081401446284203e+05);
    backward.push_back(backward_fia9);


    FA backward_arr7 = {{0, 0},
            {145.1666,  483.8887},
            {112.9945,  542.1404},
            {77.0333,  512.3959},
            {28.7277,  461.0251},
            {-47.1551,  348.0483},
            {-79.9794,  266.5981}};

    std::tuple<FA, double> backward_fia7 = make_tuple(backward_arr7, 5.397842484857133e+04);
    backward.push_back(backward_fia7);

    FA backward_arr5 = {{0, 0},
            {109.1886,  363.9620},
            {73.2275 , 334.2175},
            {24.9218,  282.8467},
            {-50.9609,  169.8698}};

    std::tuple<FA, double> backward_fia5 = make_tuple(backward_arr5, 2.043557982577802e+04);
    backward.push_back(backward_fia5);

    FA backward_arr4 = {{0, 0},
              {54.3007,  181.0023},
              {18.3395,  151.2577},
             {-29.9661,   99.8870}};

    std::tuple<FA, double> backward_fia4 = make_tuple(backward_arr4, 5.629190401031054e+03);
    backward.push_back(backward_fia4);

    FA backward_arr3 = {{0, 0},
                    {22.4423,   74.8075},
                    {-13.5189,   45.0630}};


    std::tuple<FA, double> backward_fia3 = make_tuple(backward_arr3, 1.011314568712832e+03);
    backward.push_back(backward_fia3);

    (*pFIS_stack).insert(std::pair<string, FIA> {"backward", backward});
}

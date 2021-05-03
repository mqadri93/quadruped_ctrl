#ifndef UTILS_H_
#define UTILS_H_


#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <map>
#include <tuple>
#include <stdio.h>
#include <numeric>
#include <complex>
#include <cstdlib>
#include <iostream>
#include <string>
#include <CGAL/minkowski_sum_2.h>
#include "../Utilities/bops_linear.h"
#include "../Utilities/pgn_print.h"
#include <CGAL/convex_hull_2.h>
#include <CGAL/property_map.h>

using namespace Eigen;
using namespace std;

typedef std::vector<std::vector<double>> FA;
typedef std::vector<std::tuple<FA, double>> FIA;

struct Sw_St_Xtd_out {
	vector<bool> swing_state_flag;
	vector<bool> leg_command;
	vector<float> x_td_out;
};

struct getpFIS_at_x_from_pFIS_stack_out {
    double volume;
    FA pFIS;
};

struct leg_ik_out {
      float Max_X;
      float Min_X;
      float theta1 = -99;
      float theta2 = -99;
};


static double mass = 9; // [kg]
static double g = 9.81;
static int MAX_VERTICES = 100;

template <typename T>
bool any(T a){
      for(int i=0; i < a.size(); i++)
            if(a[i]) return true; 
      return false;
}

template <typename T>
void print_vector(T a){
      for(int i=0; i < a.size(); i++)
            cout << a.at(i) << ' '; 
      cout << endl;
}
template <typename T>
void print_vector2d(T a){
      for(int i=0; i < a.size(); i++)
            print_vector(a[i]);
}

template <typename T>
bool find(T a){
      vector<int> res;
      for(int i=0; i < a.size(); i++)
            if(a[i] >= 0) return true; 
      return false;
}

template <typename T>
bool any(vector<T> v, T val) {
      for(int i =0; i< v.size(); i++) {
            if(v[i] == val) {
                  return true;
            }
      }
      return false;
}

template <typename T>
bool all(vector<T> v, T val, vector<int> indices) {
      for(int i =0; i< v.size(); i++) {
            if(std::find(indices.begin(), indices.end(), i) == indices.end()) continue;
            if(v[i] != val) {
               return false;
            }
      }
      return true;
}


vector<float> diff(vector<float> a);

float mean(vector<float> a);

std::vector<float> linspace(float  a, float b, int num);

getpFIS_at_x_from_pFIS_stack_out getpFIS_at_x_from_pFIS_stack(std::map<std::string, FIA> pFIS_nodes,
                                                            float x,
                                                            vector<float> nodes,
                                                            string WAY);

Polygon_2 FA_to_cgalPolygon(FA pFIS);
Polygon_2 cgalPoint2_to_Polygon(vector<Point_2> v);
vector<Point_2> FA_to_cgalPoint2(vector<vector<double>> v);
bool x_marg_safety_cond_f(double x, double wkspace_margin, vector<float> nodes);
bool x_marg_safety_cond_b(double x, double wkspace_margin, vector<float> nodes);
#endif

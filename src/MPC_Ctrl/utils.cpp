#include "utils.h"

vector<float> diff(vector<float> a) {
      vector<float> out;
      for(size_t i=0; i< a.size()-1; i++) {
            out.push_back(a[i+1]-a[i]);
      }
      return out;
}

float mean(vector<float> a) {
      float sum_of_elems = std::accumulate(a.begin(), a.end(), 0.0);
      return sum_of_elems/a.size();
}

vector<float> linspace(float  a, float b, int num)
{
      // create a vector of length num
      vector<float> v(num);            
      // now assign the values to the vector
      for (int i = 0; i < num; i++)
      {     
            v[i] = a + i * ( (b - a) / (num-1) );
      }
      return v;
}

getpFIS_at_x_from_pFIS_stack_out getpFIS_at_x_from_pFIS_stack(std::map<std::string, FIA> pFIS_nodes,
                                                   float x,
                                                   vector<float> nodes,
                                                   string WAY)
{
    int N = nodes.size();
    int index;
    getpFIS_at_x_from_pFIS_stack_out output;
    if(WAY == "forward") {
      for(size_t i=0; i< nodes.size(); i++) {
            if(x - nodes[i] >= 0) {
                  index=i;
            }         
      }
      output.pFIS = get<0>(pFIS_nodes["forward"][N-index-1]);
      output.volume = get<1>(pFIS_nodes["forward"][N-index-1]);
    }
    else{
      for(size_t i=0; i< nodes.size(); i++) {
            if(x - nodes[i] <= 0) {
                  index=i;
            }         
      }
      output.pFIS = get<0>(pFIS_nodes["backward"][N-index-1]);
      output.volume = get<1>(pFIS_nodes["backward"][N-index-1]);          
    }
    return output;
}


Polygon_2 FA_to_cgalPolygon(FA v) {
      Polygon_2 out;
      for(size_t i = 0; i<v.size(); i++) {
            out.push_back(Point_2(v[i][0], v[i][1]));
      }
      return out;
}

Polygon_2 cgalPoint2_to_Polygon(vector<Point_2> v) {
      Polygon_2 out;
      for(size_t i = 0; i<v.size(); i++) {
            out.push_back(v[i]);
      }
      return out;      
}

vector<Point_2> FA_to_cgalPoint2(vector<vector<double>> v) {
      vector<Point_2> out;
      for(size_t i = 0; i<v.size(); i++) {
            out.push_back(Point_2(v[i][0], v[i][1]));
      }
      return out;
}

bool x_marg_safety_cond_f(double x, double wkspace_margin, vector<float> nodes) {
      if (x <= nodes[0] + wkspace_margin) {
            return true;
      }
      return false;
}

bool x_marg_safety_cond_b(double x, double wkspace_margin, vector<float> nodes) {
      if (x >= nodes[-1] + wkspace_margin) {
            return true;
      }
      return false;
}


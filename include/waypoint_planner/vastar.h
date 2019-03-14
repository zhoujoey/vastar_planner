/*********************************************************************
 * copyright: Tencent Technology (Beijing) Co. ltd
 * author: williszhou@tencent.com
*********************************************************************/
#ifndef VASTAR_H_
#define VASTAR_H_
#include <stdio.h>
#include <set>
#include <array>
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

struct cells {
	int currentCell;
	float fCost;
};

struct node {
  int x;
  int y;
  float f_score;
  float h_score;
  float g_score;
  node *parent;
};  

struct Hop {
  bool operator() (node* const c1, node* const c2) const
  { return c1->h_score < c2->h_score; }
};

struct Fop {
  bool operator() (node* const c1, node* const c2) const
  { return c1->f_score < c2->f_score; }
};
namespace vastar {
class VASTAR{
  public:
    VASTAR();
    ~VASTAR();
    bool init(int x_cell ,int y_cell, vector<unsigned char> map, double threshold, \
              double line_value, double arc_value, double smooth_weight, double smooth_times, bool allow_vague_search);
    vector<array<float,2> > Planner(int sx, int sy, int gx, int gy);
    vector<array<float,2> > smoothPath(vector<array<int,2> > path);
  private:
    vector<array<int,2> > constructPath(node* Node);
    int cal_Hscore(int x1, int y1, int x2, int y2){
    return abs(x1-x2)+abs(y1-y2);
    }
    int threshold_{128};
    int stop_deviance_{0};
    float line_value_{1};
    float arc_value_{1.4};
    double weight_path_{5.0};
    double weight_obs_{1.0};
    bool initialized_;
    bool allow_vague_search_{false};
    int x_cell_, y_cell_;
    vector<unsigned char> map_;
    float base_weight_{0.5};
    float smooth_weight_{0.1}; 
    float sm_tolerance_{0.01};
    int smooth_times_{2};
};
};
#endif 
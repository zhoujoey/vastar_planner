/*********************************************************************
 * copyright: Tencent Technology (Beijing) Co. ltd
 * author: williszhou@tencent.com
*********************************************************************/
#include <vastar_planner/vastar.h>

using namespace std;

//Default Constructor
namespace vastar {
VASTAR::VASTAR()
{
  initialized_ = false;
}
bool VASTAR::init(int x_cell ,int y_cell, vector<unsigned char> map, double threshold, \
                  double weight_path, double weight_obs, double smooth_weight, double smooth_times, bool allow_vague_search)
{
  if (map.size()< x_cell * y_cell){
      cout<<"bad map size"<<endl;
      return false;
  }
  x_cell_ = x_cell;
  y_cell_ = y_cell;
  map_ = map;
  threshold_ = threshold;
  weight_path_ = weight_path;
  weight_obs_ = weight_obs;
  smooth_weight_ = (float)smooth_weight;
  smooth_times_ = (int)smooth_times;
  allow_vague_search_ = allow_vague_search;
  cout<<"VAstar planner initialized successfully"<<endl;
  initialized_ = true;
  return true;
}

VASTAR::~VASTAR()
{
}

vector<array<float,2> > VASTAR::Planner(int sx, int sy, int gx, int gy)
{
  vector<array<int,2> > bestPath;
  //vector<array<float,2> > smoothPath;
  vector<array<float,2> > emptyPath;
  if (!initialized_){
    cout<<"VAstar planner not initialized !"<<endl;
    return emptyPath;
  }
  unsigned char visit[x_cell_][y_cell_];
  for (int i = 0; i < x_cell_; i++){
    for (int j = 0; j < y_cell_; j++){
      int index = i * y_cell_ + j;
      visit[i][j] = map_[index];
    }
  }

  if ((sx == gx )&&(sy == gy)){
    array<float,2> point;
    point[0] = sx;
    point[1] = sy;
    emptyPath.push_back(point);
    emptyPath.push_back(point);
    return emptyPath;    
  }
  
  multiset<node*, Fop> openSet;
  multiset<node*, Hop> closeSet;
  visit[sx][sy] = 0;
  node *Start_Node = new node();
  Start_Node->x = sx;
  Start_Node->y = sy;
  Start_Node->parent = NULL;
  Start_Node->h_score = cal_Hscore(sx, sy, gx, gy);
  Start_Node->g_score = 0;
  Start_Node->f_score = Start_Node->g_score + Start_Node->h_score;
  openSet.insert(Start_Node);
  while (!openSet.empty ()) {
    node *currentNode=*openSet.begin();
    openSet.erase(openSet.begin());
    closeSet.insert(currentNode);
    //add free neibor to openSet
    for (int i=-1;i<=1;i++){
      for (int j=-1; j<=1;j++){
        int cx=currentNode->x+i;
        int cy=currentNode->y+j;
        if ((cx >= 0) && (cx < x_cell_) &&  (cy >= 0) &&(cy < y_cell_) && (!(i == 0 && j == 0)) ){
          //int c_cost=static_cast<int>(costmap_->getCost(cx, cy));
          if (visit[cx][cy]<threshold_) {
            //update g_score
            double path_cost;
            double obstacle_cost = visit[cx][cy];
            visit[cx][cy] = 255;
            if(i*j==0){
              path_cost = line_value_;
            }else{
              path_cost = arc_value_;
            }
            //double corss_cost = weight_path_ * path_cost + weight_obs_ * obstacle_cost;
            //******add NeighborCell To Openset
            node *Node=new node();
            Node->x=cx;
            Node->y=cy;
            Node->parent=currentNode;
            //key point! to make path away from obstcle and stop in a find place
            /*
            Node->g_score=currentNode->g_score + visit[cx][cy];
            Node->h_score=cal_Hscore(cx,cy,gx,gy)+c_cost;
            Node->f_score=Node->g_score+Node->h_score;
            openSet.insert(Node);*/
            Node->g_score=currentNode->g_score + weight_path_ * path_cost;
            Node->h_score=cal_Hscore(cx,cy,gx,gy)+ weight_obs_ * obstacle_cost;
            Node->f_score=Node->g_score + Node->h_score;
            openSet.insert(Node);
            //***************goal****************//
            if(Node->x == gx && Node->y == gy){
              bestPath=constructPath(Node);
              //free memory
              for (multiset<node*>::iterator itt = openSet.begin(); itt!=openSet.end(); ++itt){
                delete *itt;
              }
              for (multiset<node*>::iterator it=closeSet.begin(); it!=closeSet.end(); ++it){
                delete *it;
              }
              openSet.clear();
              closeSet.clear();
              return smoothPath(bestPath);
            }
          }
        }
      }
    }
	}
  //search all free space not find goal, set the clostest point in closeset as finnal points.
  if (!allow_vague_search_){
    return emptyPath;
  }else{
    bestPath = constructPath(*closeSet.begin());
    for (multiset<node*>::iterator it=closeSet.begin();it!=closeSet.end();++it){
      delete *it;
    }
    for (multiset<node*>::iterator itt=openSet.begin();itt!=openSet.end();++itt){
      delete *itt;
    }
    closeSet.clear();
    return smoothPath(bestPath);
  }
}

vector<array<float,2> > VASTAR::smoothPath(vector<array<int,2> > raw_path)
{
  vector<array<float,2> > smooth_path;
  vector<array<int,2> > path;
  for (int i =0; i< raw_path.size(); i++){
    if(i % smooth_times_ == 0){
      array<int,2> dpose {raw_path[i][0], raw_path[i][1]};
      path.push_back(dpose);
    }
  }
  for (int i =0; i< path.size(); i++){
    array<float,2> pose {float(path[i][0]), float(path[i][1])};
    smooth_path.push_back(pose);
  }
  float change = sm_tolerance_;
  while (change >= sm_tolerance_){
    change = 0.0;
    for(int i = 1; i < path.size() - 1; i++){
      for(int j = 0; j < 2; j++){
        float x_i = path[i][j];
        float y_i = smooth_path[i][j];
        float y_pre = smooth_path[i-1][j];
        float y_next = smooth_path[i+1][j];
        float y_i_saved = y_i;
        y_i += base_weight_ * (x_i - y_i) + smooth_weight_ * (y_next + y_pre - (2 * y_i));
        smooth_path[i][j] = y_i;
        change += fabs(y_i - y_i_saved);
      }
    }
  }
  return smooth_path;
}

vector<array<int,2> > VASTAR::constructPath(node* Node)
{
  vector<array<int, 2> > bestPath;
  while(Node->parent!=NULL){
    array<int, 2> pose;
    pose[0] = Node->x;
    pose[1] = Node->y;
    bestPath.insert(bestPath.begin(),pose);
    Node=Node->parent;
  }
  return bestPath;
}
}
;

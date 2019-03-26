#include <vastar_planner/vastar_planner.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(vastar_planner::VastarPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

namespace vastar_planner
{
//Default Constructor
VastarPlanner::VastarPlanner()
{

}
VastarPlanner::VastarPlanner(ros::NodeHandle &nh)
{
  ROSNodeHandle = nh;
}

VastarPlanner::VastarPlanner(string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(name, costmap_ros);
}

VastarPlanner::~VastarPlanner()
{
  delete planner_;
  if (dsrv_)
    delete dsrv_;
}
void VastarPlanner::initialize(string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_){
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    planner_ = new vastar::VASTAR();
    ros::NodeHandle private_nh("~/" + name);
    ros::NodeHandle nh;
    //wp_sub = nh.subscribe("waypoints", 1,  &VastarPlanner::waypoints_cb, this);
    plan_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);
    //ROS_INFO("Waypoint planner initialized successfully");
    dsrv_ = new dynamic_reconfigure::Server<vastar_planner::VastarPlannerConfig>(ros::NodeHandle("~/" + name));
    dynamic_reconfigure::Server<vastar_planner::VastarPlannerConfig>::CallbackType cb = boost::bind(
            &VastarPlanner::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
    initialized_ = true;
  }
  else
    ROS_WARN("waypoint planner has already been initialized... doing nothing");
}

void VastarPlanner::reconfigureCB(vastar_planner::VastarPlannerConfig& config, uint32_t level) {
  use_waypoint_ = config.use_waypoint;
  stop_deviance_ = config.stop_deviance;
  weight_path_ = config.planner_weight_path;
  weight_obs_ = config.planner_weight_obs;
  smooth_weight_ = config.smooth_weight;
  smooth_times_ = config.smooth_times;
  allow_vague_search_ = config.allow_vague_search;
}

bool VastarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                             vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_){
    ROS_ERROR("waypoint planner has not been initialized, please call initialize() to use the planner");
    return false;
  }
  ////////////////////////get costmap param
  ROS_WARN("start make plan");
  originX_ = costmap_->getOriginX();
  originY_ = costmap_->getOriginY();
  x_cell_ = costmap_->getSizeInCellsX();
  y_cell_ = costmap_->getSizeInCellsY();
  resolution_ = costmap_->getResolution();
  ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,
            goal.pose.position.x, goal.pose.position.y);
  plan.clear();
  if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
    ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
              costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
    return false;
  }
  tf::Stamped < tf::Pose > goal_tf;
  tf::Stamped < tf::Pose > start_tf;
  poseStampedMsgToTF(goal, goal_tf);
  poseStampedMsgToTF(start, start_tf);

  //* convert the start and goal positions into cells
  float startX = start.pose.position.x;
  float startY = start.pose.position.y;
  float goalX = goal.pose.position.x;
  float goalY = goal.pose.position.y;
  int start_x = (startX - originX_) / resolution_;
  int start_y = (startY - originY_) / resolution_;
  int goal_x = (goalX - originX_) / resolution_;
  int goal_y = (goalY - originY_) / resolution_;
  if (!((0< start_x < x_cell_ ) && (0< start_y < y_cell_ ) && (0< goal_x < x_cell_ ) && (0< goal_y < y_cell_ ))) {
    ROS_WARN("start or goal pose is out of index");
    return false;
  }
  ///////////////////////////get obstacle map data
  vector<array<float, 2> > bestPath;
  vector<unsigned char> map;
  for (int i = 0; i < x_cell_; i++){
    for (int j = 0; j < y_cell_; j++){
      int t_cost = static_cast<int>(costmap_->getCost(i, j));
      unsigned char map_value = 0;
      if(0 <= t_cost <= 255){
        map_value = t_cost;
      }else{
        map_value = 254;
      }
      map.push_back(map_value);
    }
  }
  //make path
  if (use_waypoint_){
    array<int, 2> bestwaypoint;
    bestwaypoint = SearchWaypoint(goal_x, goal_y, waypoints);
    goal_x = bestwaypoint[0];
    goal_y = bestwaypoint[1];
  }
  ///////////////////VASTAR planner;
  planner_->init(x_cell_, y_cell_, map, threshold_, weight_path_, weight_obs_, smooth_weight_, smooth_times_, allow_vague_search_);
  bestPath = planner_->Planner(start_x, start_y ,goal_x ,goal_y);
  if ( bestPath.size()>1){
    plan.clear();
    for (int i = 0; i < bestPath.size()-1; i++){
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = costmap_ros_->getGlobalFrameID();
      pose.pose.position.x = bestPath[i][0] * resolution_ + originX_;
      pose.pose.position.y = bestPath[i][1] * resolution_ + originY_;
      double yaw = atan2(bestPath[i+1][1] - bestPath[i][1], bestPath[i+1][0] - bestPath[i][0]);
      pose.pose.orientation = toQuaternion(0, 0, yaw);
      plan.push_back(pose);
    }
    publishPlan(plan);
    return true;
  }else if ( bestPath.size() == 1){
    plan.clear();
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = costmap_ros_->getGlobalFrameID();
    pose.pose.position.x = bestPath[0][0] * resolution_ + originX_;
    pose.pose.position.y = bestPath[0][1] * resolution_ + originY_;
    double yaw = 0;
    pose.pose.orientation = toQuaternion(0, 0, yaw);
    plan.push_back(pose);
    plan.push_back(pose);
    publishPlan(plan);
    return true;
  }else{
    ROS_WARN("Can not search local path");
    return false;
  }
}

array<int,2> VastarPlanner::SearchWaypoint(int gx, int gy, vector<array<int,2> > wp)
{
  array<int,2> bestpoint;
  array<int,2> startpoint;
  vector<array<int, 2> > openset;
  bool hits[x_cell_][y_cell_];
  if (wp.size() == 0){
    array<int,2> goalpoint={gx, gy};
    ROS_WARN("no waypoint, make plan of goal");
    return goalpoint;
  }
  int cost=static_cast<int>(costmap_->getCost(gx, gy));
  if (cost > threshold_/2){
    findFreeNeibour(gx, gy);
  }
  //search way_point
  startpoint = {gx, gy};

  for (int i=0; i<x_cell_; i++){
    for (int j=0; j<y_cell_; j++){
      hits[i][j] = 0;
    }
  }
  openset.push_back(startpoint);
  hits[openset[0][0]][openset[0][1]] = 1;
  while (openset.size()!=0 && ros::ok()){
    array<int ,2> cp = *openset.begin();
    openset.erase(openset.begin());
    for (int i=-1; i<=1; i++){
      for (int j=-1; j<=1; j++){
        int cx = cp[0]+i;
        int cy = cp[1]+j;
        if ((cx >= 0) && (cx < x_cell_) &&  (cy >= 0) &&(cy < y_cell_) && (!(i == 0 && j == 0))){
          if (!hits[cx][cy]){
            for (int k =0; k< wp.size(); k++){
              if (wp[k][0] == cx && wp[k][1] == cy){
                bestpoint = {cx, cy};
                return bestpoint;
              }
            }
            array<int ,2> c_point;
            c_point = {cx, cy};
            int c_cost=static_cast<int>(costmap_->getCost(cx, cy));
            if (c_cost<threshold_) {
              openset.push_back(c_point);
            }
          }
          hits[cx][cy] = 1;
        }
      }
    }   
  } 
  //if can not search way_point , find nearest waypoint as bestwaypoint
  ROS_WARN("can not search waypoint, find nearest waypoint");
  float min_dist = numeric_limits<float>::max();
  for (int l =0; l< wp.size(); l++){
    float c_dist = sqrt(pow(wp[l][0] - gx, 2) + pow(wp[l][1] - gy, 2));
    if (c_dist < min_dist) {
      min_dist = c_dist;
      bestpoint = {wp[l][0], wp[l][1]};
    }
  }
  return bestpoint;
}

bool VastarPlanner::findFreeNeibour(int& cx, int& cy)
{
  int k = 1;
  while (k < 50){
    for (int i = -k; i <= k; i++){
      for (int j = -k; j <= k;j++){
        if(abs(i)-k == 0 || abs(j)-k == 0 && cx+i>=0 \
             && cx+i < x_cell_ && cy+j >= 0 && cy+j < y_cell_){
          int cost = static_cast<int>(costmap_->getCost(cx+i, cy+j));
          if (cost < threshold_/2){
            cx = cx+i;
            cy = cy+j;
            return true;
          }
        }
      }
    }
    k++;
  }
  return false;
}

void VastarPlanner::publishPlan(const vector<geometry_msgs::PoseStamped>& path) 
{
    if (!initialized_) {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }
    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());
    gui_path.header.frame_id = path[0].header.frame_id;
    gui_path.header.stamp = ros::Time::now();
    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }
    plan_pub_.publish(gui_path);
}

geometry_msgs::Quaternion VastarPlanner::toQuaternion(double pitch, double roll, double yaw)
{
  // Abbreviations for the various angular functions
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);

  geometry_msgs::Quaternion q;
  q.w = cy * cr * cp + sy * sr * sp;
  q.x = cy * sr * cp - sy * cr * sp;
  q.y = cy * cr * sp + sy * sr * cp;
  q.z = sy * cr * cp - cy * sr * sp;
  return q;
}

void VastarPlanner::toEulerAngle(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw)
{
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w * q.y - q.z * q.x);
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
  yaw = atan2(siny_cosp, cosy_cosp);
}

}
;

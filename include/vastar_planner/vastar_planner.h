#include <stdio.h>
#include <array>
/** include ros libraries**********************/
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
/** ********************************************/ 
#include <vastar_planner/VastarPlannerConfig.h>
#include <angles/angles.h>
#include <vastar_planner/vastar.h>

using namespace std;
using std::string;

#ifndef vastar_planner_H_
#define vastar_planner_H_


namespace vastar_planner {
  
class VastarPlanner : public nav_core::BaseGlobalPlanner {
  public:
    VastarPlanner (ros::NodeHandle &); //this constructor is may be not needed
    VastarPlanner ();
    ~VastarPlanner ();
    VastarPlanner(string name, costmap_2d::Costmap2DROS* costmap_ros);
    /** overriden classes from interface nav_core::BaseGlobalPlanner **/
    void initialize(string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, 
      vector<geometry_msgs::PoseStamped>& plan
    );
    ros::NodeHandle ROSNodeHandle;
  private:  
    void toEulerAngle(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw);
    geometry_msgs::Quaternion toQuaternion(double pitch, double roll, double yaw);
    void publishPlan(const vector<geometry_msgs::PoseStamped>& path);

    array<int,2> SearchWaypoint(int gx, int gy, vector<array<int,2> > wp);
    bool findFreeNeibour(int& cx, int& cy);

    dynamic_reconfigure::Server<vastar_planner::VastarPlannerConfig> *dsrv_;
    void reconfigureCB(vastar_planner::VastarPlannerConfig &config, uint32_t level);

    ros::Publisher plan_pub_;
    ros::Subscriber wp_sub;
    vastar::VASTAR* planner_;

    float originX_;
    float originY_;
    float resolution_;
    int threshold_{127};
    int stop_deviance_{0};
    float line_value_{1};
    float arc_value_{1.4};
    double weight_obs_{1.0};
    double weight_path_{1.0};
    double smooth_weight_{0.4}; 
    double sm_tolerance_{0.01};
    int smooth_times_{2};
    bool allow_vague_search_{false};
    bool use_waypoint_{true};
    vector<array<int, 2> > waypoints; 
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    bool initialized_=false;
    int x_cell_;
    int y_cell_;
};

};
#endif

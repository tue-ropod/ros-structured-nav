#ifndef MANEUVER_NAV_HH
#define MANEUVER_NAV_HH

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

// All this is needed to interface with teh costmap and thus check for obstacles
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <nav_msgs/Path.h>

// Global planner includes
#include <maneuver_planner/maneuver_planner.h>
#include <napoleon_driving/Goal.h>
#include <napoleon_driving/Feedback.h>
#include <napoleon_driving/Configuration.h>

#include <nav_core/base_local_planner.h>
#include <pluginlib/class_loader.h>


namespace mn {
    
class NapoleonDriving
{    
    enum { LOC_NAV_IDLE = 0,
           LOC_NAV_SET_PLAN,
           LOC_NAV_BUSY,
           LOC_NAV_GOTOPOINT,
           LOC_NAV_WAYPOINT_DONE,
           LOC_NAV_DONE
         };
         
    enum { MANV_NAV_IDLE = 0,
           MANV_NAV_MAKE_INIT_PLAN,
           MANV_NAV_BUSY,
           MANV_NAV_DONE
         };         

public:

    

    NapoleonDriving(tf::TransformListener& tf, ros::NodeHandle& nh);

    ~NapoleonDriving();

    void init();
    void reinitPlanner(const geometry_msgs::Polygon& new_footprint);
    bool isGoalReachable();
    void cancel() ;
    void publishZeroVelocity();
    void publishOneVelocity();
    void publishCustomVelocity(double v_ax, double theta_dot);
    
    double footprintCost(double x_i, double y_i, double theta_i);
    bool   checkFootprintOnGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& plan, const double& max_ahead_dist, double& dist_before_obs, int &index_closest_to_pose, int &index_before_obs);
    bool gotoGoal(const geometry_msgs::PoseStamped& goal);
    bool gotoGoal(const napoleon_driving::Goal& goal);
    void callLocalNavigationStateMachine();
    void callNapoleonDrivingStateMachine();
    
    
   maneuver_planner::ManeuverPlanner  maneuver_planner;   
//    base_local_planner::TrajectoryPlannerROS local_planner;
//    teb_local_planner::TebLocalPlannerROS local_planner;
   std::vector<geometry_msgs::PoseStamped> plan;        
   
   costmap_2d::Costmap2DROS* costmap_ros_, * local_costmap_ros;
   costmap_2d::Costmap2D* costmap_;
   base_local_planner::WorldModel* world_model_; ///< @brief The world model that the controller will use  
      
private:      
   double MAX_AHEAD_DIST_BEFORE_REPLANNING;     // TODO: make static const?
   double REPLANNING_HYSTERESIS_DISTANCE;       // TODO: make static const?
   bool initialized_;
   bool goal_free_;
   bool simple_goal_;
   bool append_new_maneuver_;
   tf::TransformListener& tf_;   
   geometry_msgs::PoseStamped goal_;
   napoleon_driving::Goal mn_goal_;
   int local_nav_state_, manv_nav_state_;
   int local_nav_next_state_, manv_nav_next_state_;      
   int local_plan_infeasible_, local_plan_infeasible_cnt_;
   double xy_goal_tolerance_, yaw_goal_tolerance_;
   ros::Publisher vel_pub_;
   ros::Publisher pub_navigation_fb_;
   ros::NodeHandle& nh_;
   pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
   boost::shared_ptr<nav_core::BaseLocalPlanner> local_planner_;
   
   bool getRobotPose(tf::Stamped<tf::Pose> & global_pose);
      
      
      
//       bool checkFootprintOnGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& plan, const double max_ahead_dist, double dist_before_obs);


};

}


#endif

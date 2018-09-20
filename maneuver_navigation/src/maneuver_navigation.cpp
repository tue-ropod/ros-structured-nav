#include "maneuver_navigation.h"

namespace mn
{
ManeuverNavigation::ManeuverNavigation(tf::TransformListener& tf, ros::NodeHandle& nh) :
tf_(tf), nh_(nh), blp_loader_("nav_core", "nav_core::BaseLocalPlanner")
{    
    
    initialized_ = false;
};


ManeuverNavigation::~ManeuverNavigation() 
{
    local_planner_.reset();
};

void ManeuverNavigation::init() 
{    


    
    local_costmap_ros = new costmap_2d::Costmap2DROS("local_costmap", tf_);    
    
//     costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_); //global_costmap
    costmap_ros_ = local_costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    
    world_model_ = new base_local_planner::CostmapModel(*costmap_);
    maneuver_planner = maneuver_planner::ManeuverPlanner("maneuver_planner",costmap_ros_);
//     try{
//         local_planner.initialize("TrajectoryPlannerROS", &tf_, local_costmap_ros);
//     } catch(...) {
//       // 
//         ROS_FATAL("Failed to initialize the global planner");
//         exit(1);
//     }
    
    //create a local planner
    std::string local_planner_str;
    nh_.param("base_local_planner", local_planner_str, std::string("base_local_planner/TrajectoryPlannerROS"));
    try {      
      local_planner_ = blp_loader_.createInstance(local_planner_str);
      //ROS_INFO("Created local_planner %s", local_planner_str.c_str());
      local_planner_->initialize(blp_loader_.getName(local_planner_str), &tf_, local_costmap_ros);
    } catch (const pluginlib::PluginlibException& ex) {
      //ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
        ROS_FATAL("Failed to create the local planner");
      exit(1);
    }    
    
    nh_.getParam(blp_loader_.getName(local_planner_str)+"/xy_goal_tolerance", xy_goal_tolerance_);
    nh_.getParam(blp_loader_.getName(local_planner_str)+"/yaw_goal_tolerance", yaw_goal_tolerance_);
        
    
    mn_goal_.conf.precise_goal = true;
    mn_goal_.conf.use_line_planner = true;
    
    local_nav_state_ = LOC_NAV_IDLE;
    manv_nav_state_  = MANV_NAV_IDLE;
    
    MAX_AHEAD_DIST_BEFORE_REPLANNING = 2.0;
    REPLANNING_HYSTERESIS_DISTANCE = 1.0;
    goal_free_ =  false;
    
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    
    pub_navigation_fb_ =   nh_.advertise<geometry_msgs::PoseStamped> ( "/maneuver_navigation/feedback", 1 );
    
    initialized_ = true;

};

void ManeuverNavigation::reinitPlanner(const geometry_msgs::Polygon& new_footprint) 
{      
    costmap_ros_->setUnpaddedRobotFootprintPolygon(new_footprint);   
    costmap_ros_->resetLayers();
    // initialize maneuver planner
    maneuver_planner = maneuver_planner::ManeuverPlanner("maneuver_planner",costmap_ros_);
    // Initializelocal planner
    std::string local_planner_str;
    nh_.param("base_local_planner", local_planner_str, std::string("base_local_planner/TrajectoryPlannerROS"));
    local_planner_.reset();    
    try {      
      local_planner_ = blp_loader_.createInstance(local_planner_str);
      local_planner_->initialize(blp_loader_.getName(local_planner_str), &tf_, local_costmap_ros);
    } catch (const pluginlib::PluginlibException& ex) {
      //ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
        ROS_FATAL("Failed to create the local planner");
      exit(1);
    } 
    
    nh_.getParam(blp_loader_.getName(local_planner_str)+"/xy_goal_tolerance", xy_goal_tolerance_);
    nh_.getParam(blp_loader_.getName(local_planner_str)+"/yaw_goal_tolerance", yaw_goal_tolerance_);
    
    mn_goal_.conf.precise_goal = true;
    mn_goal_.conf.use_line_planner = true;    

}

  void ManeuverNavigation::publishZeroVelocity(){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
  }




bool ManeuverNavigation:: gotoGoal(const geometry_msgs::PoseStamped& goal) 
{    
    goal_ = goal;
    simple_goal_ = true;
    mn_goal_.conf.precise_goal = true;
    mn_goal_.conf.use_line_planner = true;
    manv_nav_state_ = MANV_NAV_MAKE_INIT_PLAN;    
    local_nav_state_ = LOC_NAV_IDLE;
    return true; // TODO: implement

};

bool ManeuverNavigation:: gotoGoal(const maneuver_navigation::Goal& goal) 
{        
    mn_goal_ = goal;
    simple_goal_ = false;
    manv_nav_state_ = MANV_NAV_MAKE_INIT_PLAN;    
    local_nav_state_ = LOC_NAV_IDLE;
    return true; // TODO: implement

};

void ManeuverNavigation:: cancel() 
{    
   publishZeroVelocity();        
   local_nav_state_ = LOC_NAV_IDLE;
   manv_nav_state_   = MANV_NAV_IDLE;
   return;

};

bool ManeuverNavigation::isGoalReachable() 
{
    return true; // TODO: implement

};


//we need to take the footprint of the robot into account when we calculate cost to obstacles
double ManeuverNavigation::footprintCost(double x_i, double y_i, double theta_i)
{
    if(!initialized_)
    {
        ROS_ERROR("The navigator has not been initialized");
        return -1.0;
    }

    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();

    //if we have no footprint... do nothing
    if(footprint.size() < 3)
        return -1.0;

    //check if the footprint is legal
    double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
    return footprint_cost;
}


bool ManeuverNavigation::checkFootprintOnGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& plan, const double& max_ahead_dist, double& dist_before_obs)
{
    tf::Stamped<tf::Pose> global_pose;
    dist_before_obs = 0.0;
    if( !getRobotPose(global_pose) )
        return false;    
    // First find the closes point from the robot pose to the path   
    double dist_to_path_min = 1e3;
    double dist_to_path; 
    tf::Pose pose_temp;
    tf::Quaternion quat_temp;
    int index_pose;
    double yaw, pitch, roll;
    double x = global_pose.getOrigin().getX();
    double y = global_pose.getOrigin().getY();
    int i;

    for (i =0; i < plan.size(); i++) 
    {
        dist_to_path = hypot(plan[i].pose.position.x-x,plan[i].pose.position.y-y);
        if(dist_to_path < dist_to_path_min)
        {
            dist_to_path_min = dist_to_path;
            index_pose = i; // TODO: Do this in a smarter way, remebering last index. Index needs to be resseted when there is a replan
        }
        else
            break;
    }    
    // Now start checking poses in the future up to the desired distance
    double total_ahead_distance = 0.0;
    dist_before_obs = max_ahead_dist;
    double dist_next_point;
    double footprint_cost;
    bool is_traj_free = true;
    tf::Stamped<tf::Pose> pose_from_plan;
    
    for (i = index_pose; i < plan.size()-1; i++) 
    {
        dist_next_point = hypot(plan[i].pose.position.x-plan[i+1].pose.position.x,plan[i].pose.position.y-plan[i+1].pose.position.y);
        total_ahead_distance += dist_next_point;
        if( total_ahead_distance < max_ahead_dist)
        {
            tf::poseStampedMsgToTF(plan[i+1],pose_from_plan);             
            pose_from_plan.getBasis().getEulerYPR(yaw, pitch, roll);
            footprint_cost = footprintCost(pose_from_plan.getOrigin().getX(), pose_from_plan.getOrigin().getY(), yaw);
            if( footprint_cost < 0 )
            {
                ROS_INFO("footprint_cost %f",footprint_cost);
                is_traj_free = false;
                dist_before_obs = total_ahead_distance;
                break;
            }
        }
        else
            break;
    }    
    return is_traj_free;
    
    
}


void ManeuverNavigation::callLocalNavigationStateMachine() 
{
    geometry_msgs::Twist cmd_vel;
    tf::Stamped<tf::Pose> global_pose;
    geometry_msgs::PoseStamped feedback_pose;    
    
    if( getRobotPose(global_pose) )
    {
        tf::poseStampedTFToMsg(global_pose, feedback_pose); 
        pub_navigation_fb_.publish(feedback_pose);
    }    
    
    switch(local_nav_state_){
        case LOC_NAV_IDLE:
            break;                    
        case LOC_NAV_SET_PLAN:
            
            if (!local_planner_->setPlan(plan))
            {
                ROS_ERROR("Plan not set");
                local_nav_state_  = LOC_NAV_IDLE;
                manv_nav_state_   = MANV_NAV_IDLE;
            }
            else
                local_nav_state_ = LOC_NAV_BUSY;
            
            break;  
        case LOC_NAV_BUSY:  
            
            if(local_planner_->isGoalReached())
            {
                ROS_INFO("local planner, Goal reached!");
                local_nav_state_ = LOC_NAV_IDLE;
                tf::Stamped<tf::Pose> goal_pose;
                tf::poseStampedMsgToTF(goal_,goal_pose);
                tf::Pose diff_pose;
                diff_pose = goal_pose.inverseTimes(global_pose);
                double dist_to_goal = hypot(diff_pose.getOrigin().getX(), diff_pose.getOrigin().getY());
                double diff_yaw =  tf::getYaw(diff_pose.getRotation()); 
                
                if( mn_goal_.conf.precise_goal && ( std::abs(dist_to_goal) > xy_goal_tolerance_ || std::abs(diff_yaw) > yaw_goal_tolerance_ ) ) 
                     manv_nav_state_  = MANV_NAV_MAKE_INIT_PLAN; // replan maneuver until tolerances are met
                else
                    manv_nav_state_   = MANV_NAV_IDLE;
                
                
            }
            else if(local_planner_->computeVelocityCommands(cmd_vel))
            {
                //make sure that we send the velocity command to the base
                vel_pub_.publish(cmd_vel);
                local_plan_infeasible_ = false;
            }
            else 
            {
                ROS_ERROR("local planner, The local planner could not find a valid plan.");
                local_plan_infeasible_ = true;
//                 publishZeroVelocity();        
//                 local_nav_state_ = LOC_NAV_IDLE;
//                 manv_nav_state_   = MANV_NAV_MAKE_INIT_PLAN;
            }
            
                  
            
            break;
        default:
            break;
    }

};


bool ManeuverNavigation::getRobotPose(tf::Stamped<tf::Pose> & global_pose) 
{
    if(!costmap_ros_->getRobotPose(global_pose))
    {
        ROS_ERROR("maneuver_navigation cannot make a plan for you because it could not get the start pose of the robot");
        publishZeroVelocity();        
        local_nav_state_ = LOC_NAV_IDLE;
        manv_nav_state_   = MANV_NAV_IDLE;
        return false;
    }   
    return true;
};

void ManeuverNavigation::callManeuverNavigationStateMachine() 
{
    double dist_before_obs;  
    
    tf::Stamped<tf::Pose> global_pose;
    geometry_msgs::PoseStamped start;    
    
    switch(manv_nav_state_){
        case MANV_NAV_IDLE:
            break;          
        case MANV_NAV_MAKE_INIT_PLAN:                            
            if( simple_goal_ )
            {
                if( !getRobotPose(global_pose) )
                    break;            
                tf::poseStampedTFToMsg(global_pose, start);            
            }
            else
            {
                goal_ = mn_goal_.goal;
                start = mn_goal_.start;
            }
            goal_free_ = maneuver_planner.makePlan(start,goal_, plan, dist_before_obs);            
            ROS_INFO("dist_before_obs: %f", dist_before_obs);
            if( dist_before_obs > MAX_AHEAD_DIST_BEFORE_REPLANNING || goal_free_ == true)
            {
                if( plan.size()>0 )
                {
                    simple_goal_ = true; // the structured goal is only the first time is received and succesful                
                    local_nav_state_ = LOC_NAV_SET_PLAN;
                    manv_nav_state_  = MANV_NAV_BUSY;
                }
                else
                {
                    ROS_ERROR("Empty plan");
                }                    
            }
            else
            {
                ROS_WARN("maneuver_navigation cannot make a plan due to obstacles, inform and keep trying");
                publishZeroVelocity();  
              //  local_nav_state_ = LOC_NAV_IDLE;
                // manv_nav_state_   = MANV_NAV_IDLE; 
            }
            
            break;
         case MANV_NAV_BUSY:
             if( !checkFootprintOnGlobalPlan(plan, MAX_AHEAD_DIST_BEFORE_REPLANNING, dist_before_obs) )
             {
                ROS_INFO("Obstacle in front at %.2f m. Try to replan",dist_before_obs);
                publishZeroVelocity();  // TODO: Do this smarter by decreasing speed while computing new path    
                if( !getRobotPose(global_pose) )
                    break;
                
                tf::poseStampedTFToMsg(global_pose, start);              
                if( maneuver_planner.makePlan(start,goal_, plan) )
                {
                    if( plan.size()>0 )
                    {
                        local_nav_state_ = LOC_NAV_SET_PLAN;
                        manv_nav_state_  = MANV_NAV_BUSY;                        
                    }
                    else
                    {
                        ROS_ERROR("Empty plan");
                    }
                }
                else
                {
                    ROS_INFO("No replan possible. Stop, inform and continue trrying");
                    publishZeroVelocity();        
                   // local_nav_state_ = LOC_NAV_IDLE;
                   // manv_nav_state_   = MANV_NAV_MAKE_INIT_PLAN;
                }                                 
             }
             
             if (goal_free_ == false && dist_before_obs < (MAX_AHEAD_DIST_BEFORE_REPLANNING+REPLANNING_HYSTERESIS_DISTANCE)) // When the goal was not free and we are close to end of temporary plan, replan
             {
                if( !getRobotPose(global_pose) )
                    break;
                
                tf::poseStampedTFToMsg(global_pose, start);            
                goal_free_ = maneuver_planner.makePlan(start,goal_, plan, dist_before_obs);            
                if( dist_before_obs > MAX_AHEAD_DIST_BEFORE_REPLANNING )
                {
                    if( plan.size()>0 )
                    {
                        local_nav_state_ = LOC_NAV_SET_PLAN;
                        manv_nav_state_  = MANV_NAV_BUSY;
                    }
                    else
                    {
                        ROS_ERROR("Empty plan");
                    }                    
                }              
             }
             
            break;
        default:
            break;
    }    

};


 
}

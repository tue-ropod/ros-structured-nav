#include "maneuver_navigation.h"

namespace mn
{
ManeuverNavigation::ManeuverNavigation(tf::TransformListener& tf, ros::NodeHandle& nh) :
tf_(tf), nh_(nh)
{    
    
    initialized_ = false;
};


ManeuverNavigation::~ManeuverNavigation() {};

void ManeuverNavigation::init() 
{    

    costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    costmap_ = costmap_ros_->getCostmap();
    
    local_costmap_ros = new costmap_2d::Costmap2DROS("local_costmap", tf_);    
    
    world_model_ = new base_local_planner::CostmapModel(*costmap_);
    maneuver_planner = maneuver_planner::ManeuverPlanner("maneuver_planner",costmap_ros_);    
    try{
        local_planner.initialize("TrajectoryPlannerROS", &tf_, local_costmap_ros);
    } catch(...) {
      // 
        ROS_FATAL("Failed to initialize the global planner");
        exit(1);
    }
    local_nav_state_ = LOC_NAV_IDLE;
    manv_nav_state_  = MANV_NAV_IDLE;
    
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    
    initialized_ = true;

};

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
    manv_nav_state_ = MANV_NAV_MAKE_INIT_PLAN;
    publishZeroVelocity();
    local_nav_state_ = LOC_NAV_IDLE;
    return true; // TODO: implement

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
    if(!costmap_ros_->getRobotPose(global_pose))
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
    switch(local_nav_state_){
        case LOC_NAV_IDLE:
            break;                    
        case LOC_NAV_SET_PLAN:
            
            if (!local_planner.setPlan(plan))
            {
                ROS_ERROR("Plan not set");
                local_nav_state_  = LOC_NAV_IDLE;
                manv_nav_state_   = MANV_NAV_IDLE;
            }
            else
                local_nav_state_ = LOC_NAV_BUSY;
            
            break;  
        case LOC_NAV_BUSY:  
            
            if(local_planner.isGoalReached())
            {
                ROS_INFO("local planner, Goal reached!");
                local_nav_state_ = LOC_NAV_IDLE;
                manv_nav_state_   = MANV_NAV_IDLE;
            }
            else if(local_planner.computeVelocityCommands(cmd_vel))
            {
                //make sure that we send the velocity command to the base
                vel_pub_.publish(cmd_vel);
            }
            else 
            {
                ROS_ERROR("local planner, The local planner could not find a valid plan.");
                publishZeroVelocity();        
                local_nav_state_ = LOC_NAV_IDLE;
                manv_nav_state_   = MANV_NAV_IDLE;
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
    double max_ahead_dist = 4.0;
    double dist_before_obs;  
    
    tf::Stamped<tf::Pose> global_pose;
    geometry_msgs::PoseStamped start;    
    
    switch(manv_nav_state_){
        case MANV_NAV_IDLE:
            break;          
        case MANV_NAV_MAKE_INIT_PLAN:                            
            
            if( !getRobotPose(global_pose) )
                break;
            
            tf::poseStampedTFToMsg(global_pose, start);              
            if( maneuver_planner.makePlan(start,goal_, plan) )
            {
                local_nav_state_ = LOC_NAV_SET_PLAN;
                manv_nav_state_  = MANV_NAV_BUSY;
            }
            else
            {
                ROS_ERROR("maneuver_navigation cannot make a plan due to obstacles");
                local_nav_state_ = LOC_NAV_IDLE;
                manv_nav_state_   = MANV_NAV_IDLE;
            }
            
            break;
         case MANV_NAV_BUSY:
             if( !checkFootprintOnGlobalPlan(plan,max_ahead_dist, dist_before_obs) )
             {
                ROS_INFO("Obstacle in front at %.2f m. Try to replan",dist_before_obs);
                
                if( !getRobotPose(global_pose) )
                    break;
                
                tf::poseStampedTFToMsg(global_pose, start);              
                if( maneuver_planner.makePlan(start,goal_, plan) )
                {
                    local_nav_state_ = LOC_NAV_SET_PLAN;
                    manv_nav_state_  = MANV_NAV_BUSY;
                }
                else
                {
                    ROS_INFO("No replan possible. Stop and inform");
                    publishZeroVelocity();        
                    local_nav_state_ = LOC_NAV_IDLE;
                    manv_nav_state_   = MANV_NAV_IDLE;
                }
                
                 
             }
             
            break;
        default:
            break;
    }    

};


 
}

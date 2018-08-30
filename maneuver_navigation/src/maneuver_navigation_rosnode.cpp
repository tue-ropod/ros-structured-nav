
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <string>


#include "maneuver_navigation.h"

#include <nav_msgs/Path.h>

#include <std_msgs/Bool.h>
#include <stdlib.h> 





// std::vector<geometry_msgs::PoseStamped> global_path;
bool goal_received = false;
geometry_msgs::PoseStamped goal;
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg)
{
    ROS_INFO("new goal received");
    goal = *goal_msg;
    goal_received = true;
}

bool reinit_planner_withload = false;
bool reinit_planner_noload = false;
void loadAttachedCallback(const std_msgs::Bool::ConstPtr& load_attached_msg)
{   
    ROS_INFO("Reinit PLanner");
    if( load_attached_msg->data == true )
        reinit_planner_withload = true;
    else
        reinit_planner_noload = true;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "route_navigation");
    ros::NodeHandle n("~");    
    
    double prediction_feasibility_check_rate, prediction_feasibility_check_period, prediction_feasibility_check_cycle_time = 0.0;
    double local_navigation_rate, local_navigation_period;    


    n.param<double>("prediction_feasibility_check_rate", prediction_feasibility_check_rate, 3.0);
    n.param<double>("local_navigation_rate", local_navigation_rate, 10.0); // local_navigation_rate>prediction_feasibility_check_rate
    ros::Rate rate(local_navigation_rate);
    prediction_feasibility_check_period = 1.0/prediction_feasibility_check_rate;
    local_navigation_period = 1.0/local_navigation_rate;
    
    ros::Subscriber goal_cmd_sub = n.subscribe<geometry_msgs::PoseStamped>("/route_navigation/goal", 10, goalCallback);
    ros::Subscriber reinit_planner_sub = n.subscribe<std_msgs::Bool>("/route_navigation/load_attached", 10, loadAttachedCallback);
   // ros::Publisher  reinit_localcostmap_footprint_sub = n.advertise<geometry_msgs::Polygon>("/maneuver_navigation/local_costmap/footprint", 1);
        
//     /move_base_simple/goal
    tf::TransformListener tf( ros::Duration(10) );
    mn::ManeuverNavigation maneuver_navigator(tf,n);
    maneuver_navigator.init();


    nav_msgs::Path path_msg;

    ROS_INFO("Wait for goal");
    
    while(n.ok())
    {
        prediction_feasibility_check_cycle_time += local_navigation_period;
        // Execute local navigation
        maneuver_navigator.callLocalNavigationStateMachine();
        
        if (goal_received)
        {
            goal_received = false;             
            maneuver_navigator.gotoGoal(goal);
            prediction_feasibility_check_cycle_time = prediction_feasibility_check_period;
        }   
        
        // Execute route navigation
        if( prediction_feasibility_check_cycle_time > prediction_feasibility_check_period)
        { 
            prediction_feasibility_check_cycle_time = 0.0;
            maneuver_navigator.callManeuverNavigationStateMachine();
        }
        
        if(reinit_planner_withload)
        {
            reinit_planner_withload = false;
            // TODO:: Read load footprint from file and do it asynchronously for not interrupting the therad. 0.01 is to avoid infeseability
            geometry_msgs::Polygon newfootprint;
            geometry_msgs::Point32 point_footprint;
            point_footprint.x = -0.1+0.01; point_footprint.y =  0.36-0.01; point_footprint.z = 0.0;
            newfootprint.points.push_back(point_footprint);
            point_footprint.x =  1.3-0.01; point_footprint.y =  0.36-0.01; point_footprint.z = 0.0;
            newfootprint.points.push_back(point_footprint);
            point_footprint.x =  1.3-0.01; point_footprint.y = -0.36+0.01; point_footprint.z = 0.0;
            newfootprint.points.push_back(point_footprint);
            point_footprint.x = -0.1+0.01; point_footprint.y = -0.36-0.01; point_footprint.z = 0.0;
            newfootprint.points.push_back(point_footprint);
            //reinit_localcostmap_footprint_sub.publish(newfootprint);
//             system("rosrun dynamic_reconfigure dynparam set /maneuver_navigation/local_costmap footprint '[[-0.1, 0.36], [1.3, 0.36], [1.3, -0.36], [-0.1, -0.36]]'");            
            
//             system("rosparam set /maneuver_navigation/TebLocalPlannerROS/footprint_model/vertices '[[-0.1, 0.36], [1.3, 0.36], [1.3, -0.36], [-0.1, -0.36]]'");
//             system("rosparam set /maneuver_navigation/TebLocalPlannerROS/max_global_plan_lookahead_dist 1.0");
//             system("rosparam set /maneuver_navigation/TebLocalPlannerROS/max_vel_x 1.0");
//             system("rosparam set /maneuver_navigation/TebLocalPlannerROS/xy_goal_tolerance 0.2");
//             system("rosparam set /maneuver_navigation/TebLocalPlannerROS/yaw_goal_tolerance 0.05");
            system("rosparam load ~/ropod-project-software/catkin_workspace/src/applications/ropod_navigation_test/config/parameters/teb_local_planner_params.yaml maneuver_navigation");

            maneuver_navigator.reinitPlanner(newfootprint);  // Dynamic reconfigurationdid not work so we had to do it in two ways. The localcostmap
            // was updated directly with functins, and the tebplanner by setting firts the parameters and then reloading the planner

        }           
        
        if(reinit_planner_noload)
        {
            reinit_planner_noload = false;
            // TODO:: Read load footprint from file and do it asynchronously for not interrupting the therad. 0.01 is to avoid infeseability
            geometry_msgs::Polygon newfootprint;
            geometry_msgs::Point32 point_footprint;
            point_footprint.x = -0.36+0.01; point_footprint.y =  0.36-0.01; point_footprint.z = 0.0;
            newfootprint.points.push_back(point_footprint);
            point_footprint.x =  0.36-0.01; point_footprint.y =  0.36-0.01; point_footprint.z = 0.0;
            newfootprint.points.push_back(point_footprint);
            point_footprint.x =  0.36-0.01; point_footprint.y = -0.36+0.01; point_footprint.z = 0.0;
            newfootprint.points.push_back(point_footprint);
            point_footprint.x = -0.36+0.01; point_footprint.y = -0.36+0.01; point_footprint.z = 0.0;
            newfootprint.points.push_back(point_footprint);
            //reinit_localcostmap_footprint_sub.publish(newfootprint);
//             system("rosrun dynamic_reconfigure dynparam set /maneuver_navigation/local_costmap footprint '[[-0.1, 0.36], [1.3, 0.36], [1.3, -0.36], [-0.1, -0.36]]'");            
            
//             system("rosparam set /maneuver_navigation/TebLocalPlannerROS/footprint_model/vertices '[[-0.36, 0.36], [0.36, 0.36], [0.36, -0.36], [-0.36, -0.36]]'");
//             system("rosparam set /maneuver_navigation/TebLocalPlannerROS/max_global_plan_lookahead_dist 1.5");
//             system("rosparam set /maneuver_navigation/TebLocalPlannerROS/max_vel_x 1.4");
//             system("rosparam set /maneuver_navigation/TebLocalPlannerROS/xy_goal_tolerance 0.1");
//             system("rosparam set /maneuver_navigation/TebLocalPlannerROS/yaw_goal_tolerance 0.05");

            system("rosparam load ~/ropod-project-software/catkin_workspace/src/applications/ropod_navigation_test/config/parameters/teb_local_planner_params_ropod.yaml maneuver_navigation");
            maneuver_navigator.reinitPlanner(newfootprint);  // Dynamic reconfigurationdid not work so we had to do it in two ways. The localcostmap
            // was updated directly with functins, and the tebplanner by setting firts the parameters and then reloading the planner            
        }

        ros::spinOnce();
        rate.sleep();
        if(rate.cycleTime() > ros::Duration(local_navigation_period) )
            ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", local_navigation_rate, rate.cycleTime().toSec());

    }



    return 0;
}

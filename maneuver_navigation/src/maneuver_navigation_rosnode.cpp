
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <string>


#include "maneuver_navigation.h"

#include <nav_msgs/Path.h>





// std::vector<geometry_msgs::PoseStamped> global_path;
bool goal_received = false;
geometry_msgs::PoseStamped goal;
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg)
{
    ROS_INFO("new goal received");
    goal = *goal_msg;
    goal_received = true;
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

     
        

        ros::spinOnce();
        rate.sleep();
        if(rate.cycleTime() > ros::Duration(local_navigation_period) )
            ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", local_navigation_rate, rate.cycleTime().toSec());

    }



    return 0;
}

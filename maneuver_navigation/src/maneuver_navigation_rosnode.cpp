
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
bool simple_goal_received = false;
geometry_msgs::PoseStamped simple_goal;
void simpleGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg)
{
    ROS_INFO("new simple goal received");
    simple_goal = *goal_msg;
    simple_goal_received = true;
}
bool goal_received = false;
maneuver_navigation::Goal goal;
void goalCallback(const maneuver_navigation::Goal::ConstPtr& goal_msg)
{
    ROS_INFO("new goal received");
    goal = *goal_msg;
    goal_received = true;
}

bool cancel_nav = false;
void cancelCallback(const std_msgs::Bool::ConstPtr& cancel_msg)
{
    ROS_INFO("Request to cancel navigation");
    cancel_nav = cancel_msg->data;
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
    
    std::string default_ropod_navigation_param_file;
    std::string default_ropod_load_navigation_param_file;


    n.param<double>("prediction_feasibility_check_rate", prediction_feasibility_check_rate, 3.0);    
    n.param<double>("local_navigation_rate", local_navigation_rate, 10.0); // local_navigation_rate>prediction_feasibility_check_rate    
    n.param<std::string>("default_ropod_navigation_param_file", default_ropod_navigation_param_file, 
                         std::string("") ); 
//                          std::string("~/ropod-project-software/catkin_workspace/src/functionalities/ros_structured_nav/maneuver_navigation/config/footprint_local_planner_params_ropod.yaml") ); 
    n.param<std::string>("default_ropod_load_navigation_param_file", default_ropod_load_navigation_param_file, 
                         std::string("") ); 
//                          std::string("~/ropod-project-software/catkin_workspace/src/functionalities/ros_structured_nav/maneuver_navigation/config/footprint_local_planner_params_ropod_load.yaml") ); 
    
    
    ros::Rate rate(local_navigation_rate);
    prediction_feasibility_check_period = 1.0/prediction_feasibility_check_rate;
    local_navigation_period = 1.0/local_navigation_rate;
    
    ros::Subscriber goal_cmd_sub = n.subscribe<geometry_msgs::PoseStamped>("/route_navigation/simple_goal", 10, simpleGoalCallback);
    ros::Subscriber mn_sendGoal_pub_ = n.subscribe<maneuver_navigation::Goal> ("/route_navigation/goal", 10,goalCallback);
    ros::Subscriber cancel_cmd_sub = n.subscribe<std_msgs::Bool>("/route_navigation/cancel", 10, cancelCallback);
    ros::Subscriber reinit_planner_sub = n.subscribe<std_msgs::Bool>("/route_navigation/set_load_attached", 10, loadAttachedCallback);
   // ros::Publisher  reinit_localcostmap_footprint_sub = n.advertise<geometry_msgs::Polygon>("/maneuver_navigation/local_costmap/footprint", 1);
    ros::Publisher  goal_visualisation_pub_ = n.advertise<geometry_msgs::PoseStamped>("/maneuver_navigation/goal_rviz", 1);
        
//     /move_base_simple/goal
    tf::TransformListener tf( ros::Duration(10) );
    mn::ManeuverNavigation maneuver_navigator(tf,n);
    maneuver_navigator.init();


    nav_msgs::Path path_msg;
    std::string load_param_str;

    ROS_INFO("Wait for goal");
    
    while(n.ok())
    {
        prediction_feasibility_check_cycle_time += local_navigation_period;
        // Execute local navigation
        maneuver_navigator.callLocalNavigationStateMachine();
        
        if (simple_goal_received)
        {
            simple_goal_received = false;             
            maneuver_navigator.gotoGoal(simple_goal);
            goal_visualisation_pub_.publish(simple_goal);
            prediction_feasibility_check_cycle_time = prediction_feasibility_check_period;
        }   
        
        if (goal_received)
        {
            goal_received = false;             
            maneuver_navigator.gotoGoal(goal);
            goal_visualisation_pub_.publish(goal.goal);
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
            geometry_msgs::Polygon new_footprint;
            geometry_msgs::Point32 point_footprint;
            
            load_param_str = "rosparam load "+ default_ropod_load_navigation_param_file +  " maneuver_navigation";
            system(load_param_str.c_str());
            std::vector<geometry_msgs::Point> new_footprint_vector;
            XmlRpc::XmlRpcValue footprint_xmlrpc;
            n.getParam("local_costmap/footprint", footprint_xmlrpc);                        
            new_footprint_vector = costmap_2d::makeFootprintFromXMLRPC(footprint_xmlrpc, "TebLocalPlannerROS/footprint_model/vertices");
            costmap_2d::padFootprint(new_footprint_vector,-0.01);
            for (std::vector<geometry_msgs::Point>::iterator it = new_footprint_vector.begin(); it != new_footprint_vector.end(); it++)
            {
                point_footprint.x = (*it).x; point_footprint.y =  (*it).y; point_footprint.z = 0.0;            
                new_footprint.points.push_back(point_footprint);        
                ROS_INFO("Footprint %f, %f", point_footprint.x, point_footprint.y);
            }
            maneuver_navigator.reinitPlanner(new_footprint);  // Dynamic reconfigurationdid not work so we had to do it in two ways. The localcostmap
            // was updated directly with functins, and the tebplanner by setting firts the parameters and then reloading the planner

        }           
        
        if(reinit_planner_noload)
        {
            reinit_planner_noload = false;
            geometry_msgs::Polygon new_footprint;
            geometry_msgs::Point32 point_footprint;
                    
            load_param_str = "rosparam load "+ default_ropod_navigation_param_file +  " maneuver_navigation";
            system(load_param_str.c_str());
            std::vector<geometry_msgs::Point> new_footprint_vector;
            XmlRpc::XmlRpcValue footprint_xmlrpc;
            n.getParam("local_costmap/footprint", footprint_xmlrpc);            
            new_footprint_vector = costmap_2d::makeFootprintFromXMLRPC(footprint_xmlrpc, "TebLocalPlannerROS/footprint_model/vertices");
            costmap_2d::padFootprint(new_footprint_vector,-0.01);
            for (std::vector<geometry_msgs::Point>::iterator it = new_footprint_vector.begin(); it != new_footprint_vector.end(); it++)
            {
                point_footprint.x = (*it).x; point_footprint.y =  (*it).y; point_footprint.z = 0.0;            
                new_footprint.points.push_back(point_footprint);     
            }
            maneuver_navigator.reinitPlanner(new_footprint);  // Dynamic reconfigurationdid not work so we had to do it in two ways. The localcostmap
            // was updated directly with functins, and the tebplanner by setting first the parameters and then reloading the planner            
        }
        
        if(cancel_nav)
        {
            cancel_nav = false;
            maneuver_navigator.cancel();
        }

        ros::spinOnce();
        rate.sleep();
        if(rate.cycleTime() > ros::Duration(local_navigation_period) )
            ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", local_navigation_rate, rate.cycleTime().toSec());

    }



    return 0;
}


#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <string>
#include "napoleon_functions.h"
#include "napoleon_functions.cpp"
#include "napoleon_driving.h"

#define _USE_MATH_DEFINES
#include <math.h>

#include <nav_msgs/Path.h>

#include <std_msgs/Bool.h>
#include <stdlib.h> 

float ropod_x = 0, ropod_y = 0, quaternion_x = 0, quaternion_y = 0, quaternion_z = 0, quaternion_w = 0, ropod_theta = 0;

//std::vector<geometry_msgs::PoseStamped> global_path;
bool simple_goal_received = false;
geometry_msgs::PoseStamped simple_goal;
void simpleGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg)
{
    ROS_INFO("new simple goal received");
    simple_goal = *goal_msg;
    simple_goal_received = true;
}
bool goal_received = false;
napoleon_driving::Goal goal;
void goalCallback(const napoleon_driving::Goal::ConstPtr& goal_msg)
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

void getAmclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
    //ROS_INFO("Amcl pose received");
    //ROS_INFO("X: %f, Y: %f", pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y);
    ropod_x = pose_msg->pose.pose.position.x;
    ropod_y = pose_msg->pose.pose.position.y;
    quaternion_x = pose_msg->pose.pose.orientation.x;
    quaternion_y = pose_msg->pose.pose.orientation.y;
    quaternion_z = pose_msg->pose.pose.orientation.z;
    quaternion_w = pose_msg->pose.pose.orientation.w;
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
//                          std::string("~/ropod-project-software/catkin_workspace/src/functionalities/ros_structured_nav/napoleon_driving/config/footprint_local_planner_params_ropod.yaml") ); 
    n.param<std::string>("default_ropod_load_navigation_param_file", default_ropod_load_navigation_param_file, 
                         std::string("") ); 
//                          std::string("~/ropod-project-software/catkin_workspace/src/functionalities/ros_structured_nav/napoleon_driving/config/footprint_local_planner_params_ropod_load.yaml") ); 
    
    
    ros::Rate rate(local_navigation_rate);
    prediction_feasibility_check_period = 1.0/prediction_feasibility_check_rate;
    local_navigation_period = 1.0/local_navigation_rate;
    
    ros::Subscriber goal_cmd_sub = n.subscribe<geometry_msgs::PoseStamped>("/route_navigation/simple_goal", 10, simpleGoalCallback);
    ros::Subscriber mn_sendGoal_pub_ = n.subscribe<napoleon_driving::Goal> ("/route_navigation/goal", 10,goalCallback);
    ros::Subscriber cancel_cmd_sub = n.subscribe<std_msgs::Bool>("/route_navigation/cancel", 10, cancelCallback);
    ros::Subscriber reinit_planner_sub = n.subscribe<std_msgs::Bool>("/route_navigation/set_load_attached", 10, loadAttachedCallback);
    ros::Subscriber amcl_pose_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, getAmclPoseCallback);
    //ros::Publisher  reinit_localcostmap_footprint_sub = n.advertise<geometry_msgs::Polygon>("/napoleon_driving/local_costmap/footprint", 1);
    ros::Publisher goal_visualisation_pub_ = n.advertise<geometry_msgs::PoseStamped>("/napoleon_driving/goal_rviz", 1);

//     /move_base_simple/goal
    tf::TransformListener tf( ros::Duration(10) );
    mn::NapoleonDriving napoleon_driving(tf,n);
    napoleon_driving.init();


    nav_msgs::Path path_msg;
    std::string load_param_str;

    ROS_INFO("Wait for goal");

    //napoleon_driving.publishOneVelocity();
    
    while(n.ok())
    {

        prediction_feasibility_check_cycle_time += local_navigation_period;
        // Execute local navigation
        napoleon_driving.callLocalNavigationStateMachine();
        
        if (simple_goal_received)
        {
            //simple_goal_received = false;             
            //napoleon_driving.publishOneVelocity();
            ROS_INFO("X_ropod: %f", ropod_x);
            ROS_INFO("Y_ropod: %f", ropod_y);

            // yaw (z-axis rotation)
            double siny_cosp = +2.0 * (quaternion_w * quaternion_z + quaternion_x * quaternion_y);
            double cosy_cosp = +1.0 - 2.0 * (quaternion_y * quaternion_y + quaternion_z * quaternion_z);  
            ropod_theta = atan2(siny_cosp, cosy_cosp);

            ROS_INFO("Theta ropod: %f", ropod_theta);

            Point top(3.98, 9.18);
            Point right(13.04, 7.00);
            Point bottom(11.14, -1.71);
            Point left(1.85, 0.57);

            Point glob_wallpoint_front, glob_wallpoint_rear;

            if (ropod_x <= 4.5 && ropod_y > 2.3){
                glob_wallpoint_front = left;
                glob_wallpoint_rear = top;
            } else if (ropod_x <= 9 && ropod_y <= 2.3){
                glob_wallpoint_front = bottom;
                glob_wallpoint_rear = left;
            } else if (ropod_x > 9 && ropod_y <= 5.5){
                glob_wallpoint_front = right;
                glob_wallpoint_rear = bottom;
            } else if (ropod_x > 4.5 && ropod_y > 5.5){
                glob_wallpoint_front = top;
                glob_wallpoint_rear = right;
            }

            //Point glob_wallpoint_front(1.85, 0.57);
            //Point glob_wallpoint_rear(3.98,9.18);
            Point ropod_pos(ropod_x, ropod_y);

            Point local_wallpoint_front = coordGlobalToRopod(glob_wallpoint_front, ropod_pos, ropod_theta);
            Point local_wallpoint_rear = coordGlobalToRopod(glob_wallpoint_rear, ropod_pos, ropod_theta);

            ROS_INFO("Local point front: [ %f , %f ]", local_wallpoint_front.x, local_wallpoint_front.y);
            ROS_INFO("Local point rear: [ %f , %f ]", local_wallpoint_rear.x, local_wallpoint_rear.y);

            double pred_phi_des = getSteering(local_wallpoint_front, local_wallpoint_rear);
            double v_des = v_cruising;

            ROS_INFO("Phi: %f, v_des: %f", pred_phi_des, v_des);

            double v_ax = cos(pred_phi_des)*v_des;
            double theta_dot = v_des/d_ax*sin(pred_phi_des);

            napoleon_driving.publishCustomVelocity(v_ax, theta_dot);
        }   
        
        if (goal_received)
        {
            goal_received = false;             
            napoleon_driving.gotoGoal(goal);
            goal_visualisation_pub_.publish(goal.goal);
            prediction_feasibility_check_cycle_time = prediction_feasibility_check_period;
        }          
        
        // Execute route navigation
        if( prediction_feasibility_check_cycle_time > prediction_feasibility_check_period)
        { 
            prediction_feasibility_check_cycle_time = 0.0;
            napoleon_driving.callNapoleonDrivingStateMachine();
        }
        
        
        if(reinit_planner_withload)
        {
            reinit_planner_withload = false;
            // TODO:: Read load footprint from file and do it asynchronously for not interrupting the therad. 0.01 is to avoid infeseability
            geometry_msgs::Polygon new_footprint;
            geometry_msgs::Point32 point_footprint;
            
            load_param_str = "rosparam load "+ default_ropod_load_navigation_param_file +  " napoleon_driving";
            system(load_param_str.c_str());
            std::vector<geometry_msgs::Point> new_footprint_vector;
            XmlRpc::XmlRpcValue footprint_xmlrpc;
            n.getParam("local_costmap/footprint", footprint_xmlrpc);                        
            //new_footprint_vector = costmap_2d::makeFootprintFromXMLRPC(footprint_xmlrpc, "TebLocalPlannerROS/footprint_model/vertices");
            costmap_2d::padFootprint(new_footprint_vector,-0.01);
            for (std::vector<geometry_msgs::Point>::iterator it = new_footprint_vector.begin(); it != new_footprint_vector.end(); it++)
            {
                point_footprint.x = (*it).x; point_footprint.y =  (*it).y; point_footprint.z = 0.0;            
                new_footprint.points.push_back(point_footprint);        
                ROS_INFO("Footprint %f, %f", point_footprint.x, point_footprint.y);
            }
            //napoleon_driving.reinitPlanner(new_footprint);  // Dynamic reconfigurationdid not work so we had to do it in two ways. The localcostmap
            // was updated directly with functins, and the tebplanner by setting firts the parameters and then reloading the planner

        }           
        
        if(reinit_planner_noload)
        {
            reinit_planner_noload = false;
            geometry_msgs::Polygon new_footprint;
            geometry_msgs::Point32 point_footprint;
                    
            load_param_str = "rosparam load "+ default_ropod_navigation_param_file +  " napoleon_driving";
            system(load_param_str.c_str());
            std::vector<geometry_msgs::Point> new_footprint_vector;
            XmlRpc::XmlRpcValue footprint_xmlrpc;
            n.getParam("local_costmap/footprint", footprint_xmlrpc);            
            //new_footprint_vector = costmap_2d::makeFootprintFromXMLRPC(footprint_xmlrpc, "TebLocalPlannerROS/footprint_model/vertices");
            costmap_2d::padFootprint(new_footprint_vector,-0.01);
            for (std::vector<geometry_msgs::Point>::iterator it = new_footprint_vector.begin(); it != new_footprint_vector.end(); it++)
            {
                point_footprint.x = (*it).x; point_footprint.y =  (*it).y; point_footprint.z = 0.0;            
                new_footprint.points.push_back(point_footprint);     
            }
            //napoleon_driving.reinitPlanner(new_footprint);  // Dynamic reconfigurationdid not work so we had to do it in two ways. The localcostmap
            // was updated directly with functins, and the tebplanner by setting first the parameters and then reloading the planner            
        }
        
        if(cancel_nav)
        {
            cancel_nav = false;
            napoleon_driving.cancel();
        }
        
        //napoleon_driving.publishOneVelocity();
        
        ros::spinOnce();
        rate.sleep();
        if(rate.cycleTime() > ros::Duration(local_navigation_period) ){
            ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", local_navigation_rate, rate.cycleTime().toSec());
        }
        
    }



    return 0;
}

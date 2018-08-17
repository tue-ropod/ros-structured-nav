

#include "maneuver_navigation_ed.h"



// std::vector<geometry_msgs::PoseStamped> global_path;
bool goal_received = false;
geometry_msgs::PoseStamped goal;
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal_msg)
{
    ROS_INFO("new goal received");
    goal = *goal_msg;
    goal_received = true;
}

ManeuverNavigationED::ManeuverNavigationED()
{
}

// ----------------------------------------------------------------------------------------------------

ManeuverNavigationED::~ManeuverNavigationED()
{
}

void ManeuverNavigationED::initialize(ed::InitData& init)
{
    //ros::init(argc, argv, "route_navigation");
    ros::NodeHandle n("~");    
    
   // double prediction_feasibility_check_rate, prediction_feasibility_check_period, prediction_feasibility_check_cycle_time = 0.0;
   // double local_navigation_rate, local_navigation_period; 
    
   prediction_feasibility_check_cycle_time_ = 0.0;

    n.param<double>("prediction_feasibility_check_rate", prediction_feasibility_check_rate_, 2.0);
    n.param<double>("local_navigation_rate", local_navigation_rate_, 10.0); // local_navigation_rate>prediction_feasibility_check_rate
    ros::Rate rate(local_navigation_rate_);
    prediction_feasibility_check_period_ = 1.0/prediction_feasibility_check_rate_;
    local_navigation_period_ = 1.0/local_navigation_rate_;
    
    ros::Subscriber goal_cmd_sub = n.subscribe<geometry_msgs::PoseStamped>("/route_navigation/goal", 10, goalCallback);
        
//     /move_base_simple/goal
    tf::TransformListener tf( ros::Duration(10) );
    mn::ManeuverNavigation maneuver_navigator(tf,n);
    maneuver_navigator_ = maneuver_navigator;
    maneuver_navigator_.init();


    nav_msgs::Path path_msg;

    ROS_INFO("Wait for goal");
    
}
   // while(n.ok())
 //   {
 ManeuverNavigationED::process(const ed::WorldModel& world, ed::UpdateRequest& req)
 {
        prediction_feasibility_check_cycle_time_ += local_navigation_period_;
        // Execute local navigation
        maneuver_navigator_.callLocalNavigationStateMachine();
        
        // Execute route navigation
        if( prediction_feasibility_check_cycle_time_ > prediction_feasibility_check_period_)
        { 
            prediction_feasibility_check_cycle_time_ = 0.0;
            maneuver_navigator_.callManeuverNavigationStateMachine();
        }

        if (goal_received)
        {
            goal_received = false;             
        
            maneuver_navigator_.gotoGoal(goal);
        }        

      //  ros::spinOnce();
     //   rate.sleep();
//         if(rate.cycleTime() > ros::Duration(local_navigation_period_) )
//             ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", local_navigation_rate, rate.cycleTime().toSec());

    }



}

ED_REGISTER_PLUGIN(ManeuverNavigationED)

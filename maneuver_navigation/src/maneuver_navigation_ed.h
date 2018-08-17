#ifndef ED_MANEUVER_NAVIGATION_PLUGIN_H_
#define ED_MANEUVER_NAVIGATION_PLUGIN_H_

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <string>


#include "maneuver_navigation.h"

#include <nav_msgs/Path.h>
#include <ed/plugin.h>

class ManeuverNavigationED : public ed::Plugin
{

public:

    ManeuverNavigationED();

    ~ManeuverNavigationED();

    void initialize(ed::InitData& init);

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:

    double prediction_feasibility_check_rate_;
    
    double prediction_feasibility_check_period_;
    
    double prediction_feasibility_check_cycle_time_;
    
    double local_navigation_rate_;
    
    double local_navigation_period_;    
    
    mn::ManeuverNavigation maneuver_navigator_;


};

#endif
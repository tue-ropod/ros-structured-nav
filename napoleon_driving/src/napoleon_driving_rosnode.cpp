
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <string>
#include "napoleon_functions.h"
#include "napoleon_functions.cpp"
#include "napoleon_driving.h"
#include "napoleon_config.h"
#include <algorithm>    // std::rotate
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <ctime>

#include <ed_gui_server/objPosVel.h>
#include <ed_gui_server/objsPosVel.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include <nav_msgs/Path.h>

#include <std_msgs/Bool.h>
#include <stdlib.h> 

double ropod_x = 0, ropod_y = 0, ropod_theta = 0;
double this_amcl_x = 0, this_amcl_y = 0, quaternion_x = 0, quaternion_y = 0, quaternion_z = 0, quaternion_w = 0, this_amcl_theta = 0, siny_cosp = 0, cosy_cosp = 0;
double odom_xdot_ropod_global = 0, odom_ydot_ropod_global = 0, odom_thetadot_global = 0, odom_phi_local = 0, odom_phi_global = 0, odom_vropod_global = 0;
int no_obs = 0;
ed_gui_server::objPosVel current_obstacle;
double obs_theta;
Point obs_center_global;
double program_duration = 0, real_time_est = 0;

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

void getObstaclesCallback(const ed_gui_server::objsPosVel::ConstPtr& obsarray) {
    no_obs = obsarray->objects.size();
    //ROS_INFO("%d obstacles detected", no_obs);
    //string s;
    // For the sake of proof of concept, the assumption is made that there will
    // only be one obstacle to avoid or overtake.
    // This scenario is not realistic and only serves as showcase.
    // A counter will decide which obstacle to choose
    // for (int q = 0; q < no_obs; ++q) {
    //     s = obsarray->objects[q].id;
    //     std::cout << s << std::endl;
    //     current_obstacle = obsarray->objects[q];
    // }
    // For now just pick first obs if there is obs
    // so we assume we only see the obstacle we want to see
    if (no_obs > 0) {
        // If no other obstacle is seen, this obstacle is kept
        current_obstacle = obsarray->objects[0];
        // ROS_INFO("Obs is %f wide and %f deep", current_obstacle.width, current_obstacle.depth);
        // ROS_INFO("Obs x: %f, obs y: %f", current_obstacle.pose.position.x, current_obstacle.pose.position.y);
        // ROS_INFO("Vx: %f, Vy %f", current_obstacle.vel.x, current_obstacle.vel.y);
        quaternion_x = obsarray->objects[0].pose.orientation.x;
        quaternion_y = obsarray->objects[0].pose.orientation.y;
        quaternion_z = obsarray->objects[0].pose.orientation.z;
        quaternion_w = obsarray->objects[0].pose.orientation.w;

        // yaw (z-axis rotation)
        siny_cosp = +2.0 * (quaternion_w * quaternion_z + quaternion_x * quaternion_y);
        cosy_cosp = +1.0 - 2.0 * (quaternion_y * quaternion_y + quaternion_z * quaternion_z);  
        obs_theta = atan2(siny_cosp, cosy_cosp);
        obs_center_global.x = current_obstacle.pose.position.x;
        obs_center_global.y = current_obstacle.pose.position.y;
    }
}

void getOdomVelCallback(const nav_msgs::Odometry::ConstPtr& odom_vel) 
{
    odom_xdot_ropod_global = odom_vel->twist.twist.linear.x;
    odom_ydot_ropod_global = odom_vel->twist.twist.linear.y;
    odom_thetadot_global = odom_vel->twist.twist.angular.z;
    // rmstart (buggy)
    // odom_phi_global = atan(D_AX/odom_xdot_ropod_global*odom_thetadot_global);
    // odom_vropod_global = odom_xdot_ropod_global/cos(odom_phi_global);
    // rstart
    odom_phi_local = atan2(odom_ydot_ropod_global, odom_xdot_ropod_global);
    odom_vropod_global = sqrt(odom_xdot_ropod_global*odom_xdot_ropod_global+odom_ydot_ropod_global*odom_ydot_ropod_global);
    //ROS_INFO("xdot: %f, ydot: %f, vabs: %f", odom_xdot_ropod_global, odom_ydot_ropod_global, odom_vropod_global);
}

void getAmclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
    //ROS_INFO("Amcl pose received");
    //ROS_INFO("X: %f, Y: %f", pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y);
    this_amcl_x = pose_msg->pose.pose.position.x;
    this_amcl_y = pose_msg->pose.pose.position.y;
    quaternion_x = pose_msg->pose.pose.orientation.x;
    quaternion_y = pose_msg->pose.pose.orientation.y;
    quaternion_z = pose_msg->pose.pose.orientation.z;
    quaternion_w = pose_msg->pose.pose.orientation.w;

    // yaw (z-axis rotation)
    siny_cosp = +2.0 * (quaternion_w * quaternion_z + quaternion_x * quaternion_y);
    cosy_cosp = +1.0 - 2.0 * (quaternion_y * quaternion_y + quaternion_z * quaternion_z);  
    this_amcl_theta = atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "route_navigation");
    ros::NodeHandle nroshndl("~");    
    
    double prediction_feasibility_check_rate, prediction_feasibility_check_period, prediction_feasibility_check_cycle_time = 0.0;
    double local_navigation_rate, local_navigation_period;    
    
    std::string default_ropod_navigation_param_file;
    std::string default_ropod_load_navigation_param_file;

    // Initialize environment (turn left)
    // PointID A(11.12,-1.66,"A"), B(13.04,7.06,"B"), C(3.94,9.23,"C"), D(1.85,0.49,"D"),
    //         E(9.33,1.00,"E"), F(10.35,5.41,"F"), G(5.87,6.93,"G"), H(4.72,2.11,"H"),
    //         K(8.89,-1.3,"K"), L(11.62,0.42,"L"), M(12.66,4.85,"M"), N(10.93,7.76,"N"),
    //         P(6.32,8.78,"P"), Q(3.38,7.46,"Q"), R(2.25,2.76,"R"), S(4.05,-0.13,"S");

    // Initialize environment (turn right)
    PointID A(11.12,-1.66,"A"), B(13.14,7.26,"B"), C(3.94,9.23,"C"), D(1.85,0.49,"D"),
            E(9.27,0.70,"E"), F(10.45,5.71,"F"), G(5.77,6.98,"G"), H(4.60,1.81,"H"),
            K(8.89,-1.3,"K"), L(11.62,0.42,"L"), M(12.68,5.02,"M"), N(10.93,7.76,"N"),
            P(6.32,8.78,"P"), Q(3.38,7.46,"Q"), R(2.25,2.76,"R"), S(4.05,-0.13,"S");

    AreaQuadID area44(E,H,S,K,44,"hallway"), area46(L,M,F,E,46,"hallway"), 
                area48(N,P,G,F,48,"hallway"), area50(Q,R,H,G,50,"hallway"),
                area45(A,L,E,K,45,"inter"), area47(M,B,N,F,47,"inter"), 
                area49(P,C,Q,G,49,"inter"), area51(R,D,S,H,51,"inter");

    std::vector<PointID> pointlist {A, B, C, D, E, F, G, H, K, L, M, N, P, Q, R, S};
    std::vector<AreaQuadID> arealist {area44, area45, area46, area47, area48, area49, area50, area51};

    int assignment[] = {50,51,44,45,46,47,48,49,50};
    //int assignment[] = {50,49,48,47,46,45,44,51,50}; // reversed (right turns)

    nroshndl.param<double>("prediction_feasibility_check_rate", prediction_feasibility_check_rate, 3.0);    
    nroshndl.param<double>("local_navigation_rate", local_navigation_rate, 10.0); // local_navigation_rate>prediction_feasibility_check_rate    
    nroshndl.param<std::string>("default_ropod_navigation_param_file", default_ropod_navigation_param_file, 
                         std::string("") ); 
//                          std::string("~/ropod-project-software/catkin_workspace/src/functionalities/ros_structured_nav/napoleon_driving/config/footprint_local_planner_params_ropod.yaml") ); 
    nroshndl.param<std::string>("default_ropod_load_navigation_param_file", default_ropod_load_navigation_param_file, 
                         std::string("") ); 
//                          std::string("~/ropod-project-software/catkin_workspace/src/functionalities/ros_structured_nav/napoleon_driving/config/footprint_local_planner_params_ropod_load.yaml") ); 
    
    ros::Rate rate(local_navigation_rate);
    prediction_feasibility_check_period = 1.0/prediction_feasibility_check_rate;
    local_navigation_period = 1.0/local_navigation_rate;
    
    ros::Subscriber goal_cmd_sub = nroshndl.subscribe<geometry_msgs::PoseStamped>("/route_navigation/simple_goal", 10, simpleGoalCallback);
    ros::Subscriber mn_sendGoal_pub_ = nroshndl.subscribe<napoleon_driving::Goal> ("/route_navigation/goal", 10, goalCallback);
    ros::Subscriber cancel_cmd_sub = nroshndl.subscribe<std_msgs::Bool>("/route_navigation/cancel", 10, cancelCallback);
    ros::Subscriber reinit_planner_sub = nroshndl.subscribe<std_msgs::Bool>("/route_navigation/set_load_attached", 10, loadAttachedCallback);
    ros::Subscriber amcl_pose_sub = nroshndl.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, getAmclPoseCallback);
    ros::Subscriber ropod_odom_sub = nroshndl.subscribe<nav_msgs::Odometry>("/ropod/odom", 100, getOdomVelCallback);
    ros::Subscriber obstacle_sub = nroshndl.subscribe<ed_gui_server::objsPosVel>("/ed/gui/objectPosVel", 10, getObstaclesCallback);
    //ros::Publisher  reinit_localcostmap_footprint_sub = nroshndl.advertise<geometry_msgs::Polygon>("/napoleon_driving/local_costmap/footprint", 1);
    ros::Publisher goal_visualisation_pub_ = nroshndl.advertise<geometry_msgs::PoseStamped>("/napoleon_driving/goal_rviz", 1);
    ros::Publisher ropodmarker_pub = nroshndl.advertise<visualization_msgs::Marker>("/napoleon_driving/ropodpoints", 1);

    // Visualize map nodes
    ros::Publisher mapmarker_pub = nroshndl.advertise<visualization_msgs::Marker>("/napoleon_driving/vmnodes", 10, true);

    visualization_msgs::Marker vis_points;
    vis_points.header.frame_id = "/map";
    vis_points.header.stamp = ros::Time::now();
    // vis_points.ns = line_strip.ns = line_list.ns = "points_in_map";
    vis_points.action = visualization_msgs::Marker::ADD;
    vis_points.pose.orientation.w = 1.0;
    vis_points.id = 0;
    vis_points.color.a = 1.0;
    vis_points.color.g = 1.0;
    vis_points.type = visualization_msgs::Marker::POINTS;
    vis_points.scale.x = 0.2;
    vis_points.scale.y = 0.2;
    geometry_msgs::Point vis_p;

    for (int imap = 0; imap < pointlist.size(); ++imap)
    {
        vis_p.x = pointlist[imap].x;
        vis_p.y = pointlist[imap].y;
        vis_p.z = 0;
        vis_points.points.push_back(vis_p);
    }

    mapmarker_pub.publish(vis_points);
    // End visualize map nodes

    tf::TransformListener tf( ros::Duration(10) );
    mn::NapoleonDriving napoleon_driving(tf,nroshndl);
    napoleon_driving.init();


    nav_msgs::Path path_msg;
    std::string load_param_str;

    ROS_INFO("Wait for 2D Nav Goal to start (goal can be anywhere, doesn't influence program)");
    
    // Initial values, not very important as long as ropod is in the right area
    // First action will be determined by the initial values (or I can just wait?)
    double control_v = 0;
    double theta_0 = -M_PI/2;     // Initial orientation of ropod [rad]
    double x_ropod_0 = 4.4;	    // X position of center of ropod [m]
    double y_ropod_0 = 8.8;     // Y position of center of ropod [m]
    double v_ropod_0 = 0.0;     // Velocity of ropod [m/s]
    double phi_0 = 0.00;        // Steering angle - CCW positive - 0 when steering straight [rad]
    double phi_dot_0 = 0.0;     // Steering velocity [rad/s]
    double x_rearax_0 = x_ropod_0 - D_AX*cos(theta_0); // X position of center of rear axle [m]
    double y_rearax_0 = y_ropod_0 - D_AX*sin(theta_0); // Y position of center of rear axle [m]
    double x_rearax; // X position of center of rear axle [m]
    double y_rearax; // Y position of center of rear axle [m]
    double prev_amcl_x = 0; // Used to update position with amcl or 'predict' new pose
    double prev_amcl_y = 0;
    double prev_amcl_theta = 0;

    static constexpr int size_m = F_MODEL*(T_MAX_PRED+1)+1;  // Array size for predictions that run at F_MODEL
    static constexpr int size_p = F_PLAN*(T_MAX_PRED+1)+1;   // Array size for predictions that run at F_PLAN
    double t_pred[size_m] {0};              // Prediction time [s]
    double t_pred_j[size_p] {0};              // Prediction time planning [s]
    double pred_v_ropod[size_m] {v_ropod_0};
    double pred_v_ropod_plan[size_p] {pred_v_ropod[0]};
    bool pred_ropod_on_entry[size_p] {false};
    //std::vector<double> sim_v_ropod {pred_v_ropod[0]};
    double pred_x_ropod[size_p] {x_ropod_0};
    double pred_y_ropod[size_p] {y_ropod_0};
    double pred_x_obs[size_p] {0};
    double pred_y_obs[size_p] {0};
    double v_obs_sq = 0;
    //std::vector<double> sim_x_rearax {x_rearax_0};
    //std::vector<double> sim_y_rearax {y_rearax_0};
    double prev_sim_phi_des;
    double pred_plan_theta[size_p] {theta_0};
    double pred_accel[size_p] {0};
    std::vector<double> sim_theta {pred_plan_theta[0]};
    std::vector<double> sim_phi {phi_0};
    //std::vector<double> sim_phi_dot {phi_dot_0};
    std::vector<double> sim_v_scale {1};
    double prev_sim_tube_width {TUBE_WIDTH_C};
    //std::vector<double> sim_xdot {cos(sim_theta[0])*pred_v_ropod[0]*cos(sim_phi[0])};
    //std::vector<double> sim_ydot {sin(sim_theta[0])*pred_v_ropod[0]*cos(sim_phi[0])};
    //std::vector<double> sim_thetadot {1/D_AX*pred_v_ropod[0]*sin(sim_phi[0])};
    std::vector<std::string> walls;
    double pred_phi[size_m];
    double pred_theta[size_m];
    double pred_xdot[size_m];
    double pred_ydot[size_m];
    double pred_thetadot[size_m];
    double pred_x_rearax[size_m];
    double pred_y_rearax[size_m];
    double prev_pred_phi_des;
    double pred_phi_des[size_p] {0};
    double pred_tube_width[size_p] {TUBE_WIDTH_C};
    Point pred_ropod_rb, pred_ropod_lb, pred_ropod_rt, pred_ropod_lt,
    pred_ropod_dil_rb, pred_ropod_dil_lb, pred_ropod_dil_lt, pred_ropod_dil_rt,
    obslt, obsrt, obsrb, obslb;
    Point vis_rt, vis_lt, vis_fr, vis_fl;
    bool update_state_points;
    bool dir_cw;
    double v_des_scaled[size_p] {0};
    double vel_dif, v_new;
    double t_pred_prev;
    double dist_to_middle_final;
    bool pred_ropod_colliding_obs[size_p] {false};
    Point OBJ_LAST_center;
    Point current_obs_in_ropod_frame_pos, obs_in_ropod_frame_pos;

    PointID rw_p_rear, rw_p_front, lw_p_rear, lw_p_front; 
    AreaQuadID cur_obj = getAreaByID(assignment[0],arealist); 
    vector<string> areaIDs; 
    int ind, qmax;
    double space_left, space_right, shift_wall, wallang;;
    
    //std::vector<int> sim_state {1};   // State of ropod
    int prev_sim_state = 1;
    int pred_state[size_p] {1};
    int prev_sim_task_counter = 0;
    int pred_task_counter[size_p] {0};
    int danger_count = 0; // Counter that ensures program stops if collision with wall predicted for sometime

    // Constants during simulation
    //double delta_t = 1/(double)F_PLAN;                      // Time ropod will execute this plan
    double delta_t = local_navigation_period;
    double max_delta_v = A_MAX*delta_t;             // Maximum change in v in delta_t
    double lpf = 2*M_PI*TS*CUTOFF_FREQ;             // Low pass filter [-]
    if (lpf > 1) {
        lpf = 1;        // Saturate
    }
    bool ropod_reached_target = false;
    // plot_pred_new = true;
    // prev_m = [];

    // Counters
    int i = 0; // - simulation/experiment plan
    int n = 1; // - simulation/experiment movement
    int k = -1; //- velocity scaling
    int q = 1; // - for loop for movement simulation (in prediction)
    int j = 0; // - prediction plan
    int m = 0; // - prediction movement
    int u = 0; // - Pred task counter
    int prevstate; // Actually just j-1
    int m_prev;

    static constexpr int ka_max = sizeof(assignment)/sizeof(assignment[0]);  // Assignment length

    AreaQuadID OBJ_FIRST = getAreaByID(assignment[0],arealist);
    AreaQuadID OBJ_LAST = getAreaByID(assignment[ka_max-1],arealist);
    AreaQuadID current_hallway = getAreaByID(assignment[0],arealist);
    AreaQuadID next_hallway = getAreaByID(assignment[2],arealist);
    Point last_obj_center = OBJ_LAST.center();
    PointID current_pivot, cur_next_hallway_rear, cur_next_hallway_front;
    PointID current_inter_rear_wall, current_inter_front_wall;
    Point pred_xy_ropod_0(pred_x_ropod[0], pred_y_ropod[0]);
    std::array<Point, size_p> pred_xy_ropod {{pred_xy_ropod_0}};
    //Point pred_xy_ropod(pred_x_ropod[0], pred_y_ropod[0]);
    Point cur_pivot_local, cur_next_hallway_rear_local, cur_next_hallway_front_local;
    Point current_inter_rear_wall_local, current_inter_front_wall_local;
    double cur_next_hallway_angle;
    PointID wall_front_p0, wall_front_p1;
    Point local_wall_front_p0, local_wall_front_p1;
    Point local_front_ropod_dilated_p0(DILATE_ROPOD_ALIGNING, -DILATE_ROPOD_ALIGNING);
    Point local_front_ropod_dilated_p1(DILATE_ROPOD_ALIGNING, DILATE_ROPOD_ALIGNING);
    PointID point_rear, point_front;
    Point glob_wallpoint_front, glob_wallpoint_rear;
    Point global_wall_front_p0, global_wall_front_p1;
    PointID point_pivot;
    Point local_wallpoint_front, local_wallpoint_rear;
    Point local_pivot;
    
    bool sharp_corner[ka_max] = {false};

    array<array<string, 6>, ka_max> OBJ_X_TASK; // Assuming this initializes empty strings (otherwise wont work)
    std::array<std::string, 6> task1, task2, task3;
    int area1, area2, area3;

    std::vector<std::string> OBJ1TASK, OBJ2TASK, OBJ_LAST_TASK;
    AreaQuadID OBJ1 = getAreaByID(assignment[0],arealist);
    AreaQuadID OBJ2 = getAreaByID(assignment[1],arealist);
    AreaQuadID OBJ3 = getAreaByID(assignment[2],arealist);
    int obj2tasklen;

    PointID obj2wall_p0, obj2wall_p1, obj3wall_p0, obj3wall_p1;
    double obj2frontwall_angle, obj3wall_angle, relative_angle;
    
    ROS_INFO("Printing assignment nodes:");
    for (int ka = 0; ka < ka_max-1; ka = ka+2) {
        OBJ1 = getAreaByID(assignment[ka],arealist); 
        OBJ2 = getAreaByID(assignment[ka+1],arealist); 
        OBJ3 = getAreaByID(assignment[ka+2],arealist); 
        OBJ1TASK = getWallPointsTowardsB(OBJ1,OBJ2);
        printstringvec(OBJ1TASK);
        OBJ_X_TASK[ka][0] = OBJ1TASK[0]; 
        OBJ_X_TASK[ka][1] = OBJ1TASK[1]; 
        OBJ2TASK = getPointsForTurning(OBJ1,OBJ2,OBJ3,OBJ1TASK); 
        printstringvec(OBJ2TASK);
        obj2tasklen = OBJ2TASK.size(); 
        OBJ_X_TASK[ka+1][0] = OBJ2TASK[0]; 
        OBJ_X_TASK[ka+1][1] = OBJ2TASK[1]; 
        if (obj2tasklen == 6) {
            OBJ_X_TASK[ka+1][2] = OBJ2TASK[2]; 
            OBJ_X_TASK[ka+1][3] = OBJ2TASK[3]; 
            OBJ_X_TASK[ka+1][4] = OBJ2TASK[4]; 
            OBJ_X_TASK[ka+1][5] = OBJ2TASK[5]; 
            
            obj2wall_p0 = getPointByID(OBJ2TASK[0],pointlist);
            obj2wall_p1 = getPointByID(OBJ2TASK[1],pointlist);
            obj3wall_p0 = getPointByID(OBJ2TASK[2],pointlist);
            obj3wall_p1 = getPointByID(OBJ2TASK[3],pointlist);
            
            obj2frontwall_angle = atan2(obj2wall_p1.y-obj2wall_p0.y, obj2wall_p1.x-obj2wall_p0.x);
            obj3wall_angle = atan2(obj3wall_p1.y-obj3wall_p0.y, obj3wall_p1.x-obj3wall_p0.x);
            relative_angle = wrapToPi(obj3wall_angle-obj2frontwall_angle);
            
            if (OBJ2TASK[5].compare("right") == 0 && relative_angle < -SHARP_ANGLE_TRESHOLD) {
                // Sharp angle to the right, we need to take the next wall into account as well
                sharp_corner[ka+1] = true;
            } else if (OBJ2TASK[5].compare("left") == 0 && relative_angle > SHARP_ANGLE_TRESHOLD) {
                // Sharp angle to the left, we need to take the next wall into account as well
                sharp_corner[ka+1] = true;
            }
        }
    }
    
    // Last area
    OBJ_LAST_TASK = getWallPointsAwayFromB(OBJ_LAST,OBJ2);
    OBJ_X_TASK[ka_max-1][0] = OBJ_LAST_TASK[0];
    OBJ_X_TASK[ka_max-1][1] = OBJ_LAST_TASK[1];
    printstringvec(OBJ_LAST_TASK);

    double v_ax = 0, theta_dot = 0, v_des, phi, v_scale;
    bool ropod_is_in_44, ropod_is_in_45, ropod_is_in_46, ropod_is_in_47,
         ropod_is_in_48, ropod_is_in_49, ropod_is_in_50, ropod_is_in_51;

    bool consider_overtaking_area1, consider_overtaking_area3;
    bool update_assignment;
    int uprev;
    bool ropod_colliding_obs = true;
    bool ropod_colliding_wall = true;

    AreaQuad current_entry = generateEntry(assignment[0], assignment[1], ENTRY_LENGTH, arealist, pointlist);

    std::ofstream myfile;
    //myfile.open ("/simdata/ropod_" + get_date() +".txt");
    myfile.open ("/home/melvin/simdata/ropod_" + get_date() +".txt");
    myfile << "time" << "\t" << "tictoc" << "\t" << "state" << "\t" << "task counter" << "\t" << "tube width" << "\t" << 
            "phi des" << "\t" << "v ropod odom" << "\t"<< "v ropod cmd" << "\t" << "x ropod" << "\t" << "y ropod" << "\t" << 
            "theta ropod" << "\t" "x obs" << "\t" << "y obs" << "\t" << "theta obs" << "\t" << "obs width" << "\t" << 
            "obs depth" << "\t" << "obs vx" << "\t" << "obs vy" << "\t" << "des accel" <<"\n";

    std::clock_t start_loop;

    while(nroshndl.ok() && !ropod_reached_target)
    {
        
        prediction_feasibility_check_cycle_time += local_navigation_period;
        // Execute local navigation
        napoleon_driving.callLocalNavigationStateMachine();
        
        if (simple_goal_received)
        {
        
        start_loop = std::clock();

        ropod_colliding_obs = true;     // Initially set to true
        ropod_colliding_wall = true;    // Initially set to true
        // bool predict_intersection_time = true; // (not used in c++)
        k = 0;                          // Start with full speed (index [0])
        n = i*F_FSTR+1;
        i = i+1;

        // AMCL data (around 3Hz) - update to AMCL data if new AMCL pose received
        // otherwise make a guess
        if (this_amcl_x == prev_amcl_x && this_amcl_y == prev_amcl_y && this_amcl_theta == prev_amcl_theta) {
            // No AMCL update, so estimate initial values for prediction
            ropod_x = pred_x_ropod[1];
            ropod_y = pred_y_ropod[1];
            ropod_theta = pred_plan_theta[1];
        } else {
            // Set latest values from AMCL to previous AMCL values for next iteration
            // And take AMCL values as initial for the prediction
            prev_amcl_x = this_amcl_x;
            prev_amcl_y = this_amcl_y;
            prev_amcl_theta = this_amcl_theta;
            ropod_x = this_amcl_x;
            ropod_y = this_amcl_y;
            ropod_theta = this_amcl_theta;
        }

        //ROS_INFO("Ropod x: %f / Ropod y: %f / Theta: %f", ropod_x, ropod_y, ropod_theta);
        //ROS_INFO("xdot: %f / ydot: %f / thetadot %f", odom_xdot_ropod_global, odom_ydot_ropod_global, odom_thetadot_global);
        //ROS_INFO("ropodx: %f / ropody: %f / ropodtheta %f", ropod_x, ropod_y, ropod_theta);

        while ((ropod_colliding_obs || ropod_colliding_wall) && k < MAX_K) 
        {
            v_scale = V_SCALE_OPTIONS[k];
            k++;
            m = 0;
            j = 0;
            t_pred[m] = 0;
            t_pred_j[j] = 0;
            
            // Vars from callbacks:
            // ropod_x
            // ropod_y
            // ropod_theta
            // odom_xdot_ropod_global
            // odom_ydot_ropod_global
            // odom_thetadot_global
            // odom_phi_local
            // odom_vropod_global
            //ROS_INFO("xdot: %f \t ydot: %f", odom_xdot_ropod_global, odom_ydot_ropod_global);

            // Initialize prediction with latest sim values
            pred_phi[0] = odom_phi_local; // On ropod
            // pred_phi[0] = odom_phi_global-ropod_theta; // In sim (rmstart)
            pred_theta[0] = ropod_theta;
            // pred_v_ropod[0] = odom_vropod_global;
            pred_v_ropod[0] = control_v;
            if (abs(odom_vropod_global-control_v) > 0.5) {
                ROS_INFO("Difference between control and actual velocity > 0.5, correcting now.");
                pred_v_ropod[0] = odom_vropod_global;
            }
            pred_xdot[0] = pred_v_ropod[0]*cos(pred_phi[0])*cos(ropod_theta);   // xdot of rearaxle in global frame
            pred_ydot[0] = pred_v_ropod[0]*cos(pred_phi[0])*sin(ropod_theta);   // ydot of rearaxle in global frame
            pred_thetadot[0] = pred_v_ropod[0]*1/D_AX*sin(pred_phi[0]);
            pred_x_rearax[0] = ropod_x-D_AX*cos(ropod_theta);
            pred_y_rearax[0] = ropod_y-D_AX*sin(ropod_theta);
            pred_xy_ropod[0].x = ropod_x;
            pred_xy_ropod[0].y = ropod_y;
            prev_pred_phi_des = prev_sim_phi_des;
            pred_phi_des[0] = prev_pred_phi_des;
            pred_tube_width[0] = prev_sim_tube_width;
            pred_plan_theta[0] = ropod_theta;
            pred_v_ropod_plan[0] = pred_v_ropod[0];
            pred_state[0] = prev_sim_state;
            pred_task_counter[0] = prev_sim_task_counter; 
            pred_x_obs[0] = current_obstacle.pose.position.x;
            pred_y_obs[0] = current_obstacle.pose.position.y;
            // pred_phi[0] = sim_phi[n-1];
            // pred_theta[0] = sim_theta[n-1];
            // pred_v_ropod[0] = sim_v_ropod[n-1];
            // pred_xdot[0] = sim_xdot[n-1];
            // pred_ydot[0] = sim_ydot[n-1];
            // pred_thetadot[0] = sim_thetadot[n-1];
            // pred_x_rearax[0] = sim_x_rearax[n-1];
            // pred_y_rearax[0] = sim_y_rearax[n-1];
            // prev_pred_phi_des = sim_phi_des[i-1];
            // pred_phi_des[0] = prev_pred_phi_des;
            // pred_plan_theta[0] = sim_theta[n-1];
            // pred_v_ropod_plan[0] = sim_v_ropod[n-1];
            // pred_state[0] = sim_state[i-1];
            // pred_task_counter[0] = sim_task_counter[i-1];
            // pred_tube_width[0] = sim_tube_width[i-1];
            
            // Initialize areas
            u = pred_task_counter[0];
            if (u < ka_max-1) {
                area1 = assignment[u];
                task1 = OBJ_X_TASK[u];
                area2 = assignment[u+1];
                task2 = OBJ_X_TASK[u+1];
                area3 = assignment[u+2];
                task3 = OBJ_X_TASK[u+2];
                // AreaQuad current_sim_entry = generateEntry(assignment[u], assignment[u+1], ENTRY_LENGTH, arealist, pointlist);
            } 

            // Printing position
            // ROS_INFO("X: %f, Y: %f, Theta: %f", ropod_x, ropod_y, ropod_theta);
            
            // Prediction
            while (t_pred[m] < T_MIN_PRED) {
                j = j+1;
                
                // j and m counter are initialized at 0 instead of 1 in Matlab so we dont have to change their indices
                consider_overtaking_area1 = false;
                consider_overtaking_area3 = false;
                pred_tube_width[j] = pred_tube_width[j-1];  // Assume same as previous, might change later

                // Get old task counter
                u = pred_task_counter[j-1];
                if (j > 2) {
                    uprev = pred_task_counter[j-2];
                    if (u == uprev) {
                        update_assignment = false;
                    } else {
                        update_assignment = true;
                    }
                } else {
                    update_assignment = true;
                } 
                
                if (u < ka_max-1) {
                    if (update_assignment) {
                        area1 = assignment[u];
                        task1 = OBJ_X_TASK[u];
                        area2 = assignment[u+1];
                        task2 = OBJ_X_TASK[u+1];
                        area3 = assignment[u+2];
                        task3 = OBJ_X_TASK[u+2];
                        current_entry = generateEntry(assignment[u], assignment[u+1], ENTRY_LENGTH, arealist, pointlist);
                    }
                    
                    // Bad but working statement to check if task2 contains 6 nonempty strings.
                    // And if so, then it is considered as a corner assignment, otherwise as 
                    // go straight assignment.
                    if (!task2[5].empty()) { //(~cellfun(@isempty,task2),2) == 6
                        if (update_assignment) {
                            current_pivot = getPointByID(task2[4],pointlist);
                            cur_next_hallway_rear = getPointByID(task2[2],pointlist);
                            cur_next_hallway_front = getPointByID(task2[3],pointlist);
                        }
                        cur_pivot_local = coordGlobalToRopod(current_pivot, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
                        cur_next_hallway_rear_local = coordGlobalToRopod(cur_next_hallway_rear, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
                        cur_next_hallway_front_local = coordGlobalToRopod(cur_next_hallway_front, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
                        cur_next_hallway_angle = atan2(cur_next_hallway_front_local.y-cur_next_hallway_rear_local.y,cur_next_hallway_front_local.x-cur_next_hallway_rear_local.x);
                    } else if (!task2[1].empty()) { //sum(~cellfun(@isempty,task2),2) == 2 
                        if (update_assignment) {
                            current_inter_rear_wall = getPointByID(task2[0],pointlist);
                            current_inter_front_wall = getPointByID(task2[1],pointlist);
                        }
                        current_inter_rear_wall_local = coordGlobalToRopod(current_inter_rear_wall, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
                        current_inter_front_wall_local = coordGlobalToRopod(current_inter_front_wall, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
                    } 
                } else {
                    // Final area from assignment
                    area1 = assignment[u]; 
                    task1 = OBJ_X_TASK[u];
                    pred_state[j] = 1;  // Cruising in last HW
                }
                
                // Consider overtaking
                if (u < ka_max-1 && no_obs > 0) {
                    if (update_assignment) {
                        current_hallway = getAreaByID(area1, arealist);
                        next_hallway = getAreaByID(area3, arealist);
                    }
                    if (current_hallway.contains(obs_center_global)) {
                        consider_overtaking_area1 = true;
                        // This strategy only allows to overtake 1 obstacle
                    }
                    if (next_hallway.contains(obs_center_global)) {
                        consider_overtaking_area3 = true;
                        // This strategy only allows to overtake 1 obstacle
                    }
                }
                
                // Original implementation was checking for overlap between ropod shape and entry shape,
                // maybe change later if this implementation causes strange behavior
                pred_ropod_on_entry[j] = current_entry.contains(pred_xy_ropod[j-1]);

                update_state_points = true; // Initially true, might change to false when not necessary
                prevstate = j-1;

                // State of ropod 
                // 1 cruising
                // 2 entry before inter (turn)
                // 3 accelerate a bit on inter
                // 4 aligning rear axis with corner & slow down
                // 5 turning
                // 6 cruising right (deprecated)
                // 7 entry before inter (straight)
                // 8 straight on inter
                // 9 tight overtake (follow left wall)
                // 10 spacious overtake (shift right wall to left virtually)
                
                // Taking a turn on an intersection
                if (!task2[5].empty()) {
                    if (pred_ropod_on_entry[j] && pred_state[prevstate] == 1) {
                        // If cruising and the y position of the ropod exceeds the y
                        // position of the entry
                        pred_state[j] = 2;
                        
                    } else if (cur_pivot_local.x < SIZE_FRONT_ROPOD && pred_state[prevstate] == 2) {
                        // If in entry and the y position of the ropod exceeds the y
                        // position of the intersection
                        pred_state[j] = 3;
                        
                    } else if (cur_pivot_local.x <= -D_AX/2 && pred_state[prevstate] == 3) {
                        // If middle of vehicle is on y height of pivot
                        pred_state[j] = 4;
                        
                    } else if (cur_pivot_local.x <= -D_AX+START_STEERING_EARLY && pred_state[prevstate] == 4) {
                        // If rearaxle is aligned with the pivot minus sse
                        pred_state[j] = 5;
                        
                    } else if (-ROTATED_ENOUGH_TRES < cur_next_hallway_angle && cur_next_hallway_angle < ROTATED_ENOUGH_TRES && pred_state[prevstate] == 5) {
                        // If ropod has turned enough
                        
                        pred_state[j] = 1;
                        //disp([num2str(i), ': Here we can switch to the next task']);
                        u = u+2;
                        area1 = assignment[u];
                        task1 = OBJ_X_TASK[u];
                        if (u < ka_max-1) {
                            area2 = assignment[u+1];
                            task2 = OBJ_X_TASK[u+1];
                            area3 = assignment[u+2];
                            task3 = OBJ_X_TASK[u+2];
                            current_entry = generateEntry(assignment[u], assignment[u+1], ENTRY_LENGTH, arealist, pointlist);
                            //current_inter = getPoints(getAreaByID(area2,arealist));
                        }
                    } else {
                        
                        pred_state[j] = pred_state[prevstate];
                        update_state_points = false;
                    }
                // Going straight on an intersection
                } else if (!task2[1].empty()) {
                    if (pred_ropod_on_entry[j] == 1 && pred_state[prevstate] == 1) {
                        // If cruising and the y position of the ropod exceeds the y
                        // position of the entry
                        
                        pred_state[j] = 7;
                    } else if (current_inter_rear_wall_local.x < SIZE_FRONT_ROPOD+START_STEERING_EARLY && pred_state[prevstate] == 7) {
                        // If in entry and the y position of the ropod exceeds the y
                        // position of the intersection
                        
                        pred_state[j] = 8;
                    } else if (current_inter_front_wall_local.x < -D_AX/2 && pred_state[prevstate] == 8) {
                        pred_state[j] = 1;
                        u = u+2;
                        area1 = assignment[u];
                        task1 = OBJ_X_TASK[u];
                        if (u < ka_max-1) {
                            area2 = assignment[u+1];
                            task2 = OBJ_X_TASK[u+1];
                            area3 = assignment[u+2];
                            task3 = OBJ_X_TASK[u+2];
                            current_entry = generateEntry(assignment[u], assignment[u+1], ENTRY_LENGTH, arealist, pointlist);
                            //current_inter = getPoints(getAreaByID(area2,arealist));
                        }
                        
                    } else {
                        pred_state[j] = pred_state[prevstate];
                        update_state_points = false;
                    }
                }
                if (pred_state[prevstate] == 9 || pred_state[prevstate] == 10) {
                    if (no_obs > 0) {
                        current_obs_in_ropod_frame_pos = coordGlobalToRopod(obs_center_global, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
                        //disp(['Obs is ',num2str(obs_in_ropod_frame_pos.x), ' m in front of ropod']); 
                        if (current_obs_in_ropod_frame_pos.x+current_obstacle.depth/2+D_AX+SIZE_REAR < 0) {
                            pred_state[j] = 1;
                            update_state_points = true;
                            pred_tube_width[j] = TUBE_WIDTH_C;
                        } else {
                            pred_state[j] = pred_state[prevstate];
                            update_state_points = false;
                        }
                    } else {
                        // Ropod doesn't see obs anymore, return cruising.
                        // When obt disappears depends on the setting how long it takes before obs disappears after not seeing it (entity-timeout)
                        // It can be found in /catkin_workspace/src/applications/ropod_navigation_test/config/model-example-ropod-navigation-ED.yaml
                        pred_state[j] = 1;
                        update_state_points = true;
                        pred_tube_width[j] = TUBE_WIDTH_C;
                    }
                }
                
                // Overtaking
                if ((pred_state[prevstate] == 1 && consider_overtaking_area1) || consider_overtaking_area3 && (pred_state[prevstate] == 5 || pred_state[prevstate] == 8)) {
                    obs_in_ropod_frame_pos = coordGlobalToRopod(obs_center_global, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
                    v_obs_sq = current_obstacle.vel.x*current_obstacle.vel.x+current_obstacle.vel.y*current_obstacle.vel.y;
                    if ((v_obs_sq < V_OBS_OVERTAKE_MAX*V_OBS_OVERTAKE_MAX) && obs_in_ropod_frame_pos.x > 0) {
                        //disp("Slow obstacle, check if there is space to overtake");
                        if (consider_overtaking_area1) {
                            rw_p_rear = getPointByID(task1[0],pointlist);
                            rw_p_front = getPointByID(task1[1],pointlist);
                            cur_obj = getAreaByID(area1,arealist);
                            areaIDs = cur_obj.getPointIDs();
                            ind = 0;
                            qmax = areaIDs.size();
                            for (int q = 0; q < qmax; ++q) {
                                if (areaIDs[q].compare(rw_p_rear.id) == 0) {
                                    ind = q; 
                                }
                            } 
                            rotate(areaIDs.begin(), areaIDs.begin() + ind, areaIDs.end());
                            lw_p_rear = getPointByID(areaIDs[3],pointlist);
                            lw_p_front = getPointByID(areaIDs[2],pointlist);
                        } else if (consider_overtaking_area3) {
                            rw_p_rear = getPointByID(task3[0],pointlist);
                            rw_p_front = getPointByID(task3[1],pointlist);
                            cur_obj = getAreaByID(area3,arealist);
                            areaIDs = cur_obj.getPointIDs();
                            ind = 0;
                            qmax = areaIDs.size();
                            for (int q = 0; q < qmax; ++q) {
                                if (areaIDs[q].compare(rw_p_rear.id) == 0) {
                                    ind = q; 
                                }
                            } 
                            rotate(areaIDs.begin(), areaIDs.begin() + ind, areaIDs.end());
                            lw_p_rear = getPointByID(areaIDs[3],pointlist);
                            lw_p_front = getPointByID(areaIDs[2],pointlist);
                        }

                        space_left = distToSegment(obs_center_global,lw_p_rear,lw_p_front); 
                        space_right = distToSegment(obs_center_global,rw_p_rear,rw_p_front); 
                        if (current_obstacle.width > current_obstacle.depth) {
                            space_left = space_left-current_obstacle.width/2;
                            space_right = space_right-current_obstacle.width/2;
                        } else {
                            space_left = space_left-current_obstacle.depth/2;
                            space_right = space_right-current_obstacle.depth/2;
                        }
                        
                        if (space_right > TUBE_WIDTH_C) {
                            if (j == 1) {
                                ROS_INFO("No overtake necessary, passing on right should be possible");
                            }
                        } else if (space_right > 2*(SIZE_SIDE+ENV_TCTW_SIZE)+OBS_AVOID_MARGIN) {
                            // Same state, but change tube width so ropod will
                            // fit through space right
                            pred_tube_width[j] = space_right;
                            if (j == 1) {
                                ROS_INFO("No overtake necessary, but tube size scaled down");
                            }
                        } else if (space_left > 2*(SIZE_SIDE+ENV_TRNS_SIZE)+ENV_TCTW_SIZE) {
                            if (j == 1) {
                                ROS_INFO("Can overtake on left side, there should be enough space there");
                            }
                            // Start overtake
                            if (space_left < TUBE_WIDTH_C) {
                                if (obs_in_ropod_frame_pos.x < MIN_DIST_TO_OVERTAKE && abs(obs_in_ropod_frame_pos.y) < MIN_DIST_TO_OVERTAKE) {
                                    ROS_INFO("Tight overtake, state 9");
                                    pred_state[j] = 9;
                                    //current_to_overtake_obs = to_overtake_obs;
                                    if (consider_overtaking_area3) {
                                        u = u+2;
                                        update_assignment = true;
                                    }
                                    pred_tube_width[j] = 2*(SIZE_SIDE+ENV_TCTW_SIZE+ENV_TRNS_SIZE);
                                }
                            } else {
                                if (obs_in_ropod_frame_pos.x < MIN_DIST_TO_OVERTAKE && abs(obs_in_ropod_frame_pos.y) < MIN_DIST_TO_OVERTAKE) {
                                    ROS_INFO("Spacious overtake, state 10");
                                    pred_state[j] = 10;
                                    //current_to_overtake_obs = to_overtake_obs;
                                    if (consider_overtaking_area3) {
                                        u = u+2;
                                        update_assignment = true;
                                    }
                                    if (current_obstacle.width > current_obstacle.depth) {
                                        shift_wall = space_right+current_obstacle.width+OBS_AVOID_MARGIN;
                                    } else {
                                        shift_wall = space_right+current_obstacle.depth+OBS_AVOID_MARGIN;
                                    }
                                    pred_tube_width[j] = TUBE_WIDTH_C;
                                }
                            }
                        } else {
                            if (j == 1) {
                                ROS_INFO("No overtake possible, stuck behind this obstacle");
                            }
                        }
                    }
                } else if (!consider_overtaking_area1 && !consider_overtaking_area3) {
                    pred_tube_width[j] = TUBE_WIDTH_C;
                }
                
                // Exception for first plan
                if (j == 1) {
                    update_state_points = true;
                }
                // Exception for when current and previous state are turning
                // And the state before that is aligning, as the state jump
                // happens immediately if that is required and then the state
                // points need to be updated.
                if (j > 1) {
                    if (pred_state[j] == 5 && pred_state[j-1] == 5 && (pred_state[j-2] == 3 || pred_state[j-2] == 4)) {
                        update_state_points = true;
                    }
                }

                pred_task_counter[j] = u;

                // Monitor if we don't bump into front wall when aligning
                // rearaxle to pivot. If so, switch to turn state (5)
                if (pred_state[j] == 3 || pred_state[j] == 4) {
                    if (update_state_points) {
                        wall_front_p0 = getPointByID(task2[0],pointlist);
                        wall_front_p1 = getPointByID(task2[1],pointlist);
                        //global_wall_front_p0 = [wall_front_p0.x, wall_front_p0.y];
                        //global_wall_front_p1 = [wall_front_p1.x, wall_front_p1.y];
                    }
                    
                    local_wall_front_p0 = coordGlobalToRopod(wall_front_p0, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
                    local_wall_front_p1 = coordGlobalToRopod(wall_front_p1, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
                    if (do_lines_intersect(local_front_ropod_dilated_p0, local_front_ropod_dilated_p1, local_wall_front_p0, local_wall_front_p1)) {
                        pred_state[j] = 5;
                        update_state_points = true;
                        //disp("Switched early while aligning pivot because too close to front wall");
                    }
                }
                //////////////////////////////////////////////////////////////////////////////////////////
                // Perform the appropriate action according to finite state machine
                if (pred_state[j] == 1 || pred_state[j] == 6 || pred_state[j] == 8 || pred_state[j] == 9 || pred_state[j] == 10) {
                    // disp([num2str[j],' - Ropod is now cruising']);
                    if (pred_state[j] == 1) {   // Cruising up
                        if (update_state_points) {
                            point_rear = getPointByID(task1[0],pointlist);
                            point_front = getPointByID(task1[1],pointlist);
                            //ROS_INFO("Point rear: %s, Point front: %s", point_rear.id, point_front.id);
                            glob_wallpoint_front.x = point_front.x;
                            glob_wallpoint_front.y = point_front.y;
                            glob_wallpoint_rear.x = point_rear.x;
                            glob_wallpoint_rear.y = point_rear.y;
                        }
                        v_des = V_CRUISING;
                    } else if (pred_state[j] == 6) { // Cruising next hallway (will only do this 1 sample and then switch)
                        if (update_state_points) {
                            point_rear = getPointByID(task3[0],pointlist);
                            point_front = getPointByID(task3[1],pointlist);
                            glob_wallpoint_front.x = point_front.x;
                            glob_wallpoint_front.y = point_front.y;
                            glob_wallpoint_rear.x = point_rear.x;
                            glob_wallpoint_rear.y = point_rear.y;
                        }
                        v_des = V_CRUISING;
                    } else if (pred_state[j] == 8) { // Straight on inter
                        if (update_state_points) {
                            point_rear = getPointByID(task2[0],pointlist);
                            point_front = getPointByID(task2[1],pointlist);
                            glob_wallpoint_front.x = point_front.x;
                            glob_wallpoint_front.y = point_front.y;
                            glob_wallpoint_rear.x = point_rear.x;
                            glob_wallpoint_rear.y = point_rear.y;
                        }
                        v_des = V_INTER_ACC;
                    } else if (pred_state[j] == 9) { // Tight overtake
                        rw_p_rear = getPointByID(task1[0],pointlist);
                        cur_obj = getAreaByID(area1,arealist);
                        areaIDs = cur_obj.getPointIDs();
                        ind = 0;
                        qmax = areaIDs.size();
                        for (int q = 0; q < qmax; ++q) {
                            if (areaIDs[q].compare(rw_p_rear.id) == 0) {
                                ind = q; 
                            }
                        } 
                        rotate(areaIDs.begin(), areaIDs.begin() + ind, areaIDs.end());
                        lw_p_rear = getPointByID(areaIDs[3],pointlist);
                        lw_p_front = getPointByID(areaIDs[2],pointlist);
                        wallang = atan2(lw_p_front.y-lw_p_rear.y,lw_p_front.x-lw_p_rear.x);
                        //glob_wallpoint_front = [lw_front.x, lw_front.y]+pred_tube_width[j]*[cos(wallang-M_PI/2), sin(wallang-M_PI/2)];
                        glob_wallpoint_front.x = lw_p_front.x+pred_tube_width[j]*cos(wallang-M_PI/2);
                        glob_wallpoint_front.y = lw_p_front.y+pred_tube_width[j]*sin(wallang-M_PI/2);
                        //glob_wallpoint_rear = [lw_rear.x, lw_rear.y]+pred_tube_width[j]*[cos(wallang-M_PI/2), sin(wallang-M_PI/2)];
                        glob_wallpoint_rear.x = lw_p_rear.x+pred_tube_width[j]*cos(wallang-M_PI/2);
                        glob_wallpoint_rear.y = lw_p_rear.y+pred_tube_width[j]*sin(wallang-M_PI/2);
                        v_des = V_OVERTAKE;
                    } else if (pred_state[j] == 10) { // Spacious overtake
                        point_rear = getPointByID(task1[0],pointlist);
                        point_front = getPointByID(task1[1],pointlist);
                        wallang = atan2(point_front.y-point_rear.y,point_front.x-point_rear.x);
                        //glob_wallpoint_front = [point_front.x, point_front.y]+shift_wall*[cos(wallang+M_PI/2), sin(wallang+M_PI/2)];
                        //glob_wallpoint_rear = [point_rear.x, point_rear.y]+shift_wall*[cos(wallang+M_PI/2), sin(wallang+M_PI/2)];
                        glob_wallpoint_front.x = point_front.x+shift_wall*cos(wallang+M_PI/2);
                        glob_wallpoint_front.y = point_front.y+shift_wall*sin(wallang+M_PI/2);
                        glob_wallpoint_rear.x = point_rear.x+shift_wall*cos(wallang+M_PI/2);
                        glob_wallpoint_rear.y = point_rear.y+shift_wall*sin(wallang+M_PI/2);
                        v_des = V_OVERTAKE;
                    }
                    local_wallpoint_front = coordGlobalToRopod(glob_wallpoint_front, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
                    local_wallpoint_rear = coordGlobalToRopod(glob_wallpoint_rear, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
                    pred_phi_des[j] = getSteering(local_wallpoint_front, local_wallpoint_rear, pred_tube_width[j]);
                } else if (pred_state[j] == 2 || pred_state[j] == 7) {
                    // disp([num2str[j],' - Ropod is now in entry']);
                    if (update_state_points) {
                        point_rear = getPointByID(task1[0],pointlist);
                        point_front = getPointByID(task1[1],pointlist);
                        glob_wallpoint_front.x = point_front.x;
                        glob_wallpoint_front.y = point_front.y;
                        glob_wallpoint_rear.x = point_rear.x;
                        glob_wallpoint_rear.y = point_rear.y;
                    }
                    local_wallpoint_front = coordGlobalToRopod(glob_wallpoint_front, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
                    local_wallpoint_rear = coordGlobalToRopod(glob_wallpoint_rear, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
                    pred_phi_des[j] = getSteering(local_wallpoint_front, local_wallpoint_rear, pred_tube_width[j]);
                    v_des = V_ENTRY;
                } else if (pred_state[j] == 3) {
                    // disp([num2str[j],' - Ropod is at inter, driving forward']);
                    if (update_state_points) {
                        point_rear = getPointByID(task1[0],pointlist);
                        point_front = getPointByID(task1[1],pointlist);
                        glob_wallpoint_front.x = point_front.x;
                        glob_wallpoint_front.y = point_front.y;
                        glob_wallpoint_rear.x = point_rear.x;
                        glob_wallpoint_rear.y = point_rear.y;
                    }
                    
                    local_wallpoint_front = coordGlobalToRopod(glob_wallpoint_front, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
                    local_wallpoint_rear = coordGlobalToRopod(glob_wallpoint_rear, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
                    pred_phi_des[j] = getSteering(local_wallpoint_front, local_wallpoint_rear, pred_tube_width[j]);
                    v_des = V_INTER_ACC;
                    
                    // Monitor if we don't bump into front wall
                    if (update_state_points) {
                        wall_front_p0 = getPointByID(task2[0],pointlist);
                        wall_front_p1 = getPointByID(task2[1],pointlist);
                        
                        global_wall_front_p0.x = wall_front_p0.x;
                        global_wall_front_p0.y = wall_front_p0.y;
                        global_wall_front_p1.x = wall_front_p1.x; 
                        global_wall_front_p1.y = wall_front_p1.y;
                    }

                    local_wall_front_p0 = coordGlobalToRopod(global_wall_front_p0, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
                    local_wall_front_p1 = coordGlobalToRopod(global_wall_front_p1, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
                    if (do_lines_intersect(local_front_ropod_dilated_p0, local_front_ropod_dilated_p1, local_wall_front_p0, local_wall_front_p1)) {
                        pred_state[j] = 5;
                        update_state_points = true;
                        //disp("Switched early while aligning pivot because too close to front wall");
                    }
                } else if (pred_state[j] == 4) {
                    // disp([num2str[j],' - Ropod is at inter, driving forward']);
                    if (update_state_points) {
                        point_rear = getPointByID(task1[0],pointlist);
                        point_front = getPointByID(task1[1],pointlist);
                        glob_wallpoint_front.x = point_front.x;
                        glob_wallpoint_front.y = point_front.y;
                        glob_wallpoint_rear.x = point_rear.x;
                        glob_wallpoint_rear.y = point_rear.y;
                    }

                    local_wallpoint_front = coordGlobalToRopod(glob_wallpoint_front, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
                    local_wallpoint_rear = coordGlobalToRopod(glob_wallpoint_rear, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
                    pred_phi_des[j] = getSteering(local_wallpoint_front, local_wallpoint_rear, pred_tube_width[j]);
                    v_des = V_INTER_DEC;
                    
                    // Monitor if we don't bump into front wall
                    if (update_state_points) {
                        wall_front_p0 = getPointByID(task2[0],pointlist);
                        wall_front_p1 = getPointByID(task2[1],pointlist);
                        global_wall_front_p0.x = wall_front_p0.x;
                        global_wall_front_p0.y = wall_front_p0.y;
                        global_wall_front_p1.x = wall_front_p1.x; 
                        global_wall_front_p1.y = wall_front_p1.y;
                    }

                    local_wall_front_p0 = coordGlobalToRopod(global_wall_front_p0, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
                    local_wall_front_p1 = coordGlobalToRopod(global_wall_front_p1, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
                    if (do_lines_intersect(local_front_ropod_dilated_p0, local_front_ropod_dilated_p1, local_wall_front_p0, local_wall_front_p1)) {
                        pred_state[j] = 5;
                        update_state_points = true;
                        //disp("Switched early while aligning pivot because too close to front wall");
                    }
                } else if (pred_state[j] == 5) {
                    // disp([num2str[j],' - Ropod is at inter, taking the turn']);
                    if (update_state_points) {
                        point_rear = getPointByID(task2[0],pointlist);
                        point_front = getPointByID(task2[1],pointlist);
                        point_pivot = getPointByID(task2[4],pointlist);
                        glob_wallpoint_front.x = point_front.x;
                        glob_wallpoint_front.y = point_front.y;
                        glob_wallpoint_rear.x = point_rear.x;
                        glob_wallpoint_rear.y = point_rear.y;
                    }
                    
                    dir_cw = true;
                    if (task2[5].compare("left") == 0) {
                    //if (strcmp(task2{6},'left')) {
                        dir_cw = false; // direction 0 = CCW, 1 = CW
                    }

                    local_pivot = coordGlobalToRopod(point_pivot, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
                    if (!sharp_corner[u+1]) {
                        local_wallpoint_front = coordGlobalToRopod(glob_wallpoint_front, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
                        local_wallpoint_rear = coordGlobalToRopod(glob_wallpoint_rear, pred_xy_ropod[j-1], pred_plan_theta[j-1]);
                        //pred_phi_des[j] = getSteeringTurn(ropod_length, size_side, feeler_size_steering, d_ax, dir_cw, local_pivot, local_wallpoint_front, local_wallpoint_rear,  follow_wall_distance, env_tctw_size, env_trns_size_cornering, env_carrot_size);
                        pred_phi_des[j] = getSteeringTurn(local_pivot, dir_cw, local_wallpoint_front, local_wallpoint_rear);
                    } else {
                        //pred_phi_des[j] = getSteeringTurnSharp(pred_x_ropod(j-1), pred_y_ropod(j-1), pred_plan_theta[j-1], size_front_ropod, size_side, feeler_size_steering, d_ax, dir_cw, task2, pointlist, follow_wall_distance, env_tctw_size, env_trns_size_cornering, env_carrot_size);
                        pred_phi_des[j] = getSteeringTurnSharp(pred_xy_ropod[j-1], pred_plan_theta[j-1], dir_cw, task2, pointlist);
                    }
                    
                    v_des = V_INTER_TURNING;
                }

                // Wrap to [-pi,pi] domain
                pred_phi_des[j] = wrapToPi(pred_phi_des[j]);
                
                // Saturate steering rate
                if (abs(pred_phi_des[j]-prev_pred_phi_des) > DELTA_DOT_LIMIT/(double)F_PLAN) {
                    //disp("Delta steering too large, steering saturated");
                    pred_phi_des[j] = prev_pred_phi_des + sgn(pred_phi_des[j]-prev_pred_phi_des)*DELTA_DOT_LIMIT/(double)F_PLAN;
                    // Decrease vel leads to better corners
                    // The velocity is already decreased in the state machine, but this is just a harsh backup
                    // pred_steer_rate_saturation[j] = 1;
                    if (v_des > V_STEERSATURATION) {
                        v_des = V_STEERSATURATION;
                    }
                    //disp(['In saturation - j: ', num2str(j) ,', Phides: ', num2str(pred_phi_des(j)), ' // Prev phides: ' , num2str(prev_pred_phi_des), ', v_des = ', num2str(v_des)]);
                }
                
                prev_pred_phi_des = pred_phi_des[j];

                // Applying the v_scale (scales down vel if collision is detected in previous prediction and therefore it failed)
                // And then we calculate the acceleration for the predictions.
                v_des_scaled[j] = v_des*v_scale;
                vel_dif = abs(pred_v_ropod_plan[j-1]-v_des_scaled[j]);   // Difference between actual and desired velocity
                if (vel_dif < max_delta_v) {
                    v_new = v_des_scaled[j];
                    pred_accel[j] = (v_des_scaled[j]-pred_v_ropod_plan[j-1])/delta_t;
                } else { // Adapt velocity with maximum acceleration
                    v_new = pred_v_ropod_plan[j-1]+sgn(v_des_scaled[j]-pred_v_ropod_plan[j-1])*max_delta_v;
                    pred_accel[j] = sgn(v_des_scaled[j]-pred_v_ropod_plan[j-1])*A_MAX;
                }
                //ROS_INFO("pred_accel: %f", pred_accel[j]);
                
                m_prev = m;
                t_pred_prev = t_pred[m];
                t_pred_j[j] = t_pred_prev+F_FSTR*TS;

                // Predict movement with current plan  
                for (int q = 1; q <= F_FSTR; ++q) { // q = [1, 2, ..., F_FSTR]
                    m = m_prev+q;           // Current iteration
                    t_pred[m] = t_pred_prev+q*TS;
                    
                    pred_phi[m] = (1-lpf)*pred_phi[m-1]+lpf*pred_phi_des[j];
                    pred_theta[m] = wrapToPi(pred_theta[m-1]+pred_thetadot[m-1]*TS);
                    pred_v_ropod[m] = pred_v_ropod[m-1]+pred_accel[j]*TS;

                    pred_xdot[m] = pred_v_ropod[m]*cos(pred_phi[m])*cos(pred_theta[m]);
                    pred_ydot[m] = pred_v_ropod[m]*cos(pred_phi[m])*sin(pred_theta[m]);
                    pred_thetadot[m] = pred_v_ropod[m]*1/D_AX*sin(pred_phi[m]);

                    pred_x_rearax[m] = pred_x_rearax[m-1]+pred_xdot[m]*TS;
                    pred_y_rearax[m] = pred_y_rearax[m-1]+pred_ydot[m]*TS;
                }

                if (no_obs > 0) {
                    pred_x_obs[j] = pred_x_obs[j-1]+current_obstacle.vel.x*TS*F_FSTR;
                    pred_y_obs[j] = pred_y_obs[j-1]+current_obstacle.vel.y*TS*F_FSTR;
                }

                // Dilated vehicle
                pred_ropod_dil_rb.x = pred_x_rearax[m] + (SIZE_REAR+(OBS_AVOID_MARGIN*v_scale))*cos(pred_theta[m]+M_PI) + (SIZE_SIDE+(OBS_AVOID_MARGIN*v_scale))*cos(pred_theta[m]-M_PI/2);
                pred_ropod_dil_rb.y = pred_y_rearax[m] + (SIZE_REAR+(OBS_AVOID_MARGIN*v_scale))*sin(pred_theta[m]+M_PI) + (SIZE_SIDE+(OBS_AVOID_MARGIN*v_scale))*sin(pred_theta[m]-M_PI/2);
                pred_ropod_dil_lb.x = pred_x_rearax[m] + (SIZE_REAR+(OBS_AVOID_MARGIN*v_scale))*cos(pred_theta[m]+M_PI) + (SIZE_SIDE+(OBS_AVOID_MARGIN*v_scale))*cos(pred_theta[m]+M_PI/2);
                pred_ropod_dil_lb.y = pred_y_rearax[m] + (SIZE_REAR+(OBS_AVOID_MARGIN*v_scale))*sin(pred_theta[m]+M_PI) + (SIZE_SIDE+(OBS_AVOID_MARGIN*v_scale))*sin(pred_theta[m]+M_PI/2);
                pred_ropod_dil_lt.x = pred_x_rearax[m] + (SIZE_FRONT_RAX+(OBS_AVOID_MARGIN*v_scale))*cos(pred_theta[m]) + (SIZE_SIDE+(OBS_AVOID_MARGIN*v_scale))*cos(pred_theta[m]+M_PI/2); 
                pred_ropod_dil_lt.y = pred_y_rearax[m] + (SIZE_FRONT_RAX+(OBS_AVOID_MARGIN*v_scale))*sin(pred_theta[m]) + (SIZE_SIDE+(OBS_AVOID_MARGIN*v_scale))*sin(pred_theta[m]+M_PI/2);
                pred_ropod_dil_rt.x = pred_x_rearax[m] + (SIZE_FRONT_RAX+(OBS_AVOID_MARGIN*v_scale))*cos(pred_theta[m]) + (SIZE_SIDE+(OBS_AVOID_MARGIN*v_scale))*cos(pred_theta[m]-M_PI/2);
                pred_ropod_dil_rt.y = pred_y_rearax[m] + (SIZE_FRONT_RAX+(OBS_AVOID_MARGIN*v_scale))*sin(pred_theta[m]) + (SIZE_SIDE+(OBS_AVOID_MARGIN*v_scale))*sin(pred_theta[m]-M_PI/2);

                // Obstacle detection (crappy implementation in C++)
                if (t_pred[m] < T_PRED_OBS_COLLISION && no_obs > 0) {
                    obslt.x = pred_x_obs[j]+current_obstacle.width/2*cos(obs_theta)-current_obstacle.depth/2*sin(obs_theta);
                    obslt.y = pred_y_obs[j]+current_obstacle.width/2*sin(obs_theta)+current_obstacle.depth/2*cos(obs_theta);
                    obsrt.x = pred_x_obs[j]+current_obstacle.width/2*cos(obs_theta)+current_obstacle.depth/2*sin(obs_theta);
                    obsrt.y = pred_y_obs[j]+current_obstacle.width/2*sin(obs_theta)-current_obstacle.depth/2*cos(obs_theta);
                    obslb.x = pred_x_obs[j]-current_obstacle.width/2*cos(obs_theta)-current_obstacle.depth/2*sin(obs_theta);
                    obslb.y = pred_y_obs[j]-current_obstacle.width/2*sin(obs_theta)+current_obstacle.depth/2*cos(obs_theta);
                    obsrb.x = pred_x_obs[j]-current_obstacle.width/2*cos(obs_theta)+current_obstacle.depth/2*sin(obs_theta);
                    obsrb.y = pred_y_obs[j]-current_obstacle.width/2*sin(obs_theta)-current_obstacle.depth/2*cos(obs_theta);
                    pred_ropod_colliding_obs[j] = do_shapes_overlap(pred_ropod_dil_rb, pred_ropod_dil_lb, pred_ropod_dil_lt, pred_ropod_dil_rt, obsrt, obslt, obslb, obsrb);
                } else {
                    pred_ropod_colliding_obs[j] = false;
                }

                vis_points.id = 1;
                vis_points.color.r = 0.0;
                vis_points.color.g = 0.0;
                vis_points.color.b = 0.0;
                vis_points.scale.x = 0.1;
                vis_points.scale.y = 0.1;
                vis_p.x = obsrt.x; vis_p.y = obsrt.y; vis_points.points.push_back(vis_p);
                vis_p.x = obslt.x; vis_p.y = obslt.y; vis_points.points.push_back(vis_p);
                vis_p.x = obslb.x; vis_p.y = obslb.y; vis_points.points.push_back(vis_p);
                vis_p.x = obsrb.x; vis_p.y = obsrb.y; vis_points.points.push_back(vis_p);
                ropodmarker_pub.publish(vis_points);

                // Predict intersection times (left out for now)
                
                // Predict intersection with walls
                // if ((u < ka_max-1) && update_assignment) {
                //     walls = getWalls(assignment[u],assignment[u+1],assignment[u+2],arealist);
                // }
                // if (t_pred[m] < T_PRED_WALL_COLLISION) {
                //     pred_ropod_wall_collision(j) = 0;
                //     for wallidx = 1:size(walls,1)
                //         wall = walls(wallidx,:);
                //         wp0 = getPointByID(wall{1},pointlist);
                //         wp1 = getPointByID(wall{2},pointlist);
                //         ropod = [pred_ropod_rb(j,:); pred_ropod_lb(j,:); pred_ropod_lt(j,:); pred_ropod_rt(j,:)];
                //         if does_line_intersect_shape(wp0, wp1, ropod)
                //             pred_ropod_wall_collision(j) = 1;
                //             // disp([num2str(t_pred(m)),': Oh noes I collided wiv a wall']);
                //         end
                //     end
                // }
                
                // Update positions used to make the prediction plan with
                pred_x_ropod[j] = pred_x_rearax[m]+D_AX*cos(pred_theta[m]);
                pred_y_ropod[j] = pred_y_rearax[m]+D_AX*sin(pred_theta[m]);
                pred_xy_ropod[j].x = pred_x_ropod[j];
                pred_xy_ropod[j].y = pred_y_ropod[j];
                pred_v_ropod_plan[j] = pred_v_ropod[m];
            
                if (j == 1) {
                    //ROS_INFO("v[0] = %f, a[1] = %f, ts = %f, v[1] = %f", pred_v_ropod[0], pred_accel[j], TS, pred_v_ropod_plan[j]);
                    ROS_INFO("v[0] = %f, a[1] = %f, deltav = %f, v[1] = %f", pred_v_ropod[0], pred_accel[j], pred_accel[j]*local_navigation_period, pred_v_ropod[0]+pred_accel[1]*local_navigation_period);
                }
                pred_plan_theta[j] = pred_theta[m];
                //ROS_INFO("j: %d / State: %d / Time: %f / Phi: %f / V_des: %f", j, pred_state[j], t_pred_j[j], pred_phi_des[j], pred_v_ropod[j]);

                //--
            } // endwhile prediction
            
            // Line 892 - 912 (alter obstacle and wall collision booleans)
            // Set collision to false
            ropod_colliding_obs = false;
            // If collision predicted at any step, set to true
            if (no_obs > 0) {
                for (int qobs = 0; qobs < size_p; ++qobs) {
                    if (pred_ropod_colliding_obs[qobs] == true) {
                        //ROS_INFO("Collision predicted");
                        ropod_colliding_obs = true;
                    }
                }
            }

            ropod_colliding_wall = false;

        }   // end while finding v_scale

        // Update after a prediction is made where no collision is caused
        // The prediction is ran until t_min_pred, however, the ropod will run a
        // new prediction the next step, so only the first part of the
        // prediction is used.
        
        prev_sim_state = pred_state[1];
        prev_sim_task_counter = pred_task_counter[1];
        prev_sim_phi_des = pred_phi_des[1];
        prev_sim_tube_width = pred_tube_width[1];
        program_duration = ( std::clock() - start_loop ) / (double) CLOCKS_PER_SEC;
        real_time_est = real_time_est+local_navigation_period;

        control_v = pred_v_ropod[0]+pred_accel[1]*local_navigation_period;
        // Compute v_ax and theta_dot from v_des and phi
        //ROS_INFO("Phi: %f / V_ax: %f / Theta_dot: %f", pred_phi_des[1], v_ax, theta_dot);
        ROS_INFO("state: %d, tube width: %f", pred_state[1], pred_tube_width[1]);
        myfile << real_time_est << "\t" << program_duration << "\t" << pred_state[0] << "\t" << pred_task_counter[0] << "\t" << pred_tube_width[0] << "\t" << 
            pred_phi_des[0] << "\t" << pred_v_ropod[0] << "\t" << control_v << "\t"  <<  pred_xy_ropod[0].x << "\t" <<  pred_xy_ropod[0].y << "\t" << 
            pred_plan_theta[0] << "\t" << pred_x_obs[0] << "\t" << pred_y_obs[0] << "\t" << obs_theta << "\t" << current_obstacle.width << "\t" << 
            current_obstacle.depth << "\t" << current_obstacle.vel.x << "\t" << current_obstacle.vel.y << "\t" << pred_accel[1] << "\t" <<"\n";
        //ROS_INFO("K: %d", k);
        //ROS_INFO("V desired: %f", pred_v_ropod_plan[1]);
        //ROS_INFO("Predphi[1]: %f / [2]: %f / [3]: %f / [4]: %f", pred_phi_des[1], pred_phi_des[2], pred_phi_des[3], pred_phi_des[4]);
        
        // if (pred_v_ropod_plan[1] > 0) {
        //     v_ax = cos(pred_phi_des[1])*pred_v_ropod_plan[1];
        //     theta_dot = pred_v_ropod_plan[1]/D_AX*sin(pred_phi_des[1]);
        //     napoleon_driving.publishCustomVelocity(v_ax, theta_dot);
        // } else {
        //     napoleon_driving.publishZeroVelocity();
        // }
        if (control_v > 0) {
            v_ax = cos(pred_phi_des[1])*control_v;
            theta_dot = control_v/D_AX*sin(pred_phi_des[1]);
            napoleon_driving.publishCustomVelocity(v_ax, theta_dot);
        } else {
            napoleon_driving.publishZeroVelocity();
        }

        OBJ_LAST_center = OBJ_LAST.center();
        dist_to_middle_final = sqrt((OBJ_LAST_center.x-pred_x_rearax[1])*(OBJ_LAST_center.x-pred_x_rearax[1])+(OBJ_LAST_center.y-pred_y_rearax[1])*(OBJ_LAST_center.y-pred_y_rearax[1]));

        if (prev_sim_task_counter == ka_max-1 && dist_to_middle_final < REACHEDTARGETTRESHOLD) {
            ropod_reached_target = true;
            ROS_INFO("Ropod has reached its target, yay!");
        }

        // Publish ropod points to rostopic
        vis_points.id = 2;
        vis_points.color.r = 0.0;
        vis_points.color.g = 0.0;
        vis_points.color.b = 1.0;
        vis_points.scale.x = 0.2;
        vis_points.scale.y = 0.2;
        vis_p.x = ropod_x;
        vis_p.y = ropod_y;
        vis_points.points.push_back(vis_p);
        vis_points.id = 3;
        vis_points.color.r = 1.0;
        vis_points.color.g = 0.0;
        vis_points.color.b = 0.0;
        x_rearax = ropod_x - D_AX*cos(ropod_theta); // X position of center of rear axle [m]
        y_rearax = ropod_y - D_AX*sin(ropod_theta); // Y position of center of rear axle [m]
        vis_rt.x = x_rearax+(D_AX+SIZE_FRONT_ROPOD)*cos(ropod_theta)+SIZE_SIDE*cos(ropod_theta-0.5*M_PI);
        vis_rt.y = y_rearax+(D_AX+SIZE_FRONT_ROPOD)*sin(ropod_theta)+SIZE_SIDE*sin(ropod_theta-0.5*M_PI);
        vis_lt.x = x_rearax+(D_AX+SIZE_FRONT_ROPOD)*cos(ropod_theta)+SIZE_SIDE*cos(ropod_theta+0.5*M_PI);
        vis_lt.y = y_rearax+(D_AX+SIZE_FRONT_ROPOD)*sin(ropod_theta)+SIZE_SIDE*sin(ropod_theta+0.5*M_PI);
        vis_fr.x = FEELER_SIZE_STEERING*cos(ropod_theta+prev_sim_phi_des);
        vis_fr.y = FEELER_SIZE_STEERING*sin(ropod_theta+prev_sim_phi_des);
        vis_fl.x = FEELER_SIZE_STEERING*cos(ropod_theta+prev_sim_phi_des);
        vis_fl.y = FEELER_SIZE_STEERING*sin(ropod_theta+prev_sim_phi_des);
        vis_fr = vis_fr.add(vis_rt); vis_fl = vis_fl.add(vis_lt);
        vis_p.x = vis_lt.x; vis_p.y = vis_lt.y; vis_points.points.push_back(vis_p);
        vis_p.x = vis_rt.x; vis_p.y = vis_rt.y; vis_points.points.push_back(vis_p);
        vis_p.x = vis_fr.x; vis_p.y = vis_fr.y; vis_points.points.push_back(vis_p);
        vis_p.x = vis_fl.x; vis_p.y = vis_fl.y; vis_points.points.push_back(vis_p);
        vis_points.id = 4;
        vis_points.color.r = 0.0;
        vis_points.color.g = 1.0;
        vis_points.color.b = 0.0;
        vis_p.x = glob_wallpoint_rear.x; vis_p.y = glob_wallpoint_rear.y; vis_points.points.push_back(vis_p);
        vis_p.x = glob_wallpoint_front.x; vis_p.y = glob_wallpoint_front.y; vis_points.points.push_back(vis_p);
        ropodmarker_pub.publish(vis_points);
        vis_points.points.clear();
        // End publish ropod points
        
        }   // end if received goal
        ros::spinOnce();
        rate.sleep();
        if(rate.cycleTime() > ros::Duration(local_navigation_period) ){
            ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", local_navigation_rate, rate.cycleTime().toSec());
        }
        
    }
    // Will only perform this when ropod has reached target
    napoleon_driving.publishZeroVelocity();
    myfile.close();

    return 0;
}

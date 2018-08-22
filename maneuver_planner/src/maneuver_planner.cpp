/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, TU/e
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Authors: Cesar Lopez
*********************************************************************/
#include <maneuver_planner/maneuver_planner.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(maneuver_planner::ManeuverPlanner, nav_core::BaseGlobalPlanner)

namespace maneuver_planner {

ManeuverPlanner::ManeuverPlanner()
    : costmap_ros_(NULL), initialized_(false)
{}

ManeuverPlanner::ManeuverPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), initialized_(false)
{
    initialize(name, costmap_ros);
}

void ManeuverPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    if(!initialized_)
    {
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();

        ros::NodeHandle private_nh("~/" + name);
        private_nh.param("step_size", step_size_, costmap_->getResolution());
        private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
        private_nh.param("turning_radius", turning_radius_, 0.8);
        private_nh.param("use_last_goal_as_start", last_goal_as_start_, false);
        valid_last_goal_ = false;
        world_model_ = new base_local_planner::CostmapModel(*costmap_);


        // For now only rectangular robot shape is supported. Initiallize transformation matrices
        std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
        //if we have no footprint... do nothing
        if(footprint.size() != 4)
        {
            ROS_ERROR("Footprint must have hour points");
            return;
        }

        // The heading of the robot is towards the x axis.
        bool topRightCorner_b = false;
        bool topLeftCorner_b = false;
        bool bottomRightCorner_b = false;
        bool bottomLeftCorner_b = false;



        for (int i_fp = 0; i_fp<footprint.size(); i_fp++)
        {
// 	ROS_INFO("footprint %.3f, %.3f", footprint[i_fp].x, footprint[i_fp].y);
            if( footprint[i_fp].x > 0 && footprint[i_fp].y > 0)
            {
                topLeftCorner_b = true;
                topLeftCorner_ << footprint[i_fp].x, footprint[i_fp].y;
            } 
            else if( footprint[i_fp].x > 0 && footprint[i_fp].y < 0)
            {
                topRightCorner_b = true;
                topRightCorner_ << footprint[i_fp].x, footprint[i_fp].y;
            } 
            else if( footprint[i_fp].x < 0 && footprint[i_fp].y < 0)
            {
                bottomRightCorner_b = true;
                bottomRightCorner_ << footprint[i_fp].x, footprint[i_fp].y;
            } 
            else if( footprint[i_fp].x < 0 && footprint[i_fp].y > 0)
            {
                bottomLeftCorner_b = true;
                bottomLeftCorner_ << footprint[i_fp].x, footprint[i_fp].y;
            }
        }

        if( ! (topRightCorner_b & topLeftCorner_b & bottomRightCorner_b & bottomLeftCorner_b) )
        {
            ROS_ERROR("Footprint must have four corners and center of rotation inside the footprint");
            return;
        }
        else
        {
	    right_side_ref_point_ << 0.1, bottomRightCorner_[1];
	    left_side_ref_point_  << 0.1, bottomLeftCorner_[1] ;
        }
        
       
        radius_search_ = parameter_generator::ParameterGenerator(0.1, 2.0, 2.0, 0.1, 10);
        midway_scale_lr_search_ = parameter_generator::ParameterGenerator(0.05, 0.95, 1.0, 0.1, 10);
        midway_side_ovt_search_ = parameter_generator::ParameterGenerator(0.2, 2.0, 2.0, 0.1, 20);
        initialized_ = true;
    }
    else
        ROS_WARN("This planner has already been initialized... doing nothing");
}

void ManeuverPlanner::rotate2D(const tf::Stamped<tf::Pose> &pose_tf_in, const double theta, tf::Stamped<tf::Pose> &pose_tf_out)
{
    tf::Vector3 origin;
    tf::Quaternion quat;
    quat.setRPY(0.0,0.0,theta);
    double ty,tp,tr;
    origin = pose_tf_in.getOrigin().rotate(tf::Vector3(0,0,1),theta);
    pose_tf_in.getBasis().getEulerYPR(ty,tp,tr);
    quat.setRPY(0.0,0.0,theta+ty);
    pose_tf_out.setData(tf::Transform(quat,origin));
    pose_tf_out.stamp_ = pose_tf_in.stamp_;

    return;


}

void ManeuverPlanner::translate2D(const tf::Stamped<tf::Pose>& pose_tf_in, const tf::Vector3& vector3_translation, tf::Stamped<tf::Pose> &pose_tf_out)
{

    tf::Vector3 origin;
    tf::Quaternion quat;

    origin = pose_tf_in.getOrigin() + vector3_translation;
    quat = pose_tf_in.getRotation();
    pose_tf_out.setData(tf::Transform(quat,origin));
    pose_tf_out.stamp_ = pose_tf_in.stamp_;

}

int  ManeuverPlanner::determineManeuverType(const tf::Stamped<tf::Pose>& pose_target,  double &signed_max_turning_radius, double& x_intersection)
{
    
    int maneuver_type = ManeuverPlanner::MANEUVER_NONE;

    double x_target   = pose_target.getOrigin().getX();
    double y_target   = pose_target.getOrigin().getY();
    double yaw_target,useless_pitcht,useless_rollt;
    pose_target.getBasis().getEulerYPR(yaw_target,useless_pitcht,useless_rollt);
    double theta_target = std::atan2(y_target,x_target);
    

    // the frame reference is at the starting position of the reference point, thus at the origin
    // By definition the intersection lies in the y-axis at (x_intersection,0.0)
    x_intersection = x_target - y_target/std::tan(yaw_target); // derived from geometry relations
     

    double dist_target_to_intersection = std::sqrt( (x_target-x_intersection)*(x_target-x_intersection) + y_target*y_target );
    double max_perpendicular_to_radius_line = std::min(std::abs(x_intersection),std::abs(dist_target_to_intersection));
    
    double abs_max_turning_radius = std::abs( max_perpendicular_to_radius_line * std::tan( (M_PI - yaw_target)/2.0 ) );
    signed_max_turning_radius = 0.0;
    
    if( x_target >= 0.0) // Target in front of the robot        
    { 
        
        if ( x_target >= ManeuverPlanner::MIN_X_DIST_OVERTAKE   &&   std::abs(theta_target)<= ManeuverPlanner::MIN_THETA_OVERTAKE  )
        {
            maneuver_type = ManeuverPlanner::MANEUVER_STRAIGHT_OTHERWISE_OVERTAKE;
        }
        else if( y_target >= 0.0 ) // Target in front left of the robot
        { 
            if ( yaw_target > theta_target && yaw_target < M_PI ) // Execute single left turn maneuver
            { 
//                 ROS_INFO(" Target in front left of the robot: Execute single left turn maneuver");
                signed_max_turning_radius = abs_max_turning_radius;
                maneuver_type = ManeuverPlanner::MANEUVER_LEFT;
            }
            else if( yaw_target < theta_target && yaw_target > -M_PI/2.0 ) // Execute left and then turn right
            { 
//                 ROS_INFO(" Target in front left of the robot: Execute left and then turn right");
                maneuver_type = ManeuverPlanner::MANEUVER_LEFT_RIGHT;
            }
            else // This could be executed with a left maneuver > M_PI turn, or multiple maneuvers. For now use carrot planner
            {
//                 ROS_INFO(" Target in front left of the robot: Unconventional orientation. Execute carrot planner");
                maneuver_type = ManeuverPlanner::MANEUVER_NONE;
            }
        }
        else // Target in front right of the robot
        {
            if ( yaw_target < theta_target && yaw_target > - M_PI ) // Execute single right turn maneuver
            { 
//                 ROS_INFO(" Target in front right of the robot: Execute single right turn maneuver");
                signed_max_turning_radius = -abs_max_turning_radius;;
                maneuver_type = ManeuverPlanner::MANEUVER_RIGHT;
            }
            else if( yaw_target > theta_target && yaw_target < M_PI/2.0 ) // Execute right and then turn left
            {
//                 ROS_INFO(" Target in front right of the robot:  Execute right and then turn left");
                maneuver_type = ManeuverPlanner::MANEUVER_RIGHT_LEFT;
            }
            else // This could be executed with a right maneuver > M_PI turn, or multiple maneuvers. For now use carrot planner
            {
//                 ROS_INFO(" Target in front left of the robot: Unconventional orientation. Execute carrot planner");
                maneuver_type = ManeuverPlanner::MANEUVER_NONE;
            }            
        }
    }
    else // Target behind robot. One option is to rotate +-pi/2 and repeat search. For now just execute carrot planner, which results in driving backwards 
    { 
        maneuver_type = ManeuverPlanner::MANEUVER_NONE;
    }
    
    return maneuver_type;
    
}


bool ManeuverPlanner::computeSingleManeuverParameters(const tf::Stamped<tf::Pose>& pose_target, const double& signed_turning_radius, const double& x_intersection, double &dist_before_steering, double &dist_after_steering)
{
    dist_before_steering   = -1.0;
    dist_after_steering    = -1.0;
    double x_target   = pose_target.getOrigin().getX();
    double y_target   = pose_target.getOrigin().getY();
    double yaw_target,useless_pitcht,useless_rollt;
    pose_target.getBasis().getEulerYPR(yaw_target,useless_pitcht,useless_rollt);

    // here no checks are done on the validity of the radius and intersection. That is done in determineManeuverType function, which must be called before
    
    double dist_target_to_intersection = std::sqrt( (x_target-x_intersection)*(x_target-x_intersection) + y_target*y_target );    

    double dist_x_intersection_steering = signed_turning_radius/tan((M_PI-yaw_target)/2.0);
    dist_before_steering =  x_intersection - dist_x_intersection_steering;
    dist_after_steering = dist_target_to_intersection - dist_x_intersection_steering;

    if ( dist_before_steering<0.0 |  dist_after_steering<0.0 | dist_before_steering > x_intersection )
    {
        // ROS_INFO("No single turn possible with desired radius, dist_bs<0 || dist_as<0 || dist_bs>xi. Change turning radius or try multiple turns maneouver");
        return false;
    }


    return true;
}

//we need to take the footprint of the robot into account when we calculate cost to obstacles
double ManeuverPlanner::footprintCost(double x_i, double y_i, double theta_i)
{
    if(!initialized_)
    {
        ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
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


void ManeuverPlanner::generateDublinTrajectory(const double& dist_before_steering_refp, const double& dist_after_steering_refp, 
                               const double& signed_turning_radius_refp, const double& theta_refp_goal, std::vector<tf::Pose> &local_plan_refp)
{
    
    // Compute trajectory as points. Only the center of teh robot has a pose and orientation
    motion_refpoint_localtraj_ << 0.0, 0.0; // starts at origin by definition
    prev_motion_refpoint_localtraj_  << 0.0, 0.0;
    tf::Quaternion temp_quat;
    tf::Vector3 temp_vector3;
    tf::Pose ref_traj_point_tf_refstart_coord; // current  Refpoint in the the start position of the reference point coordinate frame    
    bool traj_ready = false;    
    double theta_refp_traj = 0.0 ; // This is only to control evolution of the curvature.
    double theta_refp_traj_gridsz = step_size_/signed_turning_radius_refp; // Gridsize of the trajectory angle
    double dist_bef_steer = 0.0, dist_af_steer = 0.0;    
    double theta_refp_goal_internal = 0.0;
    
    if(theta_refp_traj_gridsz < 0)
    {
        theta_refp_goal_internal = - std::abs(theta_refp_goal);
    }
    else
    {
        theta_refp_goal_internal = std::abs(theta_refp_goal);
    }
    
    local_plan_refp.clear();
    
    while (!traj_ready)
    {
        // Generate next reference refpoint in trajectory
        if( dist_bef_steer <  dist_before_steering_refp)
        {   // Move straight before steering
            theta_refp_traj = 0;
            motion_refpoint_localtraj_[0] = prev_motion_refpoint_localtraj_[0]+ step_size_;
            motion_refpoint_localtraj_[1] = prev_motion_refpoint_localtraj_[1];
            dist_bef_steer += step_size_;
        } 
        else if(std::abs(theta_refp_goal_internal-theta_refp_traj) > std::abs(theta_refp_traj_gridsz/2.0))
        {   // Turn with circle. This can be as well a clothoid!
            theta_refp_traj +=theta_refp_traj_gridsz;
            motion_refpoint_localtraj_[0] = dist_before_steering_refp +signed_turning_radius_refp*std::sin(theta_refp_traj);
            motion_refpoint_localtraj_[1] = signed_turning_radius_refp*(1.0 - std::cos(theta_refp_traj));
        } 
        else if( dist_af_steer <  dist_after_steering_refp)
        {   // Move straight after steering
            theta_refp_traj = theta_refp_goal_internal;
            motion_refpoint_localtraj_[0] = prev_motion_refpoint_localtraj_[0]+ step_size_*std::cos(theta_refp_traj);
            motion_refpoint_localtraj_[1] = prev_motion_refpoint_localtraj_[1]+ step_size_*std::sin(theta_refp_traj);
            dist_af_steer += step_size_;
        }
        else
        {
            traj_ready = true;
            break;
        }
        prev_motion_refpoint_localtraj_ = motion_refpoint_localtraj_;
        
        // Add point to trajectory
        temp_quat.setRPY(0.0,0.0,theta_refp_traj);
        temp_vector3 = tf::Vector3(motion_refpoint_localtraj_[0], motion_refpoint_localtraj_[1], theta_refp_traj);        
        ref_traj_point_tf_refstart_coord.setOrigin(temp_vector3);
        ref_traj_point_tf_refstart_coord.setRotation(temp_quat);
        local_plan_refp.push_back(ref_traj_point_tf_refstart_coord);        
    }  
    
}



bool ManeuverPlanner::checkFootprintTransformLocalToGlobal(const tf::Stamped<tf::Pose>& start_tf, 
                                const tf::Stamped<tf::Pose>& goal_tf, const tf::Stamped<tf::Pose>& refpoint_tf_robot_coord, 
                                const std::vector<tf::Pose>& local_plan_refp, std::vector<geometry_msgs::PoseStamped>& plan, double & dist_without_obstacles)
{
    std::vector<tf::Pose>::const_iterator local_plan_point_it;        
    double start_yaw, goal_yaw, temp_yaw, temp_pitch, temp_roll, theta_refp_traj;
    
    tf::Quaternion temp_quat;
    tf::Vector3 temp_vector3;
    start_tf.getBasis().getEulerYPR(start_yaw, temp_pitch, temp_roll);    

    double footprint_cost;
    Eigen::Matrix2d jacobian_motrefPoint;
    Eigen::Matrix2d invjacobian_motrefPoint;

    tf::Pose ref_traj_point_tf_refstart_coord; // current  Refpoint in the the start position of the reference point coordinate frame
    tf::Stamped<tf::Pose> center_traj_point_tf_refstart_coord; // current  Center point in the the start position of the reference point coordinate frame
    tf::Stamped<tf::Pose> center_traj_point_tf; // current  Center point in the global coordinate frame

    // Initialize trajectory of reference point. By definition at the origin


    // Initialize trajectory of center point. By definition at -refpoint_tf_robot_coord
    center_traj_point_tf_refstart_coord.frame_id_ = "/refpoint_start_pos";
    center_traj_point_tf_refstart_coord.stamp_ = goal_tf.stamp_;
    temp_quat.setRPY(0.0,0.0,0.0);
    temp_vector3 = -refpoint_tf_robot_coord.getOrigin();
    center_traj_point_tf_refstart_coord.setData(tf::Transform(temp_quat,temp_vector3));
    // This pose as a vector because it will be more direct to make operations on it. x y theta
    center_pose_loctrajframe_ << temp_vector3.getX(), temp_vector3.getY(), 0.0;


    // Compute global coordinates center trajectory point
    center_traj_point_tf.frame_id_ = goal_tf.frame_id_;
    center_traj_point_tf.stamp_ = goal_tf.stamp_;

    translate2D(center_traj_point_tf_refstart_coord,refpoint_tf_robot_coord.getOrigin(),center_traj_point_tf);
    rotate2D(center_traj_point_tf,start_yaw,center_traj_point_tf);
    translate2D(center_traj_point_tf,start_tf.getOrigin(),center_traj_point_tf);
    
    // Jacobian to compute virtual velocities and therefore positions.      
    jacobian_motrefPoint  << 1.0 , -refpoint_tf_robot_coord.getOrigin().getY(),
                                0.0 ,  refpoint_tf_robot_coord.getOrigin().getX();                  

    if ( refpoint_tf_robot_coord.getOrigin().getX() != 0.0 )
    {   // Check refpoint is not at the center
        invjacobian_motrefPoint = jacobian_motrefPoint.inverse();                   
    }
    else
    {
        invjacobian_motrefPoint << 0.0, 0.0,
                                    0.0, 0.0;
    }    
    

    bool traj_free = true;
    
    
    prev_motion_refpoint_localtraj_[0] = 0.0;
    prev_motion_refpoint_localtraj_[1] = 0.0;
        
    double total_ahead_distance = 0.0;
    
    for (local_plan_point_it = local_plan_refp.begin(); local_plan_point_it != local_plan_refp.end(); local_plan_point_it++)
    {            
        motion_refpoint_localtraj_[0] = local_plan_point_it->getOrigin().getX();        
        motion_refpoint_localtraj_[1] = local_plan_point_it->getOrigin().getY(); 
        local_plan_point_it->getBasis().getEulerYPR(theta_refp_traj, temp_pitch, temp_roll);
        
        
        // Now compute robot center of rotation trajectory
        // Compute virtual velocity of reference point. Virtual time of 1.0 sec
        Eigen::Vector2d motion_refpoint_virvel_loctrajframe;
        Eigen::Vector2d motion_refpoint_deltapos_loctrajframe;        
        motion_refpoint_deltapos_loctrajframe = (motion_refpoint_localtraj_ - prev_motion_refpoint_localtraj_);
        total_ahead_distance = total_ahead_distance + hypot(motion_refpoint_deltapos_loctrajframe[0],motion_refpoint_deltapos_loctrajframe[1]);
        motion_refpoint_virvel_loctrajframe = motion_refpoint_deltapos_loctrajframe/1.0;
        prev_motion_refpoint_localtraj_ = motion_refpoint_localtraj_;

        if ( refpoint_tf_robot_coord.getOrigin().getX() != 0.0 )
        {   // Check refpoint is not at the center
            //Compute center of rotation pose from inverse Jacobian
            Eigen::Matrix2d RotM;
            RotM    << std::cos(center_pose_loctrajframe_[2]),  std::sin(center_pose_loctrajframe_[2]),
                        -std::sin(center_pose_loctrajframe_[2]),  std::cos(center_pose_loctrajframe_[2]);
            // Compute refpoint velocity local at the robot by rotating velocity vector
            Eigen::Vector2d motion_refpoint_virvel_robotframe;
            motion_refpoint_virvel_robotframe = RotM*motion_refpoint_virvel_loctrajframe;
            // Compute the corresponding robot velocity using inverse of the jacobian
            Eigen::Vector2d center_vel_robotframe;          
            motion_refpoint_virvel_robotframe[0]=motion_refpoint_virvel_robotframe[0];
            motion_refpoint_virvel_robotframe[1]=motion_refpoint_virvel_robotframe[1];
            center_vel_robotframe = invjacobian_motrefPoint*motion_refpoint_virvel_robotframe; // [dx dtheta]
            // Compute evolution of the robot by integrating virtual velocity (dt virtual is 1.0 sec)
            center_pose_loctrajframe_[2] += 1.0*center_vel_robotframe[1];
            center_pose_loctrajframe_[0] += 1.0*center_vel_robotframe[0]*std::cos(center_pose_loctrajframe_[2]);
            center_pose_loctrajframe_[1] += 1.0*center_vel_robotframe[0]*std::sin(center_pose_loctrajframe_[2]);
        }
        else
        {
            // Compute center of rotation directly from ref_point positions
            center_pose_loctrajframe_[0] = motion_refpoint_localtraj_[0];
            center_pose_loctrajframe_[1] = motion_refpoint_localtraj_[1];
            center_pose_loctrajframe_[2] = theta_refp_traj;

        }

        // Center pose in local trajectory frame, in tf::PoseStamped
        temp_quat.setRPY(0.0,0.0,center_pose_loctrajframe_[2]);
        temp_vector3 = tf::Vector3(center_pose_loctrajframe_[0],center_pose_loctrajframe_[1],0.0);
        center_traj_point_tf_refstart_coord.setData(tf::Transform(temp_quat,temp_vector3));

        // Center pose in global frame, in tf::PoseStamped
        translate2D(center_traj_point_tf_refstart_coord,refpoint_tf_robot_coord.getOrigin(),center_traj_point_tf);
        rotate2D(center_traj_point_tf,start_yaw,center_traj_point_tf);
        translate2D(center_traj_point_tf,start_tf.getOrigin(),center_traj_point_tf);
        
        center_traj_point_tf.getBasis().getEulerYPR(temp_yaw, temp_pitch, temp_roll);
        temp_vector3 = center_traj_point_tf.getOrigin();
        footprint_cost = footprintCost(temp_vector3.getX(), temp_vector3.getY(), temp_yaw);
        if(footprint_cost < 0)
        {
            traj_free  = false;
            dist_without_obstacles = total_ahead_distance;
            break;
        }
        else
        {   // Add current point to overall trajectory
            geometry_msgs::PoseStamped traj_point;
            poseStampedTFToMsg(center_traj_point_tf,traj_point);
            tf::Quaternion goal_quat = tf::createQuaternionFromYaw(temp_yaw);

            traj_point.pose.position.x = temp_vector3.getX();
            traj_point.pose.position.y = temp_vector3.getY();

            traj_point.pose.orientation.x = goal_quat.x();
            traj_point.pose.orientation.y = goal_quat.y();
            traj_point.pose.orientation.z = goal_quat.z();
            traj_point.pose.orientation.w = goal_quat.w();

            plan.push_back(traj_point);

        }
        
    }
    return traj_free;
        
}

bool ManeuverPlanner::searchTrajectoryCompoundLeftRightManeuver(const tf::Stamped<tf::Pose>& start_tf, const tf::Stamped<tf::Pose>& goal_tf, 
                                                        const tf::Stamped<tf::Pose>& refpoint_tf_robot_coord, std::vector<geometry_msgs::PoseStamped>& plan, double & dist_without_obstacles)
{
    tf::Stamped<tf::Pose> temp_goal_tf;
    tf::Stamped<tf::Pose> temp_start_tf;
    
    temp_goal_tf.frame_id_ = goal_tf.frame_id_;
    temp_goal_tf.stamp_ = goal_tf.stamp_;

            
    
    tf::Stamped<tf::Pose> refpoint_start_tf; // Start Refpoint in the global coordinate frame
    tf::Stamped<tf::Pose> refpoint_goal_tf; // Goal Refpoint in the global coordinate frame        
    tf::Stamped<tf::Pose> refpoint_goal_tf_refstart_coord; // Goal Refpoint in the the start position of the reference point coordinate frame
    tf::Stamped<tf::Pose> refpoint_goal_tf_refmidway_coord; // Goal Refpoint in the the midway position of the reference point coordinate frame. Used in second maneuver
    double dist_before_steering_refp, dist_after_steering_refp, signed_turning_radius_refp;
    double unsigned_radius;
    double dist_before_steering_center, dist_after_steering_center, signed_turning_radius_center;
    int curve_type;  
    bool maneuver_traj_succesful = false;
    double start_yaw, goal_yaw, temp_yaw, temp_pitch, temp_roll;
    tf::Quaternion temp_quat;
    tf::Vector3 temp_vector3;
    start_tf.getBasis().getEulerYPR(start_yaw, temp_pitch, temp_roll);             
    goal_tf.getBasis().getEulerYPR(goal_yaw, temp_pitch, temp_roll);             
    
    tf::Stamped<tf::Pose> goal_tf_start_coord; // Goal in the start vector coordinates
    goal_tf_start_coord.frame_id_ = "/center_rotation_start_pos";

    translate2D(goal_tf,-start_tf.getOrigin(),goal_tf_start_coord);
    rotate2D(goal_tf_start_coord,-start_yaw,goal_tf_start_coord);
    
    
    // Compute reference start and goal on global coordinates
    refpoint_start_tf.frame_id_ = goal_tf.frame_id_;
    refpoint_start_tf.stamp_ = goal_tf.stamp_;
    rotate2D(refpoint_tf_robot_coord,start_yaw,refpoint_start_tf);
    translate2D(refpoint_start_tf,start_tf.getOrigin(),refpoint_start_tf);

    refpoint_goal_tf.frame_id_ = goal_tf.frame_id_;
    refpoint_goal_tf.stamp_ = goal_tf.stamp_;
    rotate2D(refpoint_tf_robot_coord,goal_yaw,refpoint_goal_tf);
    translate2D(refpoint_goal_tf,goal_tf.getOrigin(),refpoint_goal_tf);

    // Then Compute reference start and goal on reference start coordinates
    refpoint_goal_tf_refstart_coord.frame_id_ = "/refpoint_start_pos";
    refpoint_goal_tf_refstart_coord.stamp_ = goal_tf.stamp_;

    translate2D(refpoint_goal_tf,-refpoint_start_tf.getOrigin(),refpoint_goal_tf_refstart_coord);
    refpoint_start_tf.getBasis().getEulerYPR(temp_yaw, temp_pitch, temp_roll);
    rotate2D(refpoint_goal_tf_refstart_coord,-temp_yaw,refpoint_goal_tf_refstart_coord);    
    
    tf::Stamped<tf::Pose> refpoint_midway_goal_tf_refstart_coord; // Midway Goal Refpoint in the the start position of the reference point coordinate frame
    tf::Stamped<tf::Pose> center_midway_goal_tf; // center of rotation of "ideal" midway goal in global coordinate frame
    // the minimiun angle for theta midway is the angle of the 
    double refp_theta_min;
    double refp_theta_max;
    double theta_refp_goal;
    

    std::vector<geometry_msgs::PoseStamped> plan_first_m;
    std::vector<geometry_msgs::PoseStamped> plan_second_m;
    
    
    double midway_scale_lr;
    
    std::vector< tf::Stamped<tf::Pose> > refpoint_tf_robot_coord_vec;    
    refpoint_tf_robot_coord_vec.push_back(refpoint_tf_robot_coord);
    
    midway_scale_lr_search_.resetMidSearch(midway_scale_lr_search_.lin_search_min_, midway_scale_lr_search_.lin_search_max_);
    
    while( midway_scale_lr_search_.midSearch(midway_scale_lr) & !maneuver_traj_succesful){         
               
    
        temp_vector3 = tf::Vector3( midway_scale_lr*refpoint_goal_tf_refstart_coord.getOrigin().getX(), refpoint_goal_tf_refstart_coord.getOrigin().getY()/2.0, 0.0);    
        refpoint_midway_goal_tf_refstart_coord.setOrigin(temp_vector3);
        refp_theta_min = std::atan2(refpoint_midway_goal_tf_refstart_coord.getOrigin().getY(), refpoint_midway_goal_tf_refstart_coord.getOrigin().getX());    
        if( refp_theta_min > 0)   // Left right
            refp_theta_max = M_PI/2.0;
        else // Right left
            refp_theta_max = -M_PI/2.0;
        
        theta_refp_goal = angles::normalize_angle(refp_theta_min + 0.5*(refp_theta_max-refp_theta_min)  ); // Final angle of mid goal // For now set a fix value. We can do also a search on this parameter_generator 
        temp_quat.setRPY(0.0, 0.0, theta_refp_goal); 
        refpoint_midway_goal_tf_refstart_coord.setRotation(temp_quat);
        
//         ROS_INFO("refpoint_midway_goal_loc: %f, %f, %f", refpoint_midway_goal_tf_refstart_coord.getOrigin().getX(), refpoint_midway_goal_tf_refstart_coord.getOrigin().getY(), theta_refp_goal);
        
         // Convert back to global coordinates
        temp_quat.setRPY(0.0, 0.0, 0.0); 
        center_midway_goal_tf.setOrigin(-refpoint_tf_robot_coord.getOrigin());
        center_midway_goal_tf.setRotation(temp_quat);
        rotate2D(center_midway_goal_tf,theta_refp_goal,center_midway_goal_tf); 
        translate2D(center_midway_goal_tf,refpoint_midway_goal_tf_refstart_coord.getOrigin(),center_midway_goal_tf);        
        
        translate2D(center_midway_goal_tf,refpoint_tf_robot_coord.getOrigin(),center_midway_goal_tf);        
        rotate2D(center_midway_goal_tf,start_yaw,center_midway_goal_tf); 
        translate2D(center_midway_goal_tf,start_tf.getOrigin(),center_midway_goal_tf);

       
                
        temp_start_tf = start_tf;
        temp_goal_tf.setOrigin(center_midway_goal_tf.getOrigin()); 
        temp_goal_tf.setRotation(center_midway_goal_tf.getRotation()); 
        
        dist_without_obstacles = 0.0;
        double dist_without_obstacles_single_maneuver;  
        plan_first_m.clear();
        maneuver_traj_succesful = searchTrajectorySingleManeuver(temp_start_tf, temp_goal_tf, refpoint_tf_robot_coord_vec, plan_first_m, dist_without_obstacles_single_maneuver);
        dist_without_obstacles = dist_without_obstacles_single_maneuver;
        if (!maneuver_traj_succesful)
            continue;
        
        maneuver_traj_succesful = false;
//         ROS_INFO("First local trajectory created. Second maneuver reached");   
        std::vector<geometry_msgs::PoseStamped>::iterator plan_iterator;
        plan_iterator = plan_first_m.end()-1;
        // Take last pose of previus plan
        poseStampedMsgToTF(*plan_iterator,temp_start_tf); 
        
     
        ROS_INFO("temp_start_tf: %f, %f", temp_start_tf.getOrigin().getX(), temp_start_tf.getOrigin().getY() );
        
        temp_goal_tf.setOrigin(goal_tf.getOrigin());
        temp_goal_tf.setRotation(goal_tf.getRotation());
        
        plan_second_m.clear();
        maneuver_traj_succesful = searchTrajectorySingleManeuver(temp_start_tf, temp_goal_tf, refpoint_tf_robot_coord_vec, plan_second_m, dist_without_obstacles_single_maneuver);
        dist_without_obstacles = dist_without_obstacles + dist_without_obstacles_single_maneuver;
        
//         ROS_INFO("Second trajectory maneuver_traj_succesful: %d", maneuver_traj_succesful);

        for (plan_iterator = plan_first_m.begin(); plan_iterator != plan_first_m.end(); plan_iterator++)
        {
            plan.push_back(*plan_iterator);
        }
        for (plan_iterator = plan_second_m.begin(); plan_iterator != plan_second_m.end(); plan_iterator++)
        {
            plan.push_back(*plan_iterator);
        }            
     
        
  }
  
//   ROS_INFO("Total trajectory created. Check trajectory, free: %d", (int) maneuver_traj_succesful);
  return maneuver_traj_succesful;
}

bool ManeuverPlanner::searchTrajectoryOvertakeManeuver(const tf::Stamped<tf::Pose>& start_tf, const tf::Stamped<tf::Pose>& goal_tf, 
                                                        const tf::Stamped<tf::Pose>& refpoint_tf_robot_coord, std::vector<geometry_msgs::PoseStamped>& plan, double & dist_without_obstacles)
{
    tf::Stamped<tf::Pose> temp_goal_tf;
    tf::Stamped<tf::Pose> temp_start_tf;

    temp_goal_tf.frame_id_ = goal_tf.frame_id_;
    temp_goal_tf.stamp_ = goal_tf.stamp_;

            

    tf::Stamped<tf::Pose> refpoint_start_tf; // Start Refpoint in the global coordinate frame
    tf::Stamped<tf::Pose> refpoint_goal_tf; // Goal Refpoint in the global coordinate frame        
    tf::Stamped<tf::Pose> refpoint_goal_tf_refstart_coord; // Goal Refpoint in the the start position of the reference point coordinate frame
    
    double dist_before_steering_refp, dist_after_steering_refp, signed_turning_radius_refp;
    double unsigned_radius;
    double dist_before_steering_center, dist_after_steering_center, signed_turning_radius_center;
    int curve_type;  
    bool maneuver_traj_succesful = false;
    double start_yaw, goal_yaw, temp_yaw, temp_pitch, temp_roll;
    tf::Quaternion temp_quat;
    tf::Vector3 temp_vector3;
    start_tf.getBasis().getEulerYPR(start_yaw, temp_pitch, temp_roll);     
    
    goal_tf.getBasis().getEulerYPR(goal_yaw, temp_pitch, temp_roll);     
    
    start_yaw = goal_yaw; // With this we just compute the intermediate point with same angle as the goal_yaw. This works better in corridors.


    // Compute reference start and reference goal on global coordinates
    refpoint_start_tf.frame_id_ = goal_tf.frame_id_;
    refpoint_start_tf.stamp_ = goal_tf.stamp_;
    rotate2D(refpoint_tf_robot_coord,start_yaw,refpoint_start_tf);
    translate2D(refpoint_start_tf,start_tf.getOrigin(),refpoint_start_tf);

    refpoint_goal_tf.frame_id_ = goal_tf.frame_id_;
    refpoint_goal_tf.stamp_ = goal_tf.stamp_;
    rotate2D(refpoint_tf_robot_coord,goal_yaw,refpoint_goal_tf);
    translate2D(refpoint_goal_tf,goal_tf.getOrigin(),refpoint_goal_tf);

    // Then Compute reference goal on reference start coordinates
    refpoint_goal_tf_refstart_coord.frame_id_ = "/refpoint_start_pos";
    refpoint_goal_tf_refstart_coord.stamp_ = goal_tf.stamp_;

    translate2D(refpoint_goal_tf,-refpoint_start_tf.getOrigin(),refpoint_goal_tf_refstart_coord);
    refpoint_start_tf.getBasis().getEulerYPR(temp_yaw, temp_pitch, temp_roll);
    rotate2D(refpoint_goal_tf_refstart_coord,-temp_yaw,refpoint_goal_tf_refstart_coord);    
    
   
    tf::Stamped<tf::Pose> refpoint_midway_goal_tf_refstart_coord; // Midway Goal Refpoint in the the start position of the reference point coordinate frame
    tf::Stamped<tf::Pose> center_midway_goal_tf; // center of rotation of "ideal" midway goal in global coordinate frame
    std::vector< tf::Stamped<tf::Pose> > refpoint_tf_robot_coord_vec;
    
    // the minimiun angle for theta midway is the angle of the 
    double refp_theta_min;
    double refp_theta_max;



    std::vector<geometry_msgs::PoseStamped> plan_first_m;
    std::vector<geometry_msgs::PoseStamped> plan_second_m;
    std::vector<geometry_msgs::PoseStamped>::iterator plan_iterator;

    double midway_side_ovt;
        
    //midway_side_ovt_search_.resetLinearSearch(midway_side_ovt_search_.lin_search_min_, midway_side_ovt_search_.lin_search_max_);
        midway_side_ovt_search_.resetMidSearch(midway_side_ovt_search_.lin_search_min_, midway_side_ovt_search_.lin_search_max_);

    while( midway_side_ovt_search_.midSearch(midway_side_ovt) & !maneuver_traj_succesful){         
                
        // refpoint_goal_tf_refstart_coord.getOrigin().getX()*0.5
        temp_vector3 = tf::Vector3( refpoint_goal_tf_refstart_coord.getOrigin().getX()*0.3 + refpoint_tf_robot_coord.getOrigin().getX(), midway_side_ovt + refpoint_goal_tf_refstart_coord.getOrigin().getY(), 0.0);    
        refpoint_goal_tf_refstart_coord.getBasis().getEulerYPR(temp_yaw,temp_pitch,temp_roll);                
        temp_quat.setRPY(0.0, 0.0, temp_yaw); 
        
        refpoint_midway_goal_tf_refstart_coord.setOrigin(temp_vector3);
        refpoint_midway_goal_tf_refstart_coord.setRotation(temp_quat);        

//         ROS_INFO("refpoint_midway_goal_loc: %f, %f, %f", refpoint_midway_goal_tf_refstart_coord.getOrigin().getX(), refpoint_midway_goal_tf_refstart_coord.getOrigin().getY(), theta_refp_goal);
        
        // Convert back to global coordinates
        temp_quat.setRPY(0.0, 0.0, 0.0); 
        center_midway_goal_tf.setOrigin(-refpoint_tf_robot_coord.getOrigin());
        center_midway_goal_tf.setRotation(temp_quat);
//         rotate2D(center_midway_goal_tf,theta_refp_goal,center_midway_goal_tf); 
        translate2D(center_midway_goal_tf,refpoint_midway_goal_tf_refstart_coord.getOrigin(),center_midway_goal_tf);        
        
        translate2D(center_midway_goal_tf,refpoint_tf_robot_coord.getOrigin(),center_midway_goal_tf);        
        rotate2D(center_midway_goal_tf,start_yaw,center_midway_goal_tf); 
        translate2D(center_midway_goal_tf,start_tf.getOrigin(),center_midway_goal_tf);
        temp_quat.setRPY(0.0, 0.0, goal_yaw); // force rotation to be same as goal. TODO: check transformations!
        center_midway_goal_tf.setRotation(temp_quat); 
                
        temp_start_tf = start_tf;
        temp_goal_tf.setOrigin(center_midway_goal_tf.getOrigin()); 
        temp_goal_tf.setRotation(center_midway_goal_tf.getRotation()); 
        
        
        // For the first part We try to search a single maneuver, if not possible then go to a double maneuver.
        // This is particularly useful when tehre is a replan during teh first phase
        refpoint_tf_robot_coord_vec.push_back(refpoint_tf_robot_coord);
        dist_without_obstacles = 0.0;
        double dist_without_obstacles_single_maneuver;    
        plan_first_m.clear();
        maneuver_traj_succesful = searchTrajectorySingleManeuver(temp_start_tf, temp_goal_tf, refpoint_tf_robot_coord_vec, plan_first_m, dist_without_obstacles_single_maneuver);
        if (!maneuver_traj_succesful)
            maneuver_traj_succesful = searchTrajectoryLeftRightManeuver(temp_start_tf, temp_goal_tf, refpoint_tf_robot_coord, plan_first_m, dist_without_obstacles_single_maneuver);
        
        dist_without_obstacles = dist_without_obstacles_single_maneuver;
        if (!maneuver_traj_succesful)
            continue;
        
        
        
        maneuver_traj_succesful = false;
    //         ROS_INFO("First local trajectory created. Second maneuver reached");          
        plan_iterator = plan_first_m.end()-1;
        // Take last pose of previus plan
        poseStampedMsgToTF(*plan_iterator,temp_start_tf); 
        

        
        temp_goal_tf.setOrigin(goal_tf.getOrigin());
        temp_goal_tf.setRotation(goal_tf.getRotation());
        
        plan_second_m.clear();
        maneuver_traj_succesful = searchTrajectoryLeftRightManeuver(temp_start_tf, temp_goal_tf, refpoint_tf_robot_coord, plan_second_m, dist_without_obstacles);
        dist_without_obstacles = dist_without_obstacles + dist_without_obstacles_single_maneuver;
    //         ROS_INFO("Second trajectory maneuver_traj_succesful: %d", maneuver_traj_succesful);
        
        
    }
  

    for (plan_iterator = plan_first_m.begin(); plan_iterator != plan_first_m.end(); plan_iterator++)
    {
        plan.push_back(*plan_iterator);
    }
    for (plan_iterator = plan_second_m.begin(); plan_iterator != plan_second_m.end(); plan_iterator++)
    {
        plan.push_back(*plan_iterator);
    }            

  
//   ROS_INFO("Total trajectory created. Check trajectory, free: %d", (int) maneuver_traj_succesful);
  return maneuver_traj_succesful;
}



bool ManeuverPlanner::searchTrajectoryLeftRightManeuver(const tf::Stamped<tf::Pose>& start_tf, const tf::Stamped<tf::Pose>& goal_tf, 
                                                        const tf::Stamped<tf::Pose>& refpoint_tf_robot_coord, std::vector<geometry_msgs::PoseStamped>& plan, double & dist_without_obstacles)
{
    tf::Stamped<tf::Pose> midway_goal_tf;
    midway_goal_tf.frame_id_    = goal_tf.frame_id_;
    midway_goal_tf.stamp_       = goal_tf.stamp_;
            
    
    tf::Stamped<tf::Pose> refpoint_start_tf; // Start Refpoint in the global coordinate frame
    tf::Stamped<tf::Pose> refpoint_goal_tf; // Goal Refpoint in the global coordinate frame        
    tf::Stamped<tf::Pose> refpoint_goal_tf_refstart_coord; // Goal Refpoint in the the start position of the reference point coordinate frame
    tf::Stamped<tf::Pose> refpoint_goal_tf_refmidway_coord; // Goal Refpoint in the the midway position of the reference point coordinate frame. Used in second maneuver
    double dist_before_steering_refp, dist_after_steering_refp, signed_turning_radius_refp;
    double unsigned_radius;
    double dist_before_steering_center, dist_after_steering_center, signed_turning_radius_center;
    int curve_type;  
    bool maneuver_traj_succesful = false;
    double start_yaw, goal_yaw, temp_yaw, temp_pitch, temp_roll;
    tf::Quaternion temp_quat;
    tf::Vector3 temp_vector3;
    start_tf.getBasis().getEulerYPR(start_yaw, temp_pitch, temp_roll);             
    goal_tf.getBasis().getEulerYPR(goal_yaw, temp_pitch, temp_roll);             
    
   
    // Compute reference start and goal on global coordinates
    refpoint_start_tf.frame_id_ = goal_tf.frame_id_;
    refpoint_start_tf.stamp_ = goal_tf.stamp_;
    rotate2D(refpoint_tf_robot_coord,start_yaw,refpoint_start_tf);
    translate2D(refpoint_start_tf,start_tf.getOrigin(),refpoint_start_tf);

    refpoint_goal_tf.frame_id_ = goal_tf.frame_id_;
    refpoint_goal_tf.stamp_ = goal_tf.stamp_;
    rotate2D(refpoint_tf_robot_coord,goal_yaw,refpoint_goal_tf);
    translate2D(refpoint_goal_tf,goal_tf.getOrigin(),refpoint_goal_tf);

    // Then Compute reference start and goal on reference start coordinates
    refpoint_goal_tf_refstart_coord.frame_id_ = "/refpoint_start_pos";
    refpoint_goal_tf_refstart_coord.stamp_ = goal_tf.stamp_;

    translate2D(refpoint_goal_tf,-refpoint_start_tf.getOrigin(),refpoint_goal_tf_refstart_coord);
    refpoint_start_tf.getBasis().getEulerYPR(temp_yaw, temp_pitch, temp_roll);
    rotate2D(refpoint_goal_tf_refstart_coord,-temp_yaw,refpoint_goal_tf_refstart_coord);    
    
    tf::Stamped<tf::Pose> refpoint_midway_goal_tf_refstart_coord; // Midway Goal Refpoint in the the start position of the reference point coordinate frame
    // the minimiun angle for theta midway is the angle of the 
    double refp_theta_min;
    double refp_theta_max;
    double theta_refp_goal;
    
    double signed_max_turning_radius_refp;    // Maximum steering radius using the eference point
    double xlocal_intersection_refp;          // Intersection of target in local x coodinates using the reference point    
    int maneuver_type_refp;
    std::vector<tf::Pose> local_plan_refp;    
    std::vector<tf::Pose> local_plan_refp_first_m;
    std::vector<tf::Pose> local_plan_refp_second_m;
    
    double midway_scale_lr;
    
    midway_scale_lr_search_.resetMidSearch(midway_scale_lr_search_.lin_search_min_, midway_scale_lr_search_.lin_search_max_);
    
    while( midway_scale_lr_search_.midSearch(midway_scale_lr) & !maneuver_traj_succesful){ 
//         ROS_INFO("Search midway scale: %f", midway_scale_lr);
               
    
        temp_vector3 = tf::Vector3( midway_scale_lr*refpoint_goal_tf_refstart_coord.getOrigin().getX(), refpoint_goal_tf_refstart_coord.getOrigin().getY()/2.0, 0.0);    
        refpoint_midway_goal_tf_refstart_coord.setOrigin(temp_vector3);
        refp_theta_min = std::atan2(refpoint_midway_goal_tf_refstart_coord.getOrigin().getY(), refpoint_midway_goal_tf_refstart_coord.getOrigin().getX());    
        if( refp_theta_min > 0)   // Left right
            refp_theta_max = M_PI/2.0;
        else // Right left
            refp_theta_max = -M_PI/2.0;
        
        theta_refp_goal = angles::normalize_angle(refp_theta_min + 0.5*(refp_theta_max-refp_theta_min)  ); // Final angle of mid goal // For now set a fix value. We can do also a search on this parameter_generator 
        temp_quat.setRPY(0.0, 0.0, theta_refp_goal); 
        refpoint_midway_goal_tf_refstart_coord.setRotation(temp_quat);
        
//         ROS_INFO("refpoint_midway_goal_tf_refstart_coord %f, %f", refpoint_midway_goal_tf_refstart_coord.getOrigin().getX(), refpoint_midway_goal_tf_refstart_coord.getOrigin().getY());
            
    //     ROS_INFO("First maneuver reached");
        // First maneuver    
        maneuver_type_refp = determineManeuverType(refpoint_midway_goal_tf_refstart_coord,  signed_max_turning_radius_refp, xlocal_intersection_refp);            
        if (maneuver_type_refp == ManeuverPlanner::MANEUVER_LEFT || maneuver_type_refp == ManeuverPlanner::MANEUVER_RIGHT)
        {   // This only supports single maneuvers    
        
            radius_search_.resetMidSearch(radius_search_.lin_search_min_, std::abs(signed_max_turning_radius_refp));
            while( radius_search_.midSearch(unsigned_radius) & !maneuver_traj_succesful)
            {           
                if(signed_max_turning_radius_refp > 0.0)
                    signed_turning_radius_refp = unsigned_radius;
                else
                    signed_turning_radius_refp = -unsigned_radius;
                                    
    //             ROS_INFO(" signed_turning_radius_first_maneuver: %.3f", signed_turning_radius_refp);
                curve_type = computeSingleManeuverParameters(refpoint_midway_goal_tf_refstart_coord, signed_turning_radius_refp,  xlocal_intersection_refp, dist_before_steering_refp, dist_after_steering_refp);
//                 ROS_INFO("Curve parameters: %.3f / %.3f / %.3f",dist_before_steering_refp, dist_after_steering_refp, signed_turning_radius_refp);             
                if( curve_type!= ManeuverPlanner::CURVE_NONE ) // curve possible, generate
                {
                    generateDublinTrajectory(dist_before_steering_refp, dist_after_steering_refp, signed_turning_radius_refp, theta_refp_goal, local_plan_refp_first_m);  
                    plan.clear();
                    maneuver_traj_succesful = checkFootprintTransformLocalToGlobal(start_tf, goal_tf, refpoint_tf_robot_coord, local_plan_refp_first_m, plan, dist_without_obstacles); // here goal_tf is only  
                }                
            }                                
        } 

        
        if (!maneuver_traj_succesful)
            continue;
        
        maneuver_traj_succesful = false;
//         ROS_INFO("First local trajectory created. Second maneuver reached");   
        
        refpoint_goal_tf_refmidway_coord = refpoint_goal_tf_refstart_coord;
        translate2D(refpoint_goal_tf_refmidway_coord,-refpoint_midway_goal_tf_refstart_coord.getOrigin(),refpoint_goal_tf_refmidway_coord);
        refpoint_midway_goal_tf_refstart_coord.getBasis().getEulerYPR(temp_yaw, temp_pitch, temp_roll);
        rotate2D(refpoint_goal_tf_refmidway_coord,-temp_yaw,refpoint_goal_tf_refmidway_coord); 
        
        refpoint_goal_tf_refmidway_coord.getBasis().getEulerYPR(theta_refp_goal,temp_pitch,temp_roll);        
        maneuver_type_refp = determineManeuverType(refpoint_goal_tf_refmidway_coord,  signed_max_turning_radius_refp, xlocal_intersection_refp);            
        if (maneuver_type_refp == ManeuverPlanner::MANEUVER_LEFT || maneuver_type_refp == ManeuverPlanner::MANEUVER_RIGHT)
        {   // This only supports single maneuvers    
                
            
            radius_search_.resetMidSearch(radius_search_.lin_search_min_, std::abs(signed_max_turning_radius_refp));
            while( radius_search_.midSearch(unsigned_radius) & !maneuver_traj_succesful)
            {      
                if(signed_max_turning_radius_refp > 0.0)
                    signed_turning_radius_refp = unsigned_radius;
                else
                    signed_turning_radius_refp = -unsigned_radius;
//                 ROS_INFO(" signed_turning_radius_second_maneuver: %.3f", signed_turning_radius_refp);
                curve_type = computeSingleManeuverParameters(refpoint_goal_tf_refmidway_coord, signed_turning_radius_refp,  xlocal_intersection_refp, dist_before_steering_refp, dist_after_steering_refp);
                // ROS_INFO("Curve parameters: %.3f / %.3f / %.3f",dist_before_steering_refp, dist_after_steering_refp, signed_turning_radius_refp);             
                if( curve_type!= ManeuverPlanner::CURVE_NONE ) // curve possible, generate
                {
                    generateDublinTrajectory(dist_before_steering_refp, dist_after_steering_refp, signed_turning_radius_refp, theta_refp_goal, local_plan_refp_second_m);                
                //     ROS_INFO("Second local trajectory created");
                    
                    std::vector<tf::Pose>::iterator plan_iterator;
                    tf::Pose pose_local_plan;
                    tf::Stamped<tf::Pose> point_local_plan_refp_second_m_stamped;
                    
                    local_plan_refp.clear();
                    // Add first plan. TODO: This is inefficient implementation. Later make sure rechecks are not done
                    for (plan_iterator = local_plan_refp_first_m.begin(); plan_iterator != local_plan_refp_first_m.end(); plan_iterator++)
                    {
                        local_plan_refp.push_back(*plan_iterator);
                    }
                    
                    for (plan_iterator = local_plan_refp_second_m.begin()+1; plan_iterator != local_plan_refp_second_m.end(); plan_iterator++)
                    {
                        point_local_plan_refp_second_m_stamped.setData(*plan_iterator);        
                        
                        // transform back from the middle way point 
                        refpoint_midway_goal_tf_refstart_coord.getBasis().getEulerYPR(temp_yaw, temp_pitch, temp_roll);
                        rotate2D(point_local_plan_refp_second_m_stamped,+temp_yaw,point_local_plan_refp_second_m_stamped); 
                        translate2D(point_local_plan_refp_second_m_stamped, refpoint_midway_goal_tf_refstart_coord.getOrigin(), point_local_plan_refp_second_m_stamped);                 
                        pose_local_plan.setOrigin(point_local_plan_refp_second_m_stamped.getOrigin());
                        pose_local_plan.setRotation(point_local_plan_refp_second_m_stamped.getRotation());
                        local_plan_refp.push_back(pose_local_plan);
                    }
                
                    plan.clear();
                    maneuver_traj_succesful = checkFootprintTransformLocalToGlobal(start_tf, goal_tf, refpoint_tf_robot_coord, local_plan_refp, plan, dist_without_obstacles); // here goal_tf is only                        
                    
                }
            }                                
        }       
  }
  
//   ROS_INFO("Total trajectory created. Check trajectory, free: %d", (int) maneuver_traj_succesful);
  return maneuver_traj_succesful;

}



bool ManeuverPlanner::searchTrajectorySingleManeuver(const tf::Stamped<tf::Pose>& start_tf, 
                               const tf::Stamped<tf::Pose>& goal_tf, std::vector< tf::Stamped<tf::Pose> >& refpoint_tf_robot_coord_vec, 
                               std::vector<geometry_msgs::PoseStamped>& plan, double & dist_without_obstacles)
{
    tf::Stamped<tf::Pose> refpoint_tf_robot_coord;
    tf::Stamped<tf::Pose> refpoint_start_tf; // Start Refpoint in the global coordinate frame
    tf::Stamped<tf::Pose> refpoint_goal_tf; // Goal Refpoint in the global coordinate frame        
    tf::Stamped<tf::Pose> refpoint_goal_tf_refstart_coord; // Goal Refpoint in the the start position of the reference point coordinate frame
    double dist_before_steering_refp, dist_after_steering_refp, signed_turning_radius_refp;
    double dist_before_steering_center, dist_after_steering_center, signed_turning_radius_center;
    int curve_type;  
    bool maneuver_traj_succesful = false;
    double start_yaw, goal_yaw, temp_yaw, temp_pitch, temp_roll;    
    tf::Quaternion temp_quat;
    tf::Vector3 temp_vector3;
    start_tf.getBasis().getEulerYPR(start_yaw, temp_pitch, temp_roll);             
    goal_tf.getBasis().getEulerYPR(goal_yaw, temp_pitch, temp_roll);             
    
    tf::Stamped<tf::Pose> goal_tf_start_coord; // Goal in the start vector coordinates
    goal_tf_start_coord.frame_id_ = "/center_rotation_start_pos";

    translate2D(goal_tf,-start_tf.getOrigin(),goal_tf_start_coord);
    rotate2D(goal_tf_start_coord,-start_yaw,goal_tf_start_coord);
    double theta_refp_goal = angles::normalize_angle( goal_yaw - start_yaw ); // Final angle of curvature    
    
    double min_radius = radius_search_.lin_search_min_;        
    double signed_max_turning_radius_center;    // Maximum steering radius using the center of the robot
    double xlocal_intersection_center;          // Intersection of target in local x coodinates
    int maneuver_type_center;
    std::vector<tf::Pose> local_plan_refp;
    double unsigned_radius;
    double signed_max_turning_radius_refp;    // Maximum steering radius using the eference point
    double xlocal_intersection_refp;          // Intersection of target in local x coodinates using the reference point    
    int maneuver_type_refp;

    std::vector< tf::Stamped<tf::Pose> >::iterator refpoint_tf_robot_coord_it;
     
    for ( refpoint_tf_robot_coord_it = refpoint_tf_robot_coord_vec.begin(); refpoint_tf_robot_coord_it!=refpoint_tf_robot_coord_vec.end(); refpoint_tf_robot_coord_it++)
    {
        refpoint_tf_robot_coord = *refpoint_tf_robot_coord_it;
   
    
        // Compute reference start and goal on global coordinates
        refpoint_start_tf.frame_id_ = goal_tf.frame_id_;
        refpoint_start_tf.stamp_ = goal_tf.stamp_;
        rotate2D(refpoint_tf_robot_coord,start_yaw,refpoint_start_tf);
        translate2D(refpoint_start_tf,start_tf.getOrigin(),refpoint_start_tf);

        refpoint_goal_tf.frame_id_ = goal_tf.frame_id_;
        refpoint_goal_tf.stamp_ = goal_tf.stamp_;
        rotate2D(refpoint_tf_robot_coord,goal_yaw,refpoint_goal_tf);
        translate2D(refpoint_goal_tf,goal_tf.getOrigin(),refpoint_goal_tf);

        // Then Compute reference start and goal on reference start coordinates
        refpoint_goal_tf_refstart_coord.frame_id_ = "/refpoint_start_pos";
        refpoint_goal_tf_refstart_coord.stamp_ = goal_tf.stamp_;

        translate2D(refpoint_goal_tf,-refpoint_start_tf.getOrigin(),refpoint_goal_tf_refstart_coord);
        refpoint_start_tf.getBasis().getEulerYPR(temp_yaw, temp_pitch, temp_roll);
        rotate2D(refpoint_goal_tf_refstart_coord,-temp_yaw,refpoint_goal_tf_refstart_coord);

        maneuver_type_refp = determineManeuverType(refpoint_goal_tf_refstart_coord,  signed_max_turning_radius_refp, xlocal_intersection_refp);        
                
        if (maneuver_type_refp == ManeuverPlanner::MANEUVER_LEFT || maneuver_type_refp == ManeuverPlanner::MANEUVER_RIGHT)
        {   // This only supports single maneuvers    
                
            radius_search_.resetMidSearch(min_radius, std::abs(signed_max_turning_radius_refp));
            while( radius_search_.midSearch(unsigned_radius) & !maneuver_traj_succesful)
            {      
                if(signed_max_turning_radius_refp > 0.0)
                    signed_turning_radius_refp = unsigned_radius;
                else
                    signed_turning_radius_refp = -unsigned_radius;         
//                 ROS_INFO(" signed_turning_radius_refp: %.3f", signed_turning_radius_refp);
                curve_type = computeSingleManeuverParameters(refpoint_goal_tf_refstart_coord, signed_turning_radius_refp,  xlocal_intersection_refp, dist_before_steering_refp, dist_after_steering_refp);
//                 ROS_INFO("Curve parameters: %.3f / %.3f / %.3f",dist_before_steering_refp, dist_after_steering_refp, signed_turning_radius_refp);             
                plan.clear();
                if( curve_type!= ManeuverPlanner::CURVE_NONE ) // curve possible, generate
                {
                    generateDublinTrajectory(dist_before_steering_refp, dist_after_steering_refp, signed_turning_radius_refp, theta_refp_goal, local_plan_refp);
                    maneuver_traj_succesful = checkFootprintTransformLocalToGlobal(start_tf, goal_tf, refpoint_tf_robot_coord, local_plan_refp, plan, dist_without_obstacles);
                }
            }                        
            
        }


        if (maneuver_traj_succesful)
            break;
    }

    return maneuver_traj_succesful;

}



bool ManeuverPlanner::linePlanner(const geometry_msgs::PoseStamped& start,
                               const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan, double &dist_without_obstacles)
{
    /***** Line planner ****/
    // We want to step forward along the vector created by the robot's position and the goal pose until we find an illegal cell
    tf::Stamped<tf::Pose> start_tf, goal_tf; 
    double start_yaw, goal_yaw, useless_pitch, useless_roll;
    poseStampedMsgToTF(goal,goal_tf);
    poseStampedMsgToTF(start,start_tf);
    start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
    goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);
    
    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;
    double start_x = start.pose.position.x;
    double start_y = start.pose.position.y;

    double diff_x = goal_x - start_x;
    double diff_y = goal_y - start_y;
    double diff_yaw = angles::normalize_angle(goal_yaw-start_yaw);

    double target_x = start_x;
    double target_y = start_y;
    double target_yaw = goal_yaw;

    bool done = false;
    double scale = 0.0;
    double dScale = 0.05;
    double footprint_cost;
    bool traj_free = true;
    while(!done)
    {
        if(scale > 1.0)
        {
            target_x = start_x;
            target_y = start_y;
            target_yaw = start_yaw;

            done = true;
            break;
        }
        target_x = start_x + scale * diff_x;
        target_y = start_y + scale * diff_y;
        target_yaw = angles::normalize_angle(start_yaw + scale * diff_yaw);

        footprint_cost = footprintCost(target_x, target_y, target_yaw);
        if(footprint_cost < 0)
        {
            done = true;
            traj_free = false;
            break;
        }
        scale +=dScale;

        geometry_msgs::PoseStamped new_goal = goal;
        tf::Quaternion goal_quat = tf::createQuaternionFromYaw(target_yaw);

        new_goal.pose.position.x = target_x;
        new_goal.pose.position.y = target_y;

        new_goal.pose.orientation.x = goal_quat.x();
        new_goal.pose.orientation.y = goal_quat.y();
        new_goal.pose.orientation.z = goal_quat.z();
        new_goal.pose.orientation.w = goal_quat.w();

        plan.push_back(new_goal);

    }
    
    dist_without_obstacles = scale*std::sqrt(diff_x*diff_x+diff_y*diff_y);
    
    if(scale < 1.0)
    {
        ROS_WARN("Line planner could not find a free path for this goal");        
    }
    return traj_free;

}

bool ManeuverPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                               const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan, double & dist_without_obstacles)
{
    return makePlanUntilPossible(start, goal, plan, dist_without_obstacles);
}

bool ManeuverPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                               const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
    double dist_without_obstacles;
    return makePlanUntilPossible(start, goal, plan, dist_without_obstacles);
}


bool ManeuverPlanner::makePlanUntilPossible(const geometry_msgs::PoseStamped& start,
                               const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan, double & dist_without_obstacles)
{

    if(!initialized_)
    {
        ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
        return false;
    }

    ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    plan.clear();
    costmap_ = costmap_ros_->getCostmap();

    if(goal.header.frame_id != costmap_ros_->getGlobalFrameID())
    {
        ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
                  costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
        return false;
    }

    tf::Stamped<tf::Pose> goal_tf;
    tf::Stamped<tf::Pose> start_tf;
    double useless_pitch, useless_roll, goal_yaw, start_yaw;

    /*
        poseStampedMsgToTF(goal,goal_tf);
        poseStampedMsgToTF(start,start_tf);

        start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
        goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);
       */
    /****************************/
    double temp_yaw, temp_pitch, temp_roll;
    tf::Quaternion temp_quat;
    tf::Vector3 temp_vector3;

    if( last_goal_as_start_ & valid_last_goal_)
    {
        start_ = last_goal_;
    }
    else
    {
        start_ = start;
    }

    poseStampedMsgToTF(goal,goal_tf);
    poseStampedMsgToTF(start_,start_tf);


    start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
    goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);


    tf::Stamped<tf::Pose> goal_tf_start_coord; // Goal in the start vector coordinates
    goal_tf_start_coord.frame_id_ = "/center_rotation_start_pos";

    translate2D(goal_tf,-start_tf.getOrigin(),goal_tf_start_coord);
    rotate2D(goal_tf_start_coord,-start_yaw,goal_tf_start_coord);

    
    // Initially compute the type of maneuver, use the center of the robot. This is only done once
    double signed_max_turning_radius_center;    // Maximum steering radius using the center of the robot
    double xlocal_intersection_center;          // Intersection of target in local x coodinates
    int maneuver_type = determineManeuverType(goal_tf_start_coord,  signed_max_turning_radius_center, xlocal_intersection_center);
    
    /* Next, depending on the type of maneuver, trajectories are generated using a preferred reference point on the robot 
     * If after exploration maneuver is not possible, then search again using the center of the robot
     * If that is also not possible then use carrot planner
     * If all fails, report that no free path is found
    */
    
    tf::Stamped<tf::Pose> refpoint_tf_robot_coord; // Refpoint in the robot(+load) coordinate frame
    std::vector< tf::Stamped<tf::Pose> > refpoint_tf_robot_coord_vec;
    double maneuver_traj_succesful = false;
    
    
    if (maneuver_type != ManeuverPlanner::MANEUVER_NONE)
    {
        switch (maneuver_type)
        {
        case ManeuverPlanner::MANEUVER_LEFT :
            // Initially choose top right corner (trc) as reference to generate
            // trajectory. Suitable for when trc is desired to keep parallel to the
            // wall. If the trc trajectory is not convex, we switch back
            // reference point to middle of the rotation axis.
            refpoint_tf_robot_coord.frame_id_ = "/wholerobot_link";
            refpoint_tf_robot_coord.stamp_ = goal_tf.stamp_;
            temp_quat.setRPY(0.0,0.0,0.0);
            refpoint_tf_robot_coord_vec.clear();
            
            temp_vector3 = tf::Vector3(0.0, 0.0, 0.0);
            refpoint_tf_robot_coord.setData(tf::Transform(temp_quat,temp_vector3));
            refpoint_tf_robot_coord_vec.push_back(refpoint_tf_robot_coord);            
            
            temp_vector3 = tf::Vector3(topRightCorner_[0], topRightCorner_[1], 0.0);
            refpoint_tf_robot_coord.setData(tf::Transform(temp_quat,temp_vector3));
            refpoint_tf_robot_coord_vec.push_back(refpoint_tf_robot_coord);            
            
            temp_vector3 = tf::Vector3(left_side_ref_point_[0], left_side_ref_point_[1], 0.0);
            refpoint_tf_robot_coord.setData(tf::Transform(temp_quat,temp_vector3));
            refpoint_tf_robot_coord_vec.push_back(refpoint_tf_robot_coord);
            
            ROS_INFO("Left turn");
            maneuver_traj_succesful = searchTrajectorySingleManeuver(start_tf, goal_tf, refpoint_tf_robot_coord_vec, plan, dist_without_obstacles);
            break;

        case ManeuverPlanner::MANEUVER_RIGHT :
            // Choose a point on the right side, just above axis of rotation
            // as reference to generate trajectory
            // Initially choose right side as reference to generate
            // trajectory. Suitable for when that point should just round the corner.
            // If recomputing trajectory is not possible, the switch back
            // reference point to middle of the rotation axis.
            refpoint_tf_robot_coord.frame_id_ = "/wholerobot_link";
            refpoint_tf_robot_coord.stamp_ = goal_tf.stamp_;            
            temp_quat.setRPY(0.0,0.0,0.0);
            refpoint_tf_robot_coord_vec.clear();
            
            temp_vector3 = tf::Vector3(0.0, 0.0, 0.0);
            refpoint_tf_robot_coord.setData(tf::Transform(temp_quat,temp_vector3));
            refpoint_tf_robot_coord_vec.push_back(refpoint_tf_robot_coord);             
            
            temp_vector3 = tf::Vector3(right_side_ref_point_[0], right_side_ref_point_[1], 0.0);
            refpoint_tf_robot_coord.setData(tf::Transform(temp_quat,temp_vector3));            
            refpoint_tf_robot_coord_vec.push_back(refpoint_tf_robot_coord);              
            
            temp_vector3 = tf::Vector3(topLeftCorner_[0], topLeftCorner_[1], 0.0);
            refpoint_tf_robot_coord.setData(tf::Transform(temp_quat,temp_vector3));            
            refpoint_tf_robot_coord_vec.push_back(refpoint_tf_robot_coord);
            
            
            
            ROS_INFO("Right turn");
            maneuver_traj_succesful = searchTrajectorySingleManeuver(start_tf, goal_tf, refpoint_tf_robot_coord_vec, plan, dist_without_obstacles);
            
            break;            
        case ManeuverPlanner::MANEUVER_LEFT_RIGHT :
            // Initially choose top left corner (trc) as reference to generate
            // trajectory. Suitable for when tlc is desired to keep parallel to the
            // left wall. 
            refpoint_tf_robot_coord.frame_id_ = "/wholerobot_link";
            refpoint_tf_robot_coord.stamp_ = goal_tf.stamp_;
            temp_quat.setRPY(0.0,0.0,0.0);
            
//             temp_vector3 = tf::Vector3(topLeftCorner_[0], topLeftCorner_[1], 0.0);
            temp_vector3 = tf::Vector3(topRightCorner_[0], topRightCorner_[1], 0.0);
            refpoint_tf_robot_coord.setData(tf::Transform(temp_quat,temp_vector3));
            
            ROS_INFO("Left Right turn"); 
            maneuver_traj_succesful = searchTrajectoryLeftRightManeuver(start_tf, goal_tf, refpoint_tf_robot_coord, plan, dist_without_obstacles);            
            break;
        case ManeuverPlanner::MANEUVER_RIGHT_LEFT :
            // Initially choose top left corner (trc) as reference to generate
            // trajectory. Suitable for when tlc is desired to keep parallel to the
            // left wall. 
            refpoint_tf_robot_coord.frame_id_ = "/wholerobot_link";
            refpoint_tf_robot_coord.stamp_ = goal_tf.stamp_;
            temp_quat.setRPY(0.0,0.0,0.0);

            temp_vector3 = tf::Vector3(topRightCorner_[0], topRightCorner_[1], 0.0);
            refpoint_tf_robot_coord.setData(tf::Transform(temp_quat,temp_vector3));

            
            ROS_INFO("Right Left turn"); 
            maneuver_traj_succesful = searchTrajectoryLeftRightManeuver(start_tf, goal_tf, refpoint_tf_robot_coord, plan, dist_without_obstacles);            
            break;     
        case ManeuverPlanner::MANEUVER_STRAIGHT_OTHERWISE_OVERTAKE :
            maneuver_traj_succesful = linePlanner(start, goal, plan, dist_without_obstacles);
            ROS_INFO("Try straight line"); 
            if( maneuver_traj_succesful == false)
            {                
                ROS_INFO("Overtake maneuver"); 
                refpoint_tf_robot_coord.frame_id_ = "/wholerobot_link";
                refpoint_tf_robot_coord.stamp_ = goal_tf.stamp_;
                temp_quat.setRPY(0.0,0.0,0.0);

                temp_vector3 = tf::Vector3(topRightCorner_[0], topRightCorner_[1], 0.0);
                refpoint_tf_robot_coord.setData(tf::Transform(temp_quat,temp_vector3));         
                plan.clear();
                maneuver_traj_succesful = searchTrajectoryOvertakeManeuver(start_tf, goal_tf, refpoint_tf_robot_coord, plan, dist_without_obstacles);        
            }
            
            break;            
        default:
            break;
        }

    }
   
   
    
    if( maneuver_traj_succesful == false) // Maneuver planning failed. Attemp linear planner
    {
        ROS_WARN("No single or double left or right maneuver possible. Execute default planner and or ");
        plan.clear();
        maneuver_traj_succesful = linePlanner(start, goal, plan, dist_without_obstacles);
        if( maneuver_type != ManeuverPlanner::MANEUVER_STRAIGHT_OTHERWISE_OVERTAKE && maneuver_traj_succesful == false)
        {
            ROS_INFO("Overtake maneuver, Last resource"); 
            refpoint_tf_robot_coord.frame_id_ = "/wholerobot_link";
            refpoint_tf_robot_coord.stamp_ = goal_tf.stamp_;
            temp_quat.setRPY(0.0,0.0,0.0);

            temp_vector3 = tf::Vector3(topRightCorner_[0], topRightCorner_[1], 0.0);
            refpoint_tf_robot_coord.setData(tf::Transform(temp_quat,temp_vector3));         
            plan.clear();
            maneuver_traj_succesful = searchTrajectoryOvertakeManeuver(start_tf, goal_tf, refpoint_tf_robot_coord, plan, dist_without_obstacles);                    
        }
    }

    if(maneuver_traj_succesful == false){
        ROS_WARN("No free trajectory found");
    }
    return maneuver_traj_succesful;

}



};

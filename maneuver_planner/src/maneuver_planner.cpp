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
        if( y_target >= 0.0 ) // Target in front left of the robot
        { 
            if ( yaw_target > theta_target && yaw_target < M_PI ) // Execute single left turn maneuver
            { 
                ROS_INFO(" Target in front left of the robot: Execute single left turn maneuver");
                signed_max_turning_radius = abs_max_turning_radius;
                maneuver_type = ManeuverPlanner::MANEUVER_LEFT;
            }
            else if( yaw_target < theta_target && yaw_target > -M_PI/2.0 ) // Execute left and then turn right
            { 
                ROS_INFO(" Target in front left of the robot: Execute left and then turn right");
                maneuver_type = ManeuverPlanner::MANEUVER_LEFT_RIGHT;
            }
            else // This could be executed with a left maneuver > M_PI turn, or multiple maneuvers. For now use carrot planner
            {
                ROS_INFO(" Target in front left of the robot: Uncoventional orientation. Execute carrot planner");
                maneuver_type = ManeuverPlanner::MANEUVER_NONE;
            }
        }
        else // Target in front right of the robot
        {
            if ( yaw_target < theta_target && yaw_target > - M_PI ) // Execute single right turn maneuver
            { 
                ROS_INFO(" Target in front right of the robot: Execute single right turn maneuver");
                signed_max_turning_radius = -abs_max_turning_radius;;
                maneuver_type = ManeuverPlanner::MANEUVER_RIGHT;
            }
            else if( yaw_target > theta_target && yaw_target < M_PI/2.0 ) // Execute right and then turn left
            {
                ROS_INFO(" Target in front right of the robot:  Execute right and then turn left");
                // maneuver_type = ManeuverPlanner::MANEUVER_RIGHT_LEFT;
                maneuver_type = ManeuverPlanner::MANEUVER_NONE;
            }
            else // This could be executed with a right maneuver > M_PI turn, or multiple maneuvers. For now use carrot planner
            {
                ROS_INFO(" Target in front left of the robot: Uncoventional orientation. Execute carrot planner");
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

bool ManeuverPlanner::generateTrajectorySingleManeuver(const tf::Stamped<tf::Pose>& start_tf, tf::Stamped<tf::Pose> refpoint_goal_tf_refstart_coord,
                               const tf::Stamped<tf::Pose>& goal_tf, const tf::Stamped<tf::Pose>& refpoint_tf_robot_coord, 
                               const double& dist_before_steering_refp, const double& dist_after_steering_refp, 
                               const double& signed_turning_radius_refp, std::vector<geometry_msgs::PoseStamped>& plan) //, std::vector<tf::Pose> &localplan)
{
 
    double start_yaw, temp_yaw, temp_pitch, temp_roll;
    tf::Quaternion temp_quat;
    tf::Vector3 temp_vector3;
    start_tf.getBasis().getEulerYPR(start_yaw, temp_pitch, temp_roll);        

    double footprint_cost;
    int counter = 1;
    Eigen::Matrix2d jacobian_motrefPoint;
    Eigen::Matrix2d invjacobian_motrefPoint;

    tf::Stamped<tf::Pose> ref_traj_point_tf_refstart_coord; // current  Refpoint in the the start position of the reference point coordinate frame
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

    // Compute trajectory as points. Only the center of teh robot has a pose and orientation
    motion_refpoint_localtraj_ << 0.0, 0.0; // starts at origin by definition
    prev_motion_refpoint_localtraj_ = motion_refpoint_localtraj_;

    double theta_refp_goal; // Final angle of curvature
    refpoint_goal_tf_refstart_coord.getBasis().getEulerYPR(theta_refp_goal, temp_pitch, temp_roll);
    double theta_refp_traj = 0.0 ; // This is only to control evolution of the curvature.
    double theta_refp_traj_gridsz = step_size_/signed_turning_radius_refp; // Gridsize of the trajectory angle
    double dist_bef_steer = 0.0, dist_af_steer = 0.0;
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


    bool traj_ready = false;
    bool traj_free = true;


//     ROS_ERROR("Reached here %d", counter);counter++;

    while (!traj_ready)
    {
//       counter++;
        // Check for obstacles of last computed traj pose
        center_traj_point_tf.getBasis().getEulerYPR(temp_yaw, temp_pitch, temp_roll);
        temp_vector3 = center_traj_point_tf.getOrigin();
        footprint_cost = footprintCost(temp_vector3.getX(), temp_vector3.getY(), temp_yaw);
        if(footprint_cost < 0)
        {
            // Abort. Inform that not all trajectory is free of obstcales
            // traj_ready = true;
            traj_free  = false;
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
        // Generate next reference refpoint in trajectory
        if( dist_bef_steer <  dist_before_steering_refp)
        {   // Move straight before steering
            theta_refp_traj = 0;
            motion_refpoint_localtraj_[0] = prev_motion_refpoint_localtraj_[0]+ step_size_;
            motion_refpoint_localtraj_[1] = prev_motion_refpoint_localtraj_[1];
            dist_bef_steer += step_size_;
        } 
        else if(std::abs(theta_refp_goal-theta_refp_traj) > std::abs(theta_refp_traj_gridsz/2.0))
        {   // Turn with circle. This can be as well a clothoid!
            theta_refp_traj +=theta_refp_traj_gridsz;
            motion_refpoint_localtraj_[0] = dist_before_steering_refp +signed_turning_radius_refp*std::sin(theta_refp_traj);
            motion_refpoint_localtraj_[1] = signed_turning_radius_refp*(1.0 - std::cos(theta_refp_traj));
        } 
        else if( dist_af_steer <  dist_after_steering_refp)
        {   // Move straight after steering
            theta_refp_traj = theta_refp_goal;
            motion_refpoint_localtraj_[0] = prev_motion_refpoint_localtraj_[0]+ step_size_*std::cos(theta_refp_traj);
            motion_refpoint_localtraj_[1] = prev_motion_refpoint_localtraj_[1]+ step_size_*std::sin(theta_refp_traj);
            dist_af_steer += step_size_;
        }
        else
        {
            traj_ready = true;
            break;
        }


        // Now compute robot center of rotation trajectory
        // Compute virtual velocity of reference point. Virtual time of 1.0 sec
        Eigen::Vector2d motion_refpoint_virvel_loctrajframe;
        motion_refpoint_virvel_loctrajframe = (motion_refpoint_localtraj_ - prev_motion_refpoint_localtraj_)/1.0;
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


        /*if(counter >= 17 && counter <= 20)
        {
            center_traj_point_tf.getBasis().getEulerYPR(temp_yaw,temp_pitch,temp_roll);
        ROS_INFO("center_traj_point_tf: x= %.3f y= %.3f theta= %.3f", center_traj_point_tf.getOrigin().getX(), center_traj_point_tf.getOrigin().getY(), temp_yaw);
        ROS_INFO("center_pose_loctrajframe_: %.4f / %.4f / %.4f",center_pose_loctrajframe_[0], center_pose_loctrajframe_[1],center_pose_loctrajframe_[2]);
        ROS_INFO("center_vel_robotframe_: %.4f / %.4f / %.4f",center_vel_robotframe_[0], center_vel_robotframe_[1],center_vel_robotframe_[2]);

        }*/
    }  
    
    
    return traj_free;
                    
}


bool ManeuverPlanner::searchTrajectorySingleManeuver(const tf::Stamped<tf::Pose>& start_tf, 
                               const tf::Stamped<tf::Pose>& goal_tf, const tf::Stamped<tf::Pose>& refpoint_tf_robot_coord, 
                               std::vector<geometry_msgs::PoseStamped>& plan)
{
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
    start_tf.getBasis().getEulerYPR(goal_yaw, temp_pitch, temp_roll);             
    
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
    
    
    double min_radius;     
    double signed_max_turning_radius_refp;    // Maximum steering radius using the eference point
    double xlocal_intersection_refp;          // Intersection of target in local x coodinates using the reference point
    int maneuver_type_refp = determineManeuverType(refpoint_goal_tf_refstart_coord,  signed_max_turning_radius_refp, xlocal_intersection_refp);
    double signed_max_turning_radius_center;    // Maximum steering radius using the center of the robot
    double xlocal_intersection_center;          // Intersection of target in local x coodinates
    int maneuver_type_center;
            
    if (maneuver_type_refp == ManeuverPlanner::MANEUVER_LEFT || maneuver_type_refp == ManeuverPlanner::MANEUVER_RIGHT)
    {   // This only supports single maneuvers    
            
        if(signed_max_turning_radius_refp > 0.0)
            min_radius = std::abs(radius_search_.lin_search_min_);
        else
            min_radius = - std::abs(radius_search_.lin_search_min_);   
        
        radius_search_.resetLinearSearch(min_radius, signed_max_turning_radius_refp);
        while( radius_search_.linearSearch(signed_turning_radius_refp) & !maneuver_traj_succesful){
            ROS_INFO(" signed_turning_radius_refp: %.3f", signed_turning_radius_refp);
            curve_type = computeSingleManeuverParameters(refpoint_goal_tf_refstart_coord, signed_turning_radius_refp,  xlocal_intersection_refp, dist_before_steering_refp, dist_after_steering_refp);
            // ROS_INFO("Curve parameters: %.3f / %.3f / %.3f",dist_before_steering_refp, dist_after_steering_refp, signed_turning_radius_refp);
            plan.clear();
            if( curve_type!= ManeuverPlanner::CURVE_NONE ) // curve possible, generate
                maneuver_traj_succesful = generateTrajectorySingleManeuver(start_tf, refpoint_goal_tf_refstart_coord, goal_tf,  refpoint_tf_robot_coord, dist_before_steering_refp, dist_after_steering_refp, signed_turning_radius_refp, plan);
         }                        
        
    }
    
    
    if( maneuver_traj_succesful == false ) 
    {
        // Come back to reference point at the center
        ROS_INFO("Setting reference point back to center of rotation");
        tf::Stamped<tf::Pose> center_tf_robot_coord;
        center_tf_robot_coord.frame_id_ = "/wholerobot_link";
        center_tf_robot_coord.stamp_ = goal_tf.stamp_;
        temp_quat.setRPY(0.0,0.0,0.0);
        temp_vector3 = tf::Vector3(0.0,0.0, 0.0);
        center_tf_robot_coord.setData(tf::Transform(temp_quat,temp_vector3));
        
        maneuver_type_center = determineManeuverType(goal_tf_start_coord,  signed_max_turning_radius_center, xlocal_intersection_center);
        
        if (maneuver_type_center == ManeuverPlanner::MANEUVER_LEFT || maneuver_type_center == ManeuverPlanner::MANEUVER_RIGHT)
        {
            if(signed_max_turning_radius_center > 0.0)
                min_radius = std::abs(radius_search_.lin_search_min_);
            else
                min_radius = - std::abs(radius_search_.lin_search_min_);     
            
            radius_search_.resetLinearSearch(min_radius, signed_max_turning_radius_center);
            while( radius_search_.linearSearch(signed_turning_radius_center) & !maneuver_traj_succesful){
                ROS_INFO(" signed_turning_radius_center: %.3f", signed_turning_radius_center);
                computeSingleManeuverParameters(goal_tf_start_coord, signed_turning_radius_center,  xlocal_intersection_center, dist_before_steering_center, dist_after_steering_center);
                plan.clear();
                maneuver_traj_succesful = generateTrajectorySingleManeuver(start_tf, goal_tf_start_coord, goal_tf,  center_tf_robot_coord, dist_before_steering_center, dist_after_steering_center, signed_turning_radius_center, plan);
            }
        }
    }
    
    return maneuver_traj_succesful;

}

// bool ManeuverPlanner::searchTrajectoryLeftRightManeuver(const tf::Stamped<tf::Pose>& start_tf, 
//                                const tf::Stamped<tf::Pose>& goal_tf, const tf::Stamped<tf::Pose>& refpoint_tf_robot_coord, 
//                                std::vector<geometry_msgs::PoseStamped>& plan)
// {
//     tf::Stamped<tf::Pose> midway_start_goal_tf;
//     double x_mid = start_tf.getOrigin().getX() + start_tf.getOrigin().getX() + 
//     midway_start_goal_tf.setData();
//     
// }

bool ManeuverPlanner::linePlanner(const geometry_msgs::PoseStamped& start,
                               const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
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
    if(scale < 1.0)
    {
        ROS_WARN("Line planner could not find a free path for this goal");        
    }
    return traj_free;

}

bool ManeuverPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                               const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
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


//     temp_quat.setRPY(0.0,0.0,1.5708);
//     temp_vector3 = tf::Vector3(1.19, -1.36, 0.0);
//     start_tf.setData(tf::Transform(temp_quat,temp_vector3));
//     temp_quat.setRPY(0.0,0.0,3.1416 );
//     temp_vector3 = tf::Vector3(-0.11, 1.19, 0.0);
//     goal_tf.setData(tf::Transform(temp_quat,temp_vector3));

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
            temp_vector3 = tf::Vector3(topRightCorner_[0], topRightCorner_[1], 0.0);
            refpoint_tf_robot_coord.setData(tf::Transform(temp_quat,temp_vector3));
            ROS_INFO("Left turn");
            maneuver_traj_succesful = searchTrajectorySingleManeuver(start_tf, goal_tf, refpoint_tf_robot_coord, plan);
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
            temp_vector3 = tf::Vector3(right_side_ref_point_[0], right_side_ref_point_[1], 0.0);
            refpoint_tf_robot_coord.setData(tf::Transform(temp_quat,temp_vector3));
            ROS_INFO("Right turn");
            maneuver_traj_succesful = searchTrajectorySingleManeuver(start_tf, goal_tf, refpoint_tf_robot_coord, plan);
            
            break;            
        case ManeuverPlanner::MANEUVER_LEFT_RIGHT :
            // Initially choose top left corner (trc) as reference to generate
            // trajectory. Suitable for when tlc is desired to keep parallel to the
            // left wall. 
            refpoint_tf_robot_coord.frame_id_ = "/wholerobot_link";
            refpoint_tf_robot_coord.stamp_ = goal_tf.stamp_;
            temp_quat.setRPY(0.0,0.0,0.0);
            temp_vector3 = tf::Vector3(topLeftCorner_[0], topLeftCorner_[1], 0.0);
            refpoint_tf_robot_coord.setData(tf::Transform(temp_quat,temp_vector3));
            ROS_INFO("Left turn");            
            
        default:
            break;
        }

    }
   
    
    if( maneuver_traj_succesful == false) // Maneuver planning failed. Attemp linear planner
    {
        ROS_WARN("No single left or right maneuver possible. Execute line planner");
        plan.clear();
        maneuver_traj_succesful = linePlanner(start, goal, plan);        
    }

    if(maneuver_traj_succesful == false){
        plan.clear();
    }
    return true;

}



};

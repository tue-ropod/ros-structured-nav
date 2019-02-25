/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
* Author: Cesar Lopez
*********************************************************************/
#ifndef MANEUVER_PLANNER_H_
#define MANEUVER_PLANNER_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include <maneuver_planner/parameter_generator.h>

#include <math.h>
#include <Eigen/Dense>

namespace maneuver_planner{
  /**
   * @class ManeuverPlanner
   * @brief Provides a simple global planner that will compute a valid goal point for the local planner by walking back along the vector between the robot and the user-specified goal point until a valid cost is found.
   */
  class ManeuverPlanner : public nav_core::BaseGlobalPlanner {
    public:
      /**
       * @brief  Constructor for the ManeuverPlanner
       */
      ManeuverPlanner();
      /**
       * @brief  Constructor for the ManeuverPlanner
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
      ManeuverPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Initialization function for the ManeuverPlanner
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);    
      bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan, double & dist_without_obstacles);
      bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan, double & dist_without_obstacles, bool uselinePlanner);    
      
      costmap_2d::Costmap2DROS* costmap_ros_;
    private:
      double step_size_, min_dist_from_robot_;
      costmap_2d::Costmap2D* costmap_;
      base_local_planner::WorldModel* world_model_; ///< @brief The world model that the controller will use
      
      // Rectangular robot points
      Eigen::Vector2d topRightCorner_;
      Eigen::Vector2d topLeftCorner_;
      Eigen::Vector2d bottomRightCorner_;
      Eigen::Vector2d bottomLeftCorner_;
      // Extra reference points
      Eigen::Vector2d left_side_ref_point_;
      Eigen::Vector2d right_side_ref_point_;
      // AUxiliary variables
      Eigen::Vector2d motion_refpoint_localtraj_;
      Eigen::Vector2d prev_motion_refpoint_localtraj_;
       
      Eigen::Vector3d center_pose_loctrajframe_;
            
      
      // Maneuver parameters
      double turning_radius_;
      bool last_goal_as_start_;
      
      // internal variables
      geometry_msgs::PoseStamped last_goal_;
      geometry_msgs::PoseStamped start_;
      bool valid_last_goal_;
      
      // ParameterSearch
      parameter_generator::ParameterGenerator radius_search_;
      parameter_generator::ParameterGenerator midway_scale_lr_search_;     
      parameter_generator::ParameterGenerator midway_side_ovt_search_;  // search distance on the side when overtaking
      
      
      /**
       * @brief  Checks the legality of the robot footprint at a position and orientation using the world model
       * @param x_i The x position of the robot 
       * @param y_i The y position of the robot 
       * @param theta_i The orientation of the robot
       * @return 
       */
      double footprintCost(double x_i, double y_i, double theta_i);
      
      void rotate2D(const tf::Stamped<tf::Pose> &pose_tf_in, const double theta, tf::Stamped<tf::Pose> &pose_tf_out);
      void translate2D(const tf::Stamped<tf::Pose> &pose_tf_in, const tf::Vector3 &vector3_translation, tf::Stamped<tf::Pose> &pose_tf_out);      
      bool computeSingleManeuverParameters(const tf::Stamped<tf::Pose>& pose_target, const double& signed_turning_radius, const double& x_intersection, double &dist_before_steering, double &dist_after_steering);
      int  determineManeuverType(const tf::Stamped<tf::Pose>& pose_target,  double &signed_max_turning_radius, double& x_intersection);      
      
      bool searchTrajectoryCompoundLeftRightManeuver(const tf::Stamped<tf::Pose>& start_tf, const tf::Stamped<tf::Pose>& goal_tf, 
                                             const tf::Stamped<tf::Pose>& refpoint_tf_robot_coord, std::vector<geometry_msgs::PoseStamped>& plan, double & dist_without_obstacles);
      
      bool searchTrajectoryOvertakeManeuver(const tf::Stamped<tf::Pose>& start_tf, const tf::Stamped<tf::Pose>& goal_tf, 
                                                        const tf::Stamped<tf::Pose>& refpoint_tf_robot_coord, std::vector<geometry_msgs::PoseStamped>& plan, double & dist_without_obstacles);
      bool searchTrajectoryLeftRightManeuver(const tf::Stamped<tf::Pose>& start_tf, const tf::Stamped<tf::Pose>& goal_tf, 
                                             const tf::Stamped<tf::Pose>& refpoint_tf_robot_coord, std::vector<geometry_msgs::PoseStamped>& plan, double & dist_without_obstacles);
      
      bool searchTrajectorySingleManeuver(const tf::Stamped<tf::Pose>& start_tf, const tf::Stamped<tf::Pose>& goal_tf, 
                                           std::vector< tf::Stamped<tf::Pose> >& refpoint_tf_robot_coord_vec, std::vector<geometry_msgs::PoseStamped>& plan, double & dist_without_obstacles);
      bool checkFootprintTransformLocalToGlobal(const tf::Stamped<tf::Pose>& start_tf, 
                                const tf::Stamped<tf::Pose>& goal_tf, const tf::Stamped<tf::Pose>& refpoint_tf_robot_coord, 
                                const std::vector<tf::Pose> &local_plan_refp, std::vector<geometry_msgs::PoseStamped>& plan, double &dist_without_obstacles );
      
      
      void generateDublinTrajectory(const double& dist_before_steering_refp, const double& dist_after_steering_refp, 
                               const double& signed_turning_radius_refp, const double& theta_refp_goal, std::vector<tf::Pose> &local_plan_refp);
      bool linePlanner(const geometry_msgs::PoseStamped& start,
                               const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan, double &dist_without_obstacles);
      bool makePlanUntilPossible(const geometry_msgs::PoseStamped& start,
                               const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan, double & dist_without_obstacles);
      
      void removeLastPoints( std::vector<geometry_msgs::PoseStamped>& plan, double & distToRemove);
      
      
      enum curveType
      {
	CURVE_NONE,
	CURVE_LEFT_TOP_RIGHT_CORNER,
	CURVE_LEFT_CENTER_POINT,
	CURVE_LEFT_POINT,
	CURVE_RIGHT_TOP_LEFT_CORNER,
	CURVE_RIGHT_CENTER_POINT,
	CURVE_RIGHT_POINT,	
      };

      enum maneuverType
      {
        MANEUVER_NONE,
        MANEUVER_LEFT,
        MANEUVER_RIGHT,
        MANEUVER_LEFT_RIGHT,
        MANEUVER_RIGHT_LEFT,
        MANEUVER_STRAIGHT_OTHERWISE_OVERTAKE,
      };	
      
      static const double MIN_X_DIST_OVERTAKE = 2.0;
      static const double MIN_THETA_OVERTAKE = 10.0/180.0*M_PI;
      
      
      bool initialized_;
      
      double maxDistanceBeforeObstacle_;
      double maxDistanceBeforeReplanning_;
  };
};  
#endif

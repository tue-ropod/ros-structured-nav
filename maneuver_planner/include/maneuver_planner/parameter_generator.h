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
#ifndef PARAMETER_GENERATOR_H_
#define PARAMETER_GENERATOR_H_

#include <math.h>
#include <ros/ros.h>


namespace parameter_generator{
  /**
   * @class ParameterGenerator
   * @brief Provides a simple global planner that will compute a valid goal point for the local planner by walking back along the vector between the robot and the user-specified goal point until a valid cost is found.
   */
  class ParameterGenerator{
    public:
      /**
       * @brief  Constructor for the ParameterGenerator
       */
      ParameterGenerator();
      
      
      ParameterGenerator(double lin_search_min, double lin_search_max, double lin_search_absmax, double lin_search_step_size_min, int lin_search_max_steps);
    
            /**
       * @brief  resets Linear Search variables
       * @param lin_search_min
       * @param lin_search_max
       * @return 
       */
      void resetLinearSearch(double lin_search_min, double lin_search_max);      
      bool linearSearch(double &lin_search_curr);
      
      void resetMidSearch(double lin_search_min, double lin_search_max);      
      bool midSearch(double &lin_search_curr);      
      
      double lin_search_min_;    // Minimum while searching for turning radius
      double lin_search_max_;    // Maximum while searching for turning radius

    private:

      
      // LinearSearch
      double lin_search_absmax_; // Absolute maximum while searching for turning radius      
      double lin_search_curr_;    // Current value
      int lin_search_max_steps_;     // Maximum number of steps
      double lin_search_step_size_min_; // Minimum step size
      double lin_search_step_size_;     // step size
      int lin_search_sign_;
      int lin_search_cnt_;  
      


  };
};  
#endif

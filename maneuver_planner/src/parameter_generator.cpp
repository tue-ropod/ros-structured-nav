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
#include <maneuver_planner/parameter_generator.h>

namespace parameter_generator {

    
ParameterGenerator::ParameterGenerator()
{
    lin_search_min_ = 0.0;
    lin_search_max_ = 0.0;
    lin_search_absmax_ = 0.0;     
    lin_search_step_size_min_ = 0.1;
    lin_search_max_steps_ = 1;
    lin_search_sign_ = 1;
    lin_search_cnt_ = 0;
}    
    
ParameterGenerator::ParameterGenerator(double lin_search_min, double lin_search_max, double lin_search_absmax, double lin_search_step_size_min, int lin_search_max_steps)
{
    // Linear search    
    lin_search_min_ = lin_search_min;
    lin_search_max_ = lin_search_max;
    lin_search_absmax_ = lin_search_absmax;     
    lin_search_step_size_min_ = lin_search_step_size_min;
    lin_search_max_steps_ = lin_search_max_steps;
    lin_search_sign_ = 1;
    lin_search_cnt_ = 0;

}



void ParameterGenerator::resetLinearSearch(double lin_search_min, double lin_search_max)
{
      lin_search_max_ = std::min(lin_search_absmax_,lin_search_max);
      lin_search_min_ = lin_search_min;
      lin_search_step_size_ = std::max( lin_search_step_size_min_,  std::abs( (lin_search_max_-lin_search_min_)/lin_search_max_steps_ ) );
      lin_search_curr_ = (lin_search_max_+lin_search_min_)/2.0;
      lin_search_sign_ = 1;
      lin_search_cnt_ = 0;
}

bool ParameterGenerator::linearSearch(double &lin_search_curr)
{    
    double span         = std::abs(lin_search_max_-lin_search_min_);    
    double current_span    = 2.0*lin_search_cnt_*lin_search_step_size_;
    if( current_span > span )
    {
        lin_search_curr = lin_search_curr_;
        return false;
    }
    else
    {
        lin_search_curr_ = (lin_search_max_+lin_search_min_)/2.0 + lin_search_sign_*lin_search_cnt_*lin_search_step_size_;
        if (lin_search_sign_ > 0)
            lin_search_cnt_++;
        lin_search_sign_ *= -1;
        lin_search_curr = lin_search_curr_;
        return true;
    }
}



};

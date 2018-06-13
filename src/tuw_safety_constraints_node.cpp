/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2018 by Florian Beck <florian.beck@tuwien.ac.at>        *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 ***************************************************************************/

#include "tuw_safety_constraints/tuw_safety_constraints_node.h"
#include <tuw_nav_msgs/BaseConstr.h>

using namespace tuw;

SafetyConstraintsNode::SafetyConstraintsNode(ros::NodeHandle& nh) : nh_(nh), nh_private_("~")
{
  nh_private_.param("stop_button_topics", stop_button_topics_, std::vector<std::string>(1, "stop_button"));

  for(size_t i = 0; i < stop_button_topics_.size(); i++)
  {
    stop_button_sub_vec_.emplace_back(nh_.subscribe<std_msgs::Bool>(stop_button_topics_[i], 1, std::bind(&SafetyConstraintsNode::stopCallback, this, std::placeholders::_1, stop_button_topics_[i])));
    
    stop_button_values_.emplace(stop_button_topics_[i], false);
  }
  
  laser_sub_ = nh_.subscribe("scan", 1, &SafetyConstraintsNode::laserSensorCallback, this);
  
  // initialize defaults
  omg_wh_max_ = 5.0;
  obstacle_dist_max_ = 1.0;
  
  stopped_ = false;
  obstacle_clear_ = true;
  
  constr_pub_ = nh_.advertise<tuw_nav_msgs::BaseConstr>("base_constraints", 1);
  
  reconfigureFnc_ = std::bind(&SafetyConstraintsNode::callbackParameters, this, std::placeholders::_1, std::placeholders::_2);
  reconfigureServer_.setCallback(reconfigureFnc_);
  
  namespace_ = nh_.getNamespace();
}

void SafetyConstraintsNode::restore()
{
  omg_wh_max_ = omg_wh_max_backup_;
}

void SafetyConstraintsNode::backup()
{
  omg_wh_max_backup_ = omg_wh_max_;
}

void SafetyConstraintsNode::publishConstraints()
{
  tuw_nav_msgs::BaseConstr constraints;
  constraints.omg_wh_max = omg_wh_max_;
  
  constr_pub_.publish(constraints);
}

void SafetyConstraintsNode::stop(bool request_stop)
{
  bool buttons_clear = std::all_of(stop_button_values_.begin(), stop_button_values_.end(), [](auto m)
                                  {
                                    return !m.second;
                                  });
  
  bool all_clear = buttons_clear && obstacle_clear_;
  
  if(request_stop && !stopped_)
  {
    ROS_DEBUG("Stopping robot motion");
      
    backup();
    
    omg_wh_max_ = 0;
    
    stopped_ = true;
  }
  else if(request_stop && stopped_)
  {
    ROS_DEBUG("Already stopped, not stopping again");
  }
  else if(all_clear && stopped_)
  {
    ROS_DEBUG("Resuming robot motion");
      
    restore();
   
    stopped_ = false;
  }
  else if(!all_clear && stopped_)
  {
    ROS_DEBUG("Robot will resume when all buttons are cleared and no obstacles detected");
  }
  else
  {
    // do not publish anything
    return;
  }
  
  publishConstraints();
}

void SafetyConstraintsNode::stopCallback(const std_msgs::Bool::ConstPtr& msg, const std::string& topic)
{
  stop_button_values_[topic] = msg->data;
  
  stop(msg->data);
}

void SafetyConstraintsNode::callbackParameters(tuw_safety_constraints::tuw_safety_constraintsConfig& config, uint32_t level)
{
  omg_wh_max_ = config.omg_wh_max;
  obstacle_dist_max_ = config.obstacle_dist_max;
}

void SafetyConstraintsNode::laserSensorCallback(const sensor_msgs::LaserScan::ConstPtr& _scan)
{
  laser_readings_.clear();
  
  tf::StampedTransform tfsI2r;
  try
  {
    tf_listener_.lookupTransform(tf::resolve(namespace_, "base_link"), _scan->header.frame_id, _scan->header.stamp, tfsI2r);
  }
  catch (tf::TransformException ex)
  {
    try
    {
      tf_listener_.lookupTransform(tf::resolve(namespace_, "base_link"), _scan->header.frame_id, ros::Time(0), tfsI2r);
    }
    catch (tf::TransformException ex)
    {
      std::cerr << ex.what() << std::endl;
      ;
      return;
    }
  }
  double roll = 0, pitch = 0, yaw = 0;
  tfsI2r.getBasis().getRPY(roll, pitch, yaw);
  Pose2D laserX0(tfsI2r.getOrigin().getX(), tfsI2r.getOrigin().getY(), yaw);
  tuw::Tf2D laserTf(laserX0.tf());

  size_t nr = (_scan->angle_max - _scan->angle_min) / _scan->angle_increment;
  
  double front_min = _scan->range_max;
  
  for (size_t i = 0; i < nr; i++)
  {
    if (std::isinf(_scan->ranges[i]) == 0)
    {
      double a = _scan->angle_min + (_scan->angle_increment * i);
      double d = _scan->ranges[i];
      
      tf::Vector3 v(cos(a) * d, sin(a) * d, 0);
      tf::Vector3 vr = tfsI2r * v;
      laser_readings_.emplace_back(Point2D(vr.getX(), vr.getY()));
      
      if(i < 2 * nr / 3 && i > nr / 3)
      {
        if(vr.length() < front_min)
        {
          front_min = vr.length();
        }
      }
    }
  }
  
  // check if any object is near the robot
  // stop robot if necessary
  obstacle_clear_ = front_min > obstacle_dist_max_;
  stop(!obstacle_clear_);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "safety_constraints");
  ros::NodeHandle n;

  SafetyConstraintsNode safetyConstraintsNode(n);
  
  ros::spin();  
  
  return 0;
}

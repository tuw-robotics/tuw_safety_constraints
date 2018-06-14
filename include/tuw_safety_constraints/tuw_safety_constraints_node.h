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

#ifndef TUW_SAFETY_CONSTRAINTS_NODE_H
#define TUW_SAFETY_CONSTRAINTS_NODE_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tuw_safety_constraints/tuw_safety_constraintsConfig.h>
#include <dynamic_reconfigure/server.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tuw_geometry/tuw_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tuw_airskin_msgs/AirskinPressures.h>

namespace tuw
{
  
class SafetyConstraintsNode
{
public:
  SafetyConstraintsNode(ros::NodeHandle& nh);
 
  void stopCallback(const std_msgs::Bool::ConstPtr& msg, const std::string& topic);
  void callbackParameters(tuw_safety_constraints::tuw_safety_constraintsConfig& config, uint32_t level);
  void laserSensorCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  void airskinCallback(const tuw_airskin_msgs::AirskinPressures::ConstPtr& pressures);
private:
  ros::NodeHandle nh_;  
  ros::NodeHandle nh_private_;
  ros::Publisher constr_pub_;  
  ros::Subscriber laser_sub_;
  ros::Subscriber airskin_sub_;
  std::vector<ros::Subscriber> stop_button_sub_vec_;
  std::vector<std::string> stop_button_topics_;
  
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
  
  std::string namespace_;
  
  bool path_following_;
  bool stopped_;
  bool obstacle_clear_;
  bool airskin_clear_;
  std::map<std::string, bool> stop_button_values_;
  
  double omg_wh_max_;
  double omg_wh_;
  double obstacle_dist_max_;
  double wheel_radius_;
  
  std::vector<Pose2D> laser_readings_;
  
  void stop(bool request_stop);
  void publishConstraints(double omg_wh);
  
  dynamic_reconfigure::Server<tuw_safety_constraints::tuw_safety_constraintsConfig> reconfigureServer_;
  dynamic_reconfigure::Server<tuw_safety_constraints::tuw_safety_constraintsConfig>::CallbackType reconfigureFnc_;
};

}

#endif // TUW_SAFETY_CONSTRAINTS_NODE_H

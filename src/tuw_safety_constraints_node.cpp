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
  
  // initialize defaults
  omg_wh_max_ = 5.0;
  
  stopped_ = false;
  
  constr_pub_ = nh_.advertise<tuw_nav_msgs::BaseConstr>("base_constraints", 1);
  
  reconfigureFnc_ = std::bind(&SafetyConstraintsNode::callbackParameters, this, std::placeholders::_1, std::placeholders::_2);
  reconfigureServer_.setCallback(reconfigureFnc_);
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

void SafetyConstraintsNode::stopCallback(const std_msgs::Bool::ConstPtr& msg, const std::string& topic)
{
  stop_button_values_[topic] = msg->data;
  
  bool all_clear = std::all_of(stop_button_values_.begin(), stop_button_values_.end(), [](auto m)
                                  {
                                    return !m.second;
                                  });
  
  if(msg->data && !stopped_)
  {
    ROS_INFO("Stopping robot motion");
      
    backup();
    
    omg_wh_max_ = 0;
    
    stopped_ = true;
  }
  else if(msg->data && stopped_)
  {
    ROS_INFO("Already stopped, not stopping again");
  }
  else if(all_clear && stopped_)
  {
    ROS_INFO("Resuming robot motion");
      
    restore();
   
    stopped_ = false;
  }
  else
  {
    ROS_INFO("Robot will resume when all buttons are cleared");
  }
  
  publishConstraints();
}

void SafetyConstraintsNode::callbackParameters(tuw_safety_constraints::tuw_safety_constraintsConfig& config, uint32_t level)
{
  omg_wh_max_ = config.omg_wh_max;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "safety_constraints");
  ros::NodeHandle n;

  SafetyConstraintsNode safetyConstraintsNode(n);
  
  ros::spin();  
  
  return 0;
}

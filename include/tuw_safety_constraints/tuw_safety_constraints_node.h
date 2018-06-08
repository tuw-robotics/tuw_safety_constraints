#ifndef TUW_SAFETY_CONSTRAINTS_NODE_H
#define TUW_SAFETY_CONSTRAINTS_NODE_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tuw_safety_constraints/tuw_safety_constraintsConfig.h>
#include <dynamic_reconfigure/server.h>

namespace tuw
{
  
class SafetyConstraintsNode
{
public:
  SafetyConstraintsNode(ros::NodeHandle& nh);
 
  void stopCallback(const std_msgs::Bool::ConstPtr& msg, const std::string& topic);
  void callbackParameters(tuw_safety_constraints::tuw_safety_constraintsConfig& config, uint32_t level);
private:
  ros::NodeHandle nh_;  
  ros::NodeHandle nh_private_;
  ros::Publisher constr_pub_;  
  std::vector<ros::Subscriber> stop_button_sub_vec_;
  
  std::vector<std::string> stop_button_topics_;
  
  bool stopped_;
  std::map<std::string, bool> stop_button_values_;
  
  double omg_wh_max_;
  double omg_wh_max_backup_;
  
  void backup();
  void restore();
  
  void publishConstraints();
  
  dynamic_reconfigure::Server<tuw_safety_constraints::tuw_safety_constraintsConfig> reconfigureServer_;
  dynamic_reconfigure::Server<tuw_safety_constraints::tuw_safety_constraintsConfig>::CallbackType reconfigureFnc_;
};

}

#endif // TUW_SAFETY_CONSTRAINTS_NODE_H

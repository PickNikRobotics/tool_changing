#pragma once

#include <moveit/move_group/move_group_capability.h>
#include <tool_changing_capability_msgs/srv/change_end_effector.hpp>
#include <tool_changing_capability_msgs/srv/get_current_end_effector.hpp>

namespace move_group
{
class ToolChangingCapability : public MoveGroupCapability
{
public:
  ToolChangingCapability();

  void initialize() override;

  void changeEndEffectorCB(const tool_changing_capability_msgs::srv::ChangeEndEffector::Request::SharedPtr req,
                           tool_changing_capability_msgs::srv::ChangeEndEffector::Response::SharedPtr res);
  void getCurrentEndEffectorCB(const tool_changing_capability_msgs::srv::GetCurrentEndEffector::Request::SharedPtr req,
                               tool_changing_capability_msgs::srv::GetCurrentEndEffector::Response::SharedPtr res);

private:
  collision_detection::AllowedCollisionMatrix cached_acm_;
  rclcpp::Service<tool_changing_capability_msgs::srv::ChangeEndEffector>::SharedPtr change_eef_service_;
  rclcpp::Service<tool_changing_capability_msgs::srv::GetCurrentEndEffector>::SharedPtr get_current_eef_service_;
  moveit::core::RobotModelConstPtr robot_model_;
  std::string current_end_effector;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  bool enableEndEffector(const std::string& end_effector);
};
}  // namespace move_group

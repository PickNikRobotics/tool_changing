// Copyright 2021 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PickNik Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "tool_changing_capability.hpp"

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include "constants.hpp"

namespace move_group
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit.move_group.tool_changing_capability");

ToolChangingCapability::ToolChangingCapability() : MoveGroupCapability("ToolChangingCapability")
{
}

void ToolChangingCapability::initialize()
{
  robot_model_ = context_->moveit_cpp_->getRobotModel();
  if (robot_model_->getEndEffectors().empty())
  {
    throw std::invalid_argument("Robot model doesn't contain any end-effector -- make sure to load the "
                                "robot_description_semantic parameter or add end_effector tag to the srdf file");
  }
  const rclcpp::Parameter initial_end_effector_param =
      context_->moveit_cpp_->getNode()->get_parameter("initial_end_effector");
  if (initial_end_effector_param.get_type() == rclcpp::PARAMETER_NOT_SET)
  {
    current_end_effector = robot_model_->getEndEffectors().at(0)->getName();
    RCLCPP_WARN_STREAM(LOGGER, "Parameter `initial_end_effector` not set -- setting the initial end-effector to `"
                                   << current_end_effector << "`");
  }
  else
    current_end_effector = initial_end_effector_param.as_string();
  RCLCPP_INFO_STREAM(LOGGER, "Enabling `" << current_end_effector << "` as an active end-effector");
  cached_acm_ =
      planning_scene_monitor::LockedPlanningSceneRO(context_->planning_scene_monitor_)->getAllowedCollisionMatrix();
  enableEndEffector(current_end_effector);
  callback_group_ =
      context_->moveit_cpp_->getNode()->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  change_eef_service_ =
      context_->moveit_cpp_->getNode()->create_service<tool_changing_capability_msgs::srv::ChangeEndEffector>(
          CHANGE_END_EFFECTOR_SERVICE_NAME,
          std::bind(&ToolChangingCapability::changeEndEffectorCB, this, std::placeholders::_1, std::placeholders::_2),
          rmw_qos_profile_services_default, callback_group_);
  get_current_eef_service_ =
      context_->moveit_cpp_->getNode()->create_service<tool_changing_capability_msgs::srv::GetCurrentEndEffector>(
          GET_CURRENT_END_EFFECTOR_SERVICE_NAME,
          std::bind(&ToolChangingCapability::getCurrentEndEffectorCB, this, std::placeholders::_1,
                    std::placeholders::_2),
          rmw_qos_profile_services_default, callback_group_);
}

void ToolChangingCapability::changeEndEffectorCB(
    const tool_changing_capability_msgs::srv::ChangeEndEffector::Request::SharedPtr req,
    tool_changing_capability_msgs::srv::ChangeEndEffector::Response::SharedPtr res)
{
  res->success = enableEndEffector(req->end_effector_name);
}

bool ToolChangingCapability::enableEndEffector(const std::string& end_effector)
{
  if (!robot_model_->hasEndEffector(end_effector))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Unknown end-effector `" << end_effector << "`");
    return false;
  }
  RCLCPP_INFO_STREAM(LOGGER, "Setting current end-effector to `" << end_effector << "`");
  planning_scene_monitor::LockedPlanningSceneRW scene(context_->planning_scene_monitor_);
  auto& acm = scene->getAllowedCollisionMatrixNonConst();
  acm = cached_acm_;
  for (const auto& eef_jmg : robot_model_->getEndEffectors())
  {
    if (eef_jmg->getName() == end_effector)
      continue;
    RCLCPP_INFO_STREAM(LOGGER, "Disabling end-effector `" << eef_jmg->getName() << "`");
    const auto& eef_links = robot_model_->getEndEffector(eef_jmg->getName())->getLinkModelNames();
    std::for_each(eef_links.cbegin(), eef_links.cend(),
                  [&robot_links = robot_model_->getLinkModelNames(), &acm](const auto& eef_link) {
                    std::for_each(robot_links.cbegin(), robot_links.cend(), [&eef_link, &acm](const auto& robot_link) {
                      acm.setEntry(eef_link, robot_link, true);
                    });
                  });
  }
  context_->planning_scene_monitor_->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
  current_end_effector = end_effector;
  return true;
}

void ToolChangingCapability::getCurrentEndEffectorCB(
    const tool_changing_capability_msgs::srv::GetCurrentEndEffector::Request::SharedPtr /* req */,
    tool_changing_capability_msgs::srv::GetCurrentEndEffector::Response::SharedPtr res)
{
  res->end_effector_name = current_end_effector;
}
}  // namespace move_group

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(move_group::ToolChangingCapability, move_group::MoveGroupCapability)

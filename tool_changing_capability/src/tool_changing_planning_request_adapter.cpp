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

#include <class_loader/class_loader.hpp>

#include <tool_changing_capability_msgs/srv/get_current_end_effector.hpp>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_request_adapter/planning_request_adapter.h>

#include "constants.hpp"

namespace default_planner_request_adapters
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_ros.tool_changing_adapter");

class ToolChangingAdapter : public planning_request_adapter::PlanningRequestAdapter
{
public:
  ToolChangingAdapter() : planning_request_adapter::PlanningRequestAdapter()
  {
  }

  void initialize(const rclcpp::Node::SharedPtr& node, const std::string& /* parameter_namespace */) override
  {
    callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    client_ = node->create_client<tool_changing_capability_msgs::srv::GetCurrentEndEffector>(
        GET_CURRENT_END_EFFECTOR_SERVICE_NAME, rmw_qos_profile_services_default, callback_group_);
  }

  std::string getDescription() const override
  {
    return "Check if the planning motion request's group doesn't correspond to a disabled end-effector";
  }

  bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const planning_interface::MotionPlanRequest& req, planning_interface::MotionPlanResponse& res,
                    std::vector<std::size_t>& /*added_path_index*/) const override
  {
    RCLCPP_DEBUG(LOGGER, "Running '%s'", getDescription().c_str());
    if (!client_->service_is_ready())
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Service `" << client_->get_service_name() << "` isn't available");
      return false;
    }
    auto request = std::make_shared<tool_changing_capability_msgs::srv::GetCurrentEndEffector::Request>();
    auto future = client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(1)) != std::future_status::ready)
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Didn't get a response from `" << client_->get_service_name() << "` within 1s");
      return false;
    }
    const auto& end_effectors = planning_scene->getRobotModel()->getEndEffectors();
    const auto& req_jmg = planning_scene->getRobotModel()->getJointModelGroup(req.group_name);
    std::vector<const moveit::core::JointModelGroup*> disabled_end_effectors;
    std::remove_copy_if(end_effectors.cbegin(), end_effectors.cend(), std::back_inserter(disabled_end_effectors),
                        [&current_end_effector =
                             future.get()->end_effector_name](const moveit::core::JointModelGroup* const jmg) {
                          return jmg->getName() == current_end_effector;
                        });

    if (std::any_of(disabled_end_effectors.cbegin(), disabled_end_effectors.cend(),
                    [&req_jmg](const moveit::core::JointModelGroup* const disabled_end_effector) {
                      return disabled_end_effector->getName() == req_jmg->getName() ||
                             req_jmg->isSubgroup(disabled_end_effector->getName());
                    }))
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Got planning request for a group that contains/is a disabled end-effector `"
                                      << req.group_name << "` -- currently enabled end-effector `"
                                      << future.get()->end_effector_name << "`");
      return false;
    }

    return planner(planning_scene, req, res);
  }

private:
  rclcpp::Client<tool_changing_capability_msgs::srv::GetCurrentEndEffector>::SharedPtr client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
};

}  // namespace default_planner_request_adapters

CLASS_LOADER_REGISTER_CLASS(default_planner_request_adapters::ToolChangingAdapter,
                            planning_request_adapter::PlanningRequestAdapter);

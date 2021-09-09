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

#pragma once

#include <tool_changing_capability_msgs/srv/change_end_effector.hpp>
#include <tool_changing_capability_msgs/srv/get_current_end_effector.hpp>

#include <moveit/move_group/move_group_capability.h>

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
  // This's used to cache the original disable collisions pairs in the loaded SRDF file
  collision_detection::AllowedCollisionMatrix cached_acm_;
  rclcpp::Service<tool_changing_capability_msgs::srv::ChangeEndEffector>::SharedPtr change_eef_service_;
  rclcpp::Service<tool_changing_capability_msgs::srv::GetCurrentEndEffector>::SharedPtr get_current_eef_service_;
  moveit::core::RobotModelConstPtr robot_model_;
  std::string current_end_effector;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  // Enable the input end-effector and disable the other remaining ones
  // If the end-effector name is an empty string disable all end-effectors
  bool enableEndEffector(const std::string& end_effector);
};
}  // namespace move_group

// Copyright (c) 2023 Franka Emika GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "franka/robot_state.h"
#include "franka_semantic_components/franka_state.hpp"
#include "gmock/gmock.h"

// implementing and friending so we can access member variables
class FrankaStateTestFriend : public franka_semantic_components::FrankaState {
  FRIEND_TEST(FrankaStateTest, validate_state_names_and_size);
  FRIEND_TEST(FrankaStateTest,
              given_franka_semantic_state_initialized_when_message_returned_expect_correct_values);
  FRIEND_TEST(FrankaStateTest, robot_state_ptr_uncasted_correctly);

 public:
  // Use generation of interface names
  explicit FrankaStateTestFriend(const std::string& name)
      : franka_semantic_components::FrankaState(name) {}

  virtual ~FrankaStateTestFriend() = default;
};

class FrankaStateTest : public ::testing::Test {
 public:
  void SetUp();

  void TearDown();

 protected:
  const size_t size_ = 1;
  const std::string robot_name_ = "panda";
  const std::string franka_state_interface_name_ = "franka_state";
  franka::RobotState robot_state_;
  franka::RobotState* robot_state_address_ = &robot_state_;

  std::array<double, 7> joint_angles_ = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  std::array<double, 7> joint_velocities_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  franka::RobotMode robot_mode_ = franka::RobotMode::kUserStopped;
  franka_msgs::msg::FrankaState franka_robot_state_msg_;

  std::unique_ptr<FrankaStateTestFriend> franka_state_friend_;

  std::vector<std::string> full_interface_names_;
};

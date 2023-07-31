#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/gripper_state.h>
#include <control_msgs/action/gripper_command.hpp>
#include <franka_msgs/action/grasp.hpp>
#include <franka_msgs/action/homing.hpp>
#include <franka_msgs/action/move.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "franka_msgs/action/runtime_pos.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "motion_generator.hpp"

namespace runtime_position_control {
class RuntimePosActionServer : public rclcpp::Node {
 public:
  using RuntimePos = franka_msgs::action::RuntimePos;
  using GoalHandleRuntimePos = rclcpp_action::ServerGoalHandle<RuntimePos>;

  explicit RuntimePosActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("runtime_pos_action_server", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<RuntimePos>(
        this, "runtimepos", std::bind(&RuntimePosActionServer::handle_goal, this, _1, _2),
        std::bind(&RuntimePosActionServer::handle_cancel, this, _1),
        std::bind(&RuntimePosActionServer::handle_accepted, this, _1));
  }

 private:
  rclcpp_action::Server<RuntimePos>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const RuntimePos::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal with joint1 position: %d",
                goal->goal_pos.positions[0]);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleRuntimePos> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleRuntimePos> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&RuntimePosActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleRuntimePos> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<RuntimePos::Feedback>();
    auto& current_jnt_pos = feedback->current_pos;
    auto result = std::make_shared<RuntimePos::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};  // class RuntimePosActionServer

}  // namespace runtime_position_control

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::RuntimePosActionServer)

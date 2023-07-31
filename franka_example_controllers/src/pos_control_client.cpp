#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

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

// #include "motion_generator.hpp"

namespace runtime_position_control {
class RuntimePosActionClient : public rclcpp::Node {
 public:
  using RuntimePos = franka_msgs::action::RuntimePos;
  using GoalHandleRuntimePos = rclcpp_action::ClientGoalHandle<RuntimePos>;

  explicit RuntimePosActionClient(const rclcpp::NodeOptions& options)
      : Node("runtime_pos_action_client", options) {
    this->client_ptr_ = rclcpp_action::create_client<RuntimePos>(this, "runtimepos");

    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                           std::bind(&RuntimePosActionClient::send_goal, this));
  }

  void send_goal() {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = RuntimePos::Goal();
    goal_msg.goal_pos.positions.resize(7);
    goal_msg.goal_pos.positions[0] = 0.0;
    goal_msg.goal_pos.positions[1] = 0.0;
    goal_msg.goal_pos.positions[2] = 0.0;
    goal_msg.goal_pos.positions[3] = 0.0;
    goal_msg.goal_pos.positions[4] = 0.0;
    goal_msg.goal_pos.positions[5] = 0.0;
    goal_msg.goal_pos.positions[6] = 0.0;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<RuntimePos>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&RuntimePosActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&RuntimePosActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&RuntimePosActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

 private:
  rclcpp_action::Client<RuntimePos>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(const GoalHandleRuntimePos::SharedPtr& goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(GoalHandleRuntimePos::SharedPtr,
                         const std::shared_ptr<const RuntimePos::Feedback> feedback) {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    auto current_jnt_pos = feedback->current_pos;
    ss << current_jnt_pos.positions[0] << ", " << current_jnt_pos.positions[1] << ", "
       << current_jnt_pos.positions[2] << ", " << current_jnt_pos.positions[3] << ", "
       << current_jnt_pos.positions[4] << ", " << current_jnt_pos.positions[5] << ", "
       << current_jnt_pos.positions[6] << " ";
    // for (auto number : feedback->partial_sequence) {
    //   ss << number << " ";
    // }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleRuntimePos::WrappedResult& result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    for (auto waypoint : result.result->traj_plan) {
      ss << waypoint.positions[0] << ", " << waypoint.positions[1] << ", " << waypoint.positions[2]
         << ", " << waypoint.positions[3] << ", " << waypoint.positions[4] << ", "
         << waypoint.positions[5] << ", " << waypoint.positions[6] << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};  // class RuntimePosActionClient

}  // namespace runtime_position_control

RCLCPP_COMPONENTS_REGISTER_NODE(runtime_position_control::RuntimePosActionClient)
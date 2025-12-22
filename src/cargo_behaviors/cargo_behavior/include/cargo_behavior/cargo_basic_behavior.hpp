#pragma once

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "cargo_core/aerial_platform.hpp"
#include "cargo_core/node.hpp"
#include "cargo_msgs/msg/thrust.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace cargo {
template <class MessageT> class BasicBehavior : public cargo::Node {
  public:
    using GoalHandleAction = rclcpp_action::ServerGoalHandle<MessageT>;

    explicit BasicBehavior(const std::string &name) : Node(name) {
        this->action_server_ = rclcpp_action::create_server<MessageT>(
            this, this->generate_global_name(name),
            std::bind(&BasicBehavior::handleGoal, this, std::placeholders::_1,
                      std::placeholders::_2),
            std::bind(&BasicBehavior::handleCancel, this,
                      std::placeholders::_1),
            std::bind(&BasicBehavior::handleAccepted, this,
                      std::placeholders::_1));
    }

  public:
    virtual rclcpp_action::GoalResponse
    onAccepted(const std::shared_ptr<const typename MessageT::Goal> goal) = 0;
    virtual rclcpp_action::CancelResponse
    onCancel(const std::shared_ptr<GoalHandleAction> goal_handle) = 0;
    virtual void onExecute(const std::shared_ptr<GoalHandleAction>
                               goal_handle) = 0; // return true when finished

  private:
    rclcpp_action::GoalResponse
    handleGoal(const rclcpp_action::GoalUUID &uuid,
               std::shared_ptr<const typename MessageT::Goal> goal) {
        RCLCPP_DEBUG(this->get_logger(), "Received goal request with UUID: %d",
                     uuid);
        return onAccepted(goal);
    }

    rclcpp_action::CancelResponse
    handleCancel(const std::shared_ptr<GoalHandleAction> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Request to cancel goal received");
        return onCancel(goal_handle);
    }

    // TODO(CVAR): explore the use of Timers instead std::thread
    void handleAccepted(const std::shared_ptr<GoalHandleAction> goal_handle) {
        // this needs to return quickly to avoid blocking the executor, so spin
        // up a new thread
        if (execution_thread_.joinable()) {
            execution_thread_.join();
        }

        execution_thread_ = std::thread(
            std::bind(&BasicBehavior::onExecute, this, std::placeholders::_1),
            goal_handle);
    }

  private:
    std::thread execution_thread_;
    typename rclcpp_action::Server<MessageT>::SharedPtr action_server_;
}; // BasicBehavior class

} // namespace cargo

#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <pluginlib/class_loader.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>

#include "cargo_behavior/behavior_server.hpp"
#include "cargo_core/names/actions.hpp"
#include "cargo_core/names/services.hpp"
#include "cargo_core/names/topics.hpp"
#include "cargo_core/synchronous_service_client.hpp"
#include "cargo_core/utils/tf_utils.hpp"
#include "cargo_msgs/action/takeoff.hpp"
#include "cargo_msgs/msg/platform_info.hpp"
#include "cargo_msgs/srv/set_platform_state_machine_event.hpp"
#include "takeoff_base.hpp"

class TakeoffBehavior
    : public cargo_behavior::BehaviorServer<cargo_msgs::action::Takeoff> {
  public:
    using GoalHandleTakeoff =
        rclcpp_action::ServerGoalHandle<cargo_msgs::action::Takeoff>;
    using PSME = cargo_msgs::msg::PlatformStateMachineEvent;

    explicit TakeoffBehavior(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    ~TakeoffBehavior();

    void state_callback(
        const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg);

    bool sendEventFSME(const int8_t _event);

    bool
    process_goal(std::shared_ptr<const cargo_msgs::action::Takeoff::Goal> goal,
                 cargo_msgs::action::Takeoff::Goal &new_goal);

    bool on_activate(
        std::shared_ptr<const cargo_msgs::action::Takeoff::Goal> goal) override;
    bool on_modify(
        std::shared_ptr<const cargo_msgs::action::Takeoff::Goal> goal) override;
    bool on_deactivate(const std::shared_ptr<std::string> &message) override;
    bool on_pause(const std::shared_ptr<std::string> &message) override;
    bool on_resume(const std::shared_ptr<std::string> &message) override;
    cargo_behavior::ExecutionStatus
    on_run(const std::shared_ptr<const cargo_msgs::action::Takeoff::Goal> &goal,
           std::shared_ptr<cargo_msgs::action::Takeoff::Feedback> &feedback_msg,
           std::shared_ptr<cargo_msgs::action::Takeoff::Result> &result_msg)
        override;
    void
    on_execution_end(const cargo_behavior::ExecutionStatus &state) override;

  private:
    std::string base_link_frame_id_;
    std::shared_ptr<pluginlib::ClassLoader<takeoff_base::TakeoffBase>> loader_;
    std::shared_ptr<takeoff_base::TakeoffBase> takeoff_plugin_;
    std::shared_ptr<cargo::tf::TfHandler> tf_handler_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
        twist_sub_;
    cargo::SynchronousServiceClient<
        cargo_msgs::srv::SetPlatformStateMachineEvent>::SharedPtr platform_cli_;
};

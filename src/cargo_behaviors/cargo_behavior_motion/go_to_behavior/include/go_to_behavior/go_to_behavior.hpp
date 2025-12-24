#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <pluginlib/class_loader.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>

#include "cargo_behavior/behavior_server.hpp"
#include "cargo_core/names/actions.hpp"
#include "cargo_core/names/topics.hpp"
#include "cargo_core/utils/tf_utils.hpp"
#include "cargo_msgs/action/go_to_waypoint.hpp"
#include "cargo_msgs/msg/platform_info.hpp"
#include "go_to_base.hpp"

class GoToBehavior
    : public cargo_behavior::BehaviorServer<cargo_msgs::action::GoToWaypoint> {
  public:
    using GoalHandleGoTo =
        rclcpp_action::ServerGoalHandle<cargo_msgs::action::GoToWaypoint>;

    explicit GoToBehavior(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~GoToBehavior();

    void state_callback(
        const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg);

    void
    platform_info_callback(const cargo_msgs::msg::PlatformInfo::SharedPtr msg);

    bool process_goal(
        std::shared_ptr<const cargo_msgs::action::GoToWaypoint::Goal> goal,
        cargo_msgs::action::GoToWaypoint::Goal &new_goal);

    bool on_activate(
        std::shared_ptr<const cargo_msgs::action::GoToWaypoint::Goal> goal)
        override;
    bool on_modify(std::shared_ptr<const cargo_msgs::action::GoToWaypoint::Goal>
                       goal) override;
    bool on_deactivate(const std::shared_ptr<std::string> &message) override;
    bool on_pause(const std::shared_ptr<std::string> &message) override;
    bool on_resume(const std::shared_ptr<std::string> &message) override;
    cargo_behavior::ExecutionStatus
    on_run(const std::shared_ptr<const cargo_msgs::action::GoToWaypoint::Goal>
               &goal,
           std::shared_ptr<cargo_msgs::action::GoToWaypoint::Feedback>
               &feedback_msg,
           std::shared_ptr<cargo_msgs::action::GoToWaypoint::Result>
               &result_msg) override;
    void
    on_execution_end(const cargo_behavior::ExecutionStatus &state) override;

  private:
    std::string base_link_frame_id_;
    std::shared_ptr<pluginlib::ClassLoader<go_to_base::GoToBase>> loader_;
    std::shared_ptr<go_to_base::GoToBase> go_to_plugin_;
    std::shared_ptr<cargo::tf::TfHandler> tf_handler_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
        twist_sub_;
    rclcpp::Subscription<cargo_msgs::msg::PlatformInfo>::SharedPtr
        platform_info_sub_;
};

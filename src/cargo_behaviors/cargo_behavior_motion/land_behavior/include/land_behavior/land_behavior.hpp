#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <pluginlib/class_loader.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <string>

#include "cargo_behavior/behavior_server.hpp"
#include "cargo_core/names/actions.hpp"
#include "cargo_core/names/services.hpp"
#include "cargo_core/names/topics.hpp"
#include "cargo_core/synchronous_service_client.hpp"
#include "cargo_core/utils/tf_utils.hpp"
#include "cargo_msgs/action/land.hpp"
#include "cargo_msgs/msg/platform_info.hpp"
#include "cargo_msgs/srv/set_platform_state_machine_event.hpp"
#include "land_base.hpp"
#include "rclcpp/rclcpp.hpp"

class LandBehavior
    : public cargo_behavior::BehaviorServer<cargo_msgs::action::Land> {
  public:
    using GoalHandleLand =
        rclcpp_action::ServerGoalHandle<cargo_msgs::action::Land>;
    using PSME = cargo_msgs::msg::PlatformStateMachineEvent;

    explicit LandBehavior(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    ~LandBehavior();

    void state_callback(
        const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg);

    bool sendEventFSME(const int8_t _event);

    bool sendDisarm();

    bool
    process_goal(std::shared_ptr<const cargo_msgs::action::Land::Goal> goal,
                 cargo_msgs::action::Land::Goal &new_goal);

    bool on_activate(
        std::shared_ptr<const cargo_msgs::action::Land::Goal> goal) override;
    bool on_modify(
        std::shared_ptr<const cargo_msgs::action::Land::Goal> goal) override;
    bool on_deactivate(const std::shared_ptr<std::string> &message) override;
    bool on_pause(const std::shared_ptr<std::string> &message) override;
    bool on_resume(const std::shared_ptr<std::string> &message) override;
    cargo_behavior::ExecutionStatus on_run(
        const std::shared_ptr<const cargo_msgs::action::Land::Goal> &goal,
        std::shared_ptr<cargo_msgs::action::Land::Feedback> &feedback_msg,
        std::shared_ptr<cargo_msgs::action::Land::Result> &result_msg) override;
    void
    on_execution_end(const cargo_behavior::ExecutionStatus &state) override;

  private:
    std::string base_link_frame_id_;
    std::shared_ptr<pluginlib::ClassLoader<land_base::LandBase>> loader_;
    std::shared_ptr<land_base::LandBase> land_plugin_;
    std::shared_ptr<cargo::tf::TfHandler> tf_handler_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
        twist_sub_;
    cargo::SynchronousServiceClient<
        cargo_msgs::srv::SetPlatformStateMachineEvent>::SharedPtr
        platform_land_cli_;
    cargo::SynchronousServiceClient<std_srvs::srv::SetBool>::SharedPtr
        platform_disarm_cli_;
};

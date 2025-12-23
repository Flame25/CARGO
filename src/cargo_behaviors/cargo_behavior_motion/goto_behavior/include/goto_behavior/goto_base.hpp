#pragma once

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>

#include "cargo_behavior/behavior_server.hpp"
#include "cargo_core/utils/frame_utils.hpp"
#include "cargo_core/utils/tf_utils.hpp"
#include "cargo_motion_reference_handlers/hover_motion.hpp"
#include "cargo_msgs/action/go_to_waypoint.hpp"
#include "cargo_msgs/msg/platform_info.hpp"
#include "cargo_msgs/msg/platform_status.hpp"

namespace go_to_base {

struct go_to_plugin_params {
    double go_to_speed = 0.0;
    double go_to_threshold = 0.0;
};

class GoToBase {
  public:
    using GoalHandleGoTo =
        rclcpp_action::ServerGoalHandle<cargo_msgs::action::GoToWaypoint>;

    GoToBase() {}
    virtual ~GoToBase() {}

    void initialize(cargo::Node *node_ptr,
                    std::shared_ptr<cargo::tf::TfHandler> tf_handler,
                    go_to_plugin_params &params) {
        node_ptr_ = node_ptr;
        tf_handler = tf_handler;
        params_ = params;
        hover_motion_handler_ =
            std::make_shared<cargo::motionReferenceHandlers::HoverMotion>(
                node_ptr_);
        this->ownInit();
    }

    void state_callback(geometry_msgs::msg::PoseStamped &pose_msg,
                        geometry_msgs::msg::TwistStamped &twist_msg) {
        actual_pose_ = pose_msg;

        feedback_.actual_speed =
            Eigen::Vector3d(twist_msg.twist.linear.x, twist_msg.twist.linear.y,
                            twist_msg.twist.linear.z)
                .norm();

        feedback_.actual_distance_to_goal =
            (Eigen::Vector3d(actual_pose_.pose.position.x,
                             actual_pose_.pose.position.y,
                             actual_pose_.pose.position.z) -
             Eigen::Vector3d(goal_.target_pose.point.x,
                             goal_.target_pose.point.y,
                             goal_.target_pose.point.z))
                .norm();

        localization_flag_ = true;
        return;
    }

    void
    platform_info_callback(const cargo_msgs::msg::PlatformInfo::SharedPtr msg) {
        platform_state_ = msg->status.state;
        return;
    }

    bool on_activate(
        std::shared_ptr<const cargo_msgs::action::GoToWaypoint::Goal> goal) {
        cargo_msgs::action::GoToWaypoint::Goal goal_candidate = *goal;
        if (!processGoal(goal_candidate)) {
            return false;
        }

        if (own_activate(goal_candidate)) {
            goal_ = goal_candidate;
            return true;
        }
        return false;
    }

    bool on_modify(
        std::shared_ptr<const cargo_msgs::action::GoToWaypoint::Goal> goal) {
        cargo_msgs::action::GoToWaypoint::Goal goal_candidate = *goal;
        if (!processGoal(goal_candidate)) {
            return false;
        }

        if (own_modify(goal_candidate)) {
            goal_ = goal_candidate;
            return true;
        }
        return false;
    }

    inline bool on_deactivate(const std::shared_ptr<std::string> &message) {
        return own_deactivate(message);
    }

    inline bool on_pause(const std::shared_ptr<std::string> &message) {
        return own_pause(message);
    }

    inline bool on_resume(const std::shared_ptr<std::string> &message) {
        return own_resume(message);
    }

    void on_execution_end(const cargo_behavior::ExecutionStatus &state) {
        localization_flag_ = false;
        own_execution_end(state);
        return;
    }

    cargo_behavior::ExecutionStatus on_run(
        const std::shared_ptr<const cargo_msgs::action::GoToWaypoint::Goal>
            goal,
        std::shared_ptr<cargo_msgs::action::GoToWaypoint::Feedback>
            &feedback_msg,
        std::shared_ptr<cargo_msgs::action::GoToWaypoint::Result> &result_msg) {
        cargo_behavior::ExecutionStatus status = own_run();

        feedback_msg =
            std::make_shared<cargo_msgs::action::GoToWaypoint::Feedback>(
                feedback_);
        result_msg =
            std::make_shared<cargo_msgs::action::GoToWaypoint::Result>(result_);
        return status;
    }

  private:
    bool processGoal(cargo_msgs::action::GoToWaypoint::Goal &_goal) {
        if (platform_state_ != cargo_msgs::msg::PlatformStatus::FLYING) {
            RCLCPP_ERROR(node_ptr_->get_logger(),
                         "Behavior reject, platform is not flying");
            return false;
        }

        if (!localization_flag_) {
            RCLCPP_ERROR(node_ptr_->get_logger(),
                         "Behavior reject, there is no localization");
            return false;
        }

        return true;
    }

  private:
    std::shared_ptr<cargo::motionReferenceHandlers::HoverMotion>
        hover_motion_handler_ = nullptr;

    /* Interface with plugin */

  protected:
    virtual void ownInit() {}

    virtual bool own_activate(cargo_msgs::action::GoToWaypoint::Goal &goal) = 0;

    virtual bool own_modify(cargo_msgs::action::GoToWaypoint::Goal &goal) {
        RCLCPP_INFO(node_ptr_->get_logger(),
                    "Go to can not be modified, not implemented");
        return false;
    }

    virtual bool
    own_deactivate(const std::shared_ptr<std::string> &message) = 0;

    virtual bool own_pause(const std::shared_ptr<std::string> &message) {
        RCLCPP_INFO(
            node_ptr_->get_logger(),
            "Go to can not be paused, not implemented, try to cancel it");
        return false;
    }

    virtual bool own_resume(const std::shared_ptr<std::string> &message) {
        RCLCPP_INFO(node_ptr_->get_logger(),
                    "Go to can not be resumed, not implemented");
        return false;
    }

    virtual void
    own_execution_end(const cargo_behavior::ExecutionStatus &state) = 0;
    virtual cargo_behavior::ExecutionStatus own_run() = 0;

    inline void sendHover() {
        hover_motion_handler_->sendHover();
        return;
    }

  protected:
    cargo::Node *node_ptr_;
    std::shared_ptr<cargo::tf::TfHandler> tf_handler = nullptr;

    cargo_msgs::action::GoToWaypoint::Goal goal_;
    cargo_msgs::action::GoToWaypoint::Feedback feedback_;
    cargo_msgs::action::GoToWaypoint::Result result_;

    int platform_state_;
    go_to_plugin_params params_;
    geometry_msgs::msg::PoseStamped actual_pose_;
    bool localization_flag_;
}; // class GoToBase
} // namespace go_to_base

#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>

#include "cargo_behavior/behavior_server.hpp"
#include "cargo_core/utils/tf_utils.hpp"
#include "cargo_motion_reference_handlers/hover_motion.hpp"
#include "cargo_msgs/action/takeoff.hpp"
#include "cargo_msgs/msg/platform_info.hpp"
#include "cargo_msgs/msg/platform_status.hpp"

namespace takeoff_base {

struct takeoff_plugin_params {
    double takeoff_height = 0.0;
    double takeoff_speed = 0.0;
    double takeoff_threshold = 0.0;
};

class TakeoffBase {
  public:
    using GoalHandleTakeoff =
        rclcpp_action::ServerGoalHandle<cargo_msgs::action::Takeoff>;

    TakeoffBase() {}
    virtual ~TakeoffBase() {}

    void initialize(cargo::Node *node_ptr,
                    std::shared_ptr<cargo::tf::TfHandler> tf_handler,
                    takeoff_plugin_params &params) {
        node_ptr_ = node_ptr;
        tf_handler = tf_handler;
        params_ = params;
        hover_motion_handler_ =
            std::make_shared<cargo::motionReferenceHandlers::HoverMotion>(
                node_ptr_);
        this->ownInit();
    }

    virtual void state_callback(geometry_msgs::msg::PoseStamped &pose_msg,
                                geometry_msgs::msg::TwistStamped &twist_msg) {
        actual_pose_ = pose_msg;

        feedback_.actual_takeoff_height = actual_pose_.pose.position.z;
        feedback_.actual_takeoff_speed = twist_msg.twist.linear.z;

        localization_flag_ = true;
        return;
    }

    bool
    on_activate(std::shared_ptr<const cargo_msgs::action::Takeoff::Goal> goal) {
        cargo_msgs::action::Takeoff::Goal goal_candidate = *goal;
        if (!processGoal(goal_candidate)) {
            return false;
        }

        if (own_activate(goal_candidate)) {
            goal_ = goal_candidate;
            return true;
        }
        return false;
    }

    bool
    on_modify(std::shared_ptr<const cargo_msgs::action::Takeoff::Goal> goal) {
        cargo_msgs::action::Takeoff::Goal goal_candidate = *goal;
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

    cargo_behavior::ExecutionStatus
    on_run(const std::shared_ptr<const cargo_msgs::action::Takeoff::Goal> goal,
           std::shared_ptr<cargo_msgs::action::Takeoff::Feedback> &feedback_msg,
           std::shared_ptr<cargo_msgs::action::Takeoff::Result> &result_msg) {
        cargo_behavior::ExecutionStatus status = own_run();

        feedback_msg =
            std::make_shared<cargo_msgs::action::Takeoff::Feedback>(feedback_);
        result_msg =
            std::make_shared<cargo_msgs::action::Takeoff::Result>(result_);
        return status;
    }

  private:
    bool processGoal(cargo_msgs::action::Takeoff::Goal &_goal) {
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

    virtual bool own_activate(cargo_msgs::action::Takeoff::Goal &goal) = 0;

    virtual bool own_modify(cargo_msgs::action::Takeoff::Goal &goal) {
        RCLCPP_INFO(node_ptr_->get_logger(),
                    "Takeoff can not be modified, not implemented");
        return false;
    }

    virtual bool
    own_deactivate(const std::shared_ptr<std::string> &message) = 0;

    virtual bool own_pause(const std::shared_ptr<std::string> &message) {
        RCLCPP_INFO(
            node_ptr_->get_logger(),
            "Takeoff can not be paused, not implemented, try to cancel it");
        return false;
    }

    virtual bool own_resume(const std::shared_ptr<std::string> &message) {
        RCLCPP_INFO(node_ptr_->get_logger(),
                    "Takeoff can not be resumed, not implemented");
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

    cargo_msgs::action::Takeoff::Goal goal_;
    cargo_msgs::action::Takeoff::Feedback feedback_;
    cargo_msgs::action::Takeoff::Result result_;

    takeoff_plugin_params params_;
    geometry_msgs::msg::PoseStamped actual_pose_;
    bool localization_flag_;
}; // class TakeoffBase
} // namespace takeoff_base

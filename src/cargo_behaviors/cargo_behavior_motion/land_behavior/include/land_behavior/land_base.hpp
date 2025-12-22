#pragma once

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>

#include "cargo_behavior/behavior_server.hpp"
#include "cargo_core/names/actions.hpp"
#include "cargo_core/names/topics.hpp"
#include "cargo_core/node.hpp"
#include "cargo_core/utils/frame_utils.hpp"
#include "cargo_core/utils/tf_utils.hpp"
#include "cargo_motion_reference_handlers/hover_motion.hpp"
#include "cargo_msgs/action/land.hpp"
#include "cargo_msgs/msg/platform_info.hpp"
#include "cargo_msgs/msg/platform_status.hpp"

namespace land_base {

struct land_plugin_params {
    double land_speed = 0.0;
};

class LandBase {
  public:
    using GoalHandleLand =
        rclcpp_action::ServerGoalHandle<cargo_msgs::action::Land>;

    LandBase() {}
    virtual ~LandBase() {}

    void initialize(cargo::Node *node_ptr,
                    std::shared_ptr<cargo::tf::TfHandler> tf_handler,
                    land_plugin_params &params) {
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

        feedback_.actual_land_height = actual_pose_.pose.position.z;
        feedback_.actual_land_speed = twist_msg.twist.linear.z;

        localization_flag_ = true;
        return;
    }

    bool
    on_activate(std::shared_ptr<const cargo_msgs::action::Land::Goal> goal) {
        cargo_msgs::action::Land::Goal goal_candidate = *goal;
        if (!processGoal(goal_candidate)) {
            return false;
        }

        if (own_activate(goal_candidate)) {
            goal_ = goal_candidate;
            return true;
        }
        return false;
    }

    bool on_modify(std::shared_ptr<const cargo_msgs::action::Land::Goal> goal) {
        cargo_msgs::action::Land::Goal goal_candidate = *goal;
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
    on_run(const std::shared_ptr<const cargo_msgs::action::Land::Goal> goal,
           std::shared_ptr<cargo_msgs::action::Land::Feedback> &feedback_msg,
           std::shared_ptr<cargo_msgs::action::Land::Result> &result_msg) {
        cargo_behavior::ExecutionStatus status = own_run();

        feedback_msg =
            std::make_shared<cargo_msgs::action::Land::Feedback>(feedback_);
        result_msg =
            std::make_shared<cargo_msgs::action::Land::Result>(result_);
        return status;
    }

  private:
    bool processGoal(cargo_msgs::action::Land::Goal &_goal) {
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

    virtual bool own_activate(cargo_msgs::action::Land::Goal &goal) = 0;

    virtual bool own_modify(cargo_msgs::action::Land::Goal &goal) {
        RCLCPP_INFO(node_ptr_->get_logger(),
                    "Land can not be modified, not implemented");
        return false;
    }

    virtual bool
    own_deactivate(const std::shared_ptr<std::string> &message) = 0;

    virtual bool own_pause(const std::shared_ptr<std::string> &message) {
        RCLCPP_INFO(
            node_ptr_->get_logger(),
            "Land can not be paused, not implemented, try to cancel it");
        return false;
    }

    virtual bool own_resume(const std::shared_ptr<std::string> &message) {
        RCLCPP_INFO(node_ptr_->get_logger(),
                    "Land can not be resumed, not implemented");
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

    cargo_msgs::action::Land::Goal goal_;
    cargo_msgs::action::Land::Feedback feedback_;
    cargo_msgs::action::Land::Result result_;

    land_plugin_params params_;
    geometry_msgs::msg::PoseStamped actual_pose_;
    bool localization_flag_ = false;
}; // class LandBase
} // namespace land_base

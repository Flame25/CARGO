#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "cargo_behavior/behavior_server.hpp"
#include "cargo_core/names/actions.hpp"
#include "cargo_core/names/topics.hpp"
#include "cargo_core/synchronous_service_client.hpp"
#include "cargo_core/utils/tf_utils.hpp"
#include "cargo_motion_reference_handlers/position_motion.hpp"
#include "cargo_msgs/action/follow_reference.hpp"
#include "cargo_msgs/msg/platform_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class DroneSwarm {
  public:
    DroneSwarm(cargo::Node *node_ptr, std::string drone_id,
               geometry_msgs::msg::Pose init_pose,
               rclcpp::CallbackGroup::SharedPtr cbk_group);
    ~DroneSwarm() {}

  public:
    std::string drone_id_;
    geometry_msgs::msg::Pose init_pose_;
    geometry_msgs::msg::PoseStamped drone_pose_;
    geometry_msgs::msg::TransformStamped transform_;

  private:
    cargo::Node *node_ptr_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfstatic_broadcaster_;
    std::string base_link_frame_id_;
    std::string parent_frame_id;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        drone_pose_sub_;
    rclcpp_action::Client<cargo_msgs::action::FollowReference>::SharedPtr
        follow_reference_client_ = nullptr;
    cargo::SynchronousServiceClient<std_srvs::srv::Trigger>::SharedPtr
        follow_reference_stop_client_ = nullptr;
    rclcpp::CallbackGroup::SharedPtr cbk_group_;
    std::shared_ptr<const cargo_msgs::action::FollowReference::Feedback>
        follow_reference_feedback_;
    float max_speed_;

  private:
    /**
     * @brief Callback to update the current pose of the drone
     * @param _pose_msg The curent pose of the drone
     */
    void dronePoseCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr _pose_msg);

    /**
     * @brief Callback to update the feedback of the follow reference action
     * @param goal_handle The goal handle of the follow reference action
     * @param feedback The feedback of the follow reference action
     */
    void follow_reference_feedback_cbk(
        rclcpp_action::ClientGoalHandle<
            cargo_msgs::action::FollowReference>::SharedPtr goal_handle,
        const std::shared_ptr<
            const cargo_msgs::action::FollowReference::Feedback>
            feedback);

  public:
    /**
     * @brief Initialize the follow refrence
     * @return
     * std::shared_ptr<rclcpp_action::ClientGoalHandle<cargo_msgs::action::FollowReference>>
     * Future of the FollowReference behavior
     */
    std::shared_ptr<
        rclcpp_action::ClientGoalHandle<cargo_msgs::action::FollowReference>>
    initFollowReference();

    /**
     * @brief Send request to stop following the refernce in the swarm
     * @return bool Response of the service
     */
    bool stopFollowReference();

    /**
     * @brief Check if the drones are in their reference position
     * @return bool Return true if the drones are less than 0.3 meters from the
     * reference
     */
    bool checkPosition();

    /**
     * @brief Update the static tf of the drones
     * @param pose The new reference pose
     */
    bool updateStaticTf(geometry_msgs::msg::Pose pose);
};

#pragma once

#include <algorithm>
#include <chrono>
#include <drone_swarm.hpp>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "cargo_behavior/behavior_server.hpp"
#include "cargo_core/names/actions.hpp"
#include "cargo_core/utils/frame_utils.hpp"
#include "cargo_msgs/action/swarm_flocking.hpp"
#include "cargo_msgs/msg/pose_with_id_array.hpp"
#include "cargo_msgs/srv/modify_swarm.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class SwarmFlockingBehavior
    : public cargo_behavior::BehaviorServer<cargo_msgs::action::SwarmFlocking> {
  public:
    SwarmFlockingBehavior();
    ~SwarmFlockingBehavior() {}

    std::vector<std::shared_ptr<
        rclcpp_action::ClientGoalHandle<cargo_msgs::action::FollowReference>>>
        goal_future_handles_;

  private:
    cargo_msgs::action::SwarmFlocking::Goal goal_;
    cargo_msgs::action::SwarmFlocking::Result result_;
    cargo_msgs::action::SwarmFlocking::Feedback feedback_;
    std::shared_ptr<rclcpp::Service<
        cargo_msgs::action::SwarmFlocking::Impl::SendGoalService>>
        modify_srv_ = nullptr;

    rclcpp::CallbackGroup::SharedPtr cbk_group_;
    std::unordered_map<std::string, std::shared_ptr<DroneSwarm>> drones_;

    std::unique_ptr<tf2_ros::StaticTransformBroadcaster>
        tfstatic_swarm_broadcaster_;
    std::shared_ptr<geometry_msgs::msg::TransformStamped> transform_;
    std::string swarm_base_link_frame_id_;

    // Suscriber to the dynamic swarm formation
    rclcpp::Subscription<cargo_msgs::msg::PoseWithIDArray>::SharedPtr
        dynamic_swarm_formation_;

    // Tf to pause the behavior
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  public:
    bool on_activate(
        std::shared_ptr<const cargo_msgs::action::SwarmFlocking::Goal> goal)
        override;
    bool on_modify(
        std::shared_ptr<const cargo_msgs::action::SwarmFlocking::Goal> goal)
        override;
    cargo_behavior::ExecutionStatus
    on_run(const std::shared_ptr<const cargo_msgs::action::SwarmFlocking::Goal>
               &goal,
           std::shared_ptr<cargo_msgs::action::SwarmFlocking::Feedback>
               &feedback_msg,
           std::shared_ptr<cargo_msgs::action::SwarmFlocking::Result>
               &result_msg) override;
    bool on_deactivate(const std::shared_ptr<std::string> &message) override;
    bool on_pause(const std::shared_ptr<std::string> &message) override;
    bool on_resume(const std::shared_ptr<std::string> &message) override;
    void
    on_execution_end(const cargo_behavior::ExecutionStatus &state) override;

  private:
    /**
     * @brief Set up the virtual centroid of the swarm with offset in the
     * desired frame
     * @param virtual_centroid The virtual centroid of the swarm in the desired
     * frame
     * @return bool Return true if the frame is not empty
     */
    bool setUpVirtualCentroid(
        const geometry_msgs::msg::PoseStamped &virtual_centroid);

    /**
     * @brief Set the drones reference in the swarm
     * @param centroid The centroid of the swarm
     * @param drones_namespace The namespaced of the drones
     * @param formation The position of the drones in the swarm
     * @return bool Always true at the end of the function
     */
    bool
    setUpDronesFormation(geometry_msgs::msg::PoseStamped centroid,
                         std::vector<std::string> drones_namespace,
                         std::vector<cargo_msgs::msg::PoseWithID> formation);

    /**
     * @brief Active the followReference of the drones and check if the drones
     * are ready to execute the action
     * @return bool Return true if the drones are ready to execute
     */
    bool initDroneReferences();

    /**
     * @brief Check the followReference status of the drones
     * @param goal_future_handles The goal future handles of the drones
     * @return cargo_behavior::ExecutionStatus Return the status of the
     * monitoring
     */
    cargo_behavior::ExecutionStatus monitoring(
        const std::vector<std::shared_ptr<rclcpp_action::ClientGoalHandle<
            cargo_msgs::action::FollowReference>>>
            goal_future_handles);

    /**
     * @brief Callback to update the refrences of the drones inside the swarm
     * @param new_formation The new formation of the swarm
     */
    void dynamicSwarmFormationCallback(
        cargo_msgs::msg::PoseWithIDArray new_formation);

    /**
     * @brief Service to modify the virtual_centroid, add or detach a new drone
     * reference within the swarm
     * @param request The request of the service
     * @param response The response of the service
     */
    void modifySwarmSrv(
        const std::shared_ptr<
            cargo_msgs::action::SwarmFlocking::Impl::SendGoalService::Request>
            request,
        const std::shared_ptr<
            cargo_msgs::action::SwarmFlocking::Impl::SendGoalService::Response>
            response);
};

#pragma once

#include <filesystem>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <cargo_core/names/topics.hpp>
#include <cargo_core/node.hpp>
#include <cargo_core/utils/frame_utils.hpp>
#include <cargo_core/utils/tf_utils.hpp>

#include "plugin_base.hpp"

namespace cargo_state_estimator {

class StateEstimator : public cargo::Node {
  public:
    explicit StateEstimator(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~StateEstimator() {}

  private:
    std::filesystem::path plugin_name_;
    std::shared_ptr<pluginlib::ClassLoader<
        cargo_state_estimator_plugin_base::StateEstimatorBase>>
        loader_;
    std::shared_ptr<cargo_state_estimator_plugin_base::StateEstimatorBase>
        plugin_ptr_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tfstatic_broadcaster_;
    std::shared_ptr<cargo::tf::TfHandler> tf_handler_;

  private:
    /**
     * @brief Modify the node options to allow undeclared parameters
     */
    static rclcpp::NodeOptions
    get_modified_options(const rclcpp::NodeOptions &options);
}; // class StateEstimator
} // namespace cargo_state_estimator

#pragma once

#include <string>

#include <cargo_msgs/msg/control_mode.hpp>
#include <cargo_msgs/msg/controller_info.hpp>
#include <cargo_msgs/msg/thrust.hpp>
#include <cargo_msgs/msg/trajectory_setpoints.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "cargo_core/node.hpp"

namespace cargo {
namespace motionReferenceHandlers {
class BasicMotionReferenceHandler {
  public:
    explicit BasicMotionReferenceHandler(cargo::Node *cargo_ptr,
                                         const std::string &ns = "");
    ~BasicMotionReferenceHandler();

  protected:
    cargo::Node *node_ptr_;
    std::string namespace_;

    cargo_msgs::msg::TrajectorySetpoints command_trajectory_msg_;
    geometry_msgs::msg::PoseStamped command_pose_msg_;
    geometry_msgs::msg::TwistStamped command_twist_msg_;
    cargo_msgs::msg::Thrust command_thrust_msg_;

    cargo_msgs::msg::ControlMode desired_control_mode_;

    bool sendThrustCommand();
    bool sendPoseCommand();
    bool sendTwistCommand();
    bool sendTrajectoryCommand();
    bool checkMode();

  private:
    static int number_of_instances_;

    static rclcpp::Subscription<cargo_msgs::msg::ControllerInfo>::SharedPtr
        controller_info_sub_;
    static cargo_msgs::msg::ControlMode current_mode_;

    static rclcpp::Publisher<cargo_msgs::msg::TrajectorySetpoints>::SharedPtr
        command_traj_pub_;
    static rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        command_pose_pub_;
    static rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
        command_twist_pub_;
    static rclcpp::Publisher<cargo_msgs::msg::Thrust>::SharedPtr
        command_thrust_pub_;

    bool setMode(const cargo_msgs::msg::ControlMode &mode);
};

} // namespace motionReferenceHandlers
} // namespace cargo

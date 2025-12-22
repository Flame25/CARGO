#include "cargo_motion_reference_handlers/speed_in_a_plane_motion.hpp"

namespace cargo {
namespace motionReferenceHandlers {
SpeedInAPlaneMotion::SpeedInAPlaneMotion(cargo::Node *node_ptr,
                                         const std::string &ns)
    : BasicMotionReferenceHandler(node_ptr, ns) {
    desired_control_mode_.yaw_mode = cargo_msgs::msg::ControlMode::NONE;
    desired_control_mode_.control_mode =
        cargo_msgs::msg::ControlMode::SPEED_IN_A_PLANE;
    desired_control_mode_.reference_frame =
        cargo_msgs::msg::ControlMode::UNDEFINED_FRAME;
}

bool SpeedInAPlaneMotion::ownSendCommand() {
    bool send_pose = sendPoseCommand();
    bool send_twist = sendTwistCommand();
    return send_pose && send_twist;
}

bool SpeedInAPlaneMotion::sendSpeedInAPlaneCommandWithYawSpeed(
    const std::string &frame_id_speed, const float vx, const float vy,
    const std::string &frame_id_pose, const float hz, const float yaw_speed) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id_pose;
    pose.pose.position.z = hz;

    geometry_msgs::msg::TwistStamped twist;
    twist.header.frame_id = frame_id_speed;
    twist.twist.linear.x = vx;
    twist.twist.linear.y = vy;
    twist.twist.angular.z = yaw_speed;
    return sendSpeedInAPlaneCommandWithYawSpeed(pose, twist);
}

bool SpeedInAPlaneMotion::sendSpeedInAPlaneCommandWithYawSpeed(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::TwistStamped &twist) {
    if (pose.header.frame_id == "" || twist.header.frame_id == "") {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Frame id is empty");
        return false;
    }
    desired_control_mode_.yaw_mode = cargo_msgs::msg::ControlMode::YAW_SPEED;
    this->command_pose_msg_ = pose;
    this->command_twist_msg_ = twist;
    return this->ownSendCommand();
}

bool SpeedInAPlaneMotion::sendSpeedInAPlaneCommandWithYawAngle(
    const std::string &frame_id_speed, const float vx, const float vy,
    const std::string &frame_id_pose, const float hz, const float yaw_angle) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id_pose;
    pose.pose.position.z = hz;
    pose.pose.orientation =
        tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw_angle));

    geometry_msgs::msg::TwistStamped twist;
    twist.header.frame_id = frame_id_speed;
    twist.twist.linear.x = vx;
    twist.twist.linear.y = vy;
    return sendSpeedInAPlaneCommandWithYawAngle(pose, twist);
}

bool SpeedInAPlaneMotion::sendSpeedInAPlaneCommandWithYawAngle(
    const std::string &frame_id_speed, const float vx, const float vy,
    const std::string &frame_id_pose, const float hz,
    const geometry_msgs::msg::Quaternion &q) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id_pose;
    pose.pose.position.z = hz;
    pose.pose.orientation = q;

    geometry_msgs::msg::TwistStamped twist;
    twist.header.frame_id = frame_id_speed;
    twist.twist.linear.x = vx;
    twist.twist.linear.y = vy;
    return sendSpeedInAPlaneCommandWithYawAngle(pose, twist);
}

bool SpeedInAPlaneMotion::sendSpeedInAPlaneCommandWithYawAngle(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::TwistStamped &twist) {
    if (pose.header.frame_id == "" || twist.header.frame_id == "") {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Frame id is empty");
        return false;
    }
    desired_control_mode_.yaw_mode = cargo_msgs::msg::ControlMode::YAW_ANGLE;
    this->command_pose_msg_ = pose;
    this->command_twist_msg_ = twist;
    return this->ownSendCommand();
}

} // namespace motionReferenceHandlers
} // namespace cargo

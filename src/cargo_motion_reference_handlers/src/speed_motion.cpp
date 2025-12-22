#include "cargo_motion_reference_handlers/speed_motion.hpp"

namespace cargo {
namespace motionReferenceHandlers {
SpeedMotion::SpeedMotion(cargo::Node *node_ptr, const std::string &ns)
    : BasicMotionReferenceHandler(node_ptr, ns) {
    desired_control_mode_.yaw_mode = cargo_msgs::msg::ControlMode::NONE;
    desired_control_mode_.control_mode = cargo_msgs::msg::ControlMode::SPEED;
    desired_control_mode_.reference_frame =
        cargo_msgs::msg::ControlMode::UNDEFINED_FRAME;
}

bool SpeedMotion::ownSendCommand() {
    bool send_pose = sendPoseCommand();
    bool send_twist = sendTwistCommand();
    return send_pose && send_twist;
}

bool SpeedMotion::sendSpeedCommandWithYawAngle(
    const std::string &frame_id_speed, const float &vx, const float &vy,
    const float &vz, const std::string &frame_id_yaw, const float &yaw_angle) {
    return sendSpeedCommandWithYawAngle(
        frame_id_speed, vx, vy, vz, frame_id_yaw,
        tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw_angle)));
}

bool SpeedMotion::sendSpeedCommandWithYawAngle(
    const std::string &frame_id_speed, const float &vx, const float &vy,
    const float &vz, const std::string &frame_id_yaw,
    const geometry_msgs::msg::Quaternion &q) {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = frame_id_yaw;
    pose_msg.pose.orientation = q;

    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.frame_id = frame_id_speed;
    twist_msg.twist.linear.x = vx;
    twist_msg.twist.linear.y = vy;
    twist_msg.twist.linear.z = vz;

    rclcpp::Time stamp = node_ptr_->now();
    pose_msg.header.stamp = stamp;
    twist_msg.header.stamp = stamp;

    return sendSpeedCommandWithYawAngle(pose_msg, twist_msg);
}

bool SpeedMotion::sendSpeedCommandWithYawAngle(
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

bool SpeedMotion::sendSpeedCommandWithYawSpeed(const std::string &frame_id,
                                               const float &vx, const float &vy,
                                               const float &vz,
                                               const float &yaw_speed) {
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.frame_id = frame_id;
    twist_msg.twist.linear.x = vx;
    twist_msg.twist.linear.y = vy;
    twist_msg.twist.linear.z = vz;
    twist_msg.twist.angular.z = yaw_speed;

    rclcpp::Time stamp = node_ptr_->now();
    twist_msg.header.stamp = stamp;

    return sendSpeedCommandWithYawSpeed(twist_msg);
}

bool SpeedMotion::sendSpeedCommandWithYawSpeed(
    const geometry_msgs::msg::TwistStamped &twist) {
    if (twist.header.frame_id == "") {
        RCLCPP_ERROR(node_ptr_->get_logger(), "Frame id is empty");
        return false;
    }
    desired_control_mode_.yaw_mode = cargo_msgs::msg::ControlMode::YAW_SPEED;
    this->command_twist_msg_ = twist;

    return this->sendTwistCommand();
}

} // namespace motionReferenceHandlers
} // namespace cargo

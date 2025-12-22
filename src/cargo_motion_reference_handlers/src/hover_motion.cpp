#include "cargo_motion_reference_handlers/hover_motion.hpp"

namespace cargo {
namespace motionReferenceHandlers {
HoverMotion::HoverMotion(cargo::Node *node_ptr, const std::string &ns)
    : BasicMotionReferenceHandler(node_ptr, ns) {
    desired_control_mode_.yaw_mode = cargo_msgs::msg::ControlMode::NONE;
    desired_control_mode_.control_mode = cargo_msgs::msg::ControlMode::HOVER;
    desired_control_mode_.reference_frame =
        cargo_msgs::msg::ControlMode::UNDEFINED_FRAME;
}

bool HoverMotion::sendHover() {
    desired_control_mode_.yaw_mode = cargo_msgs::msg::ControlMode::NONE;
    desired_control_mode_.control_mode = cargo_msgs::msg::ControlMode::HOVER;
    desired_control_mode_.reference_frame =
        cargo_msgs::msg::ControlMode::UNDEFINED_FRAME;
    return checkMode();
}
} // namespace motionReferenceHandlers
} // namespace cargo

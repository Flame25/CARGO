#include "cargo_behavior_tree/action/go_to_action.hpp"

namespace cargo_behavior_tree {
GoToAction::GoToAction(const std::string &xml_tag_name,
                       const BT::NodeConfiguration &conf)
    : cargo_behavior_tree::BtActionNode<cargo_msgs::action::GoToWaypoint>(
          xml_tag_name, cargo_names::actions::behaviors::gotowaypoint, conf) {}

void GoToAction::on_tick() {
    getInput("max_speed", goal_.max_speed);
    getInput("yaw_angle", goal_.yaw.angle);
    getInput(
        "yaw_mode",
        goal_.yaw.mode); // TODO(pariaspe): runtime warning, called
                         // BT::convertFromString() for type [unsigned char]
    getInput<geometry_msgs::msg::PointStamped>("pose", goal_.target_pose);
}

void GoToAction::on_wait_for_result(
    std::shared_ptr<const cargo_msgs::action::GoToWaypoint::Feedback>
        feedback) {}

} // namespace cargo_behavior_tree

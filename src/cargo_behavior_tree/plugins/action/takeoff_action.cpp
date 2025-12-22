#include "cargo_behavior_tree/action/takeoff_action.hpp"

namespace cargo_behavior_tree {
TakeoffAction::TakeoffAction(const std::string &xml_tag_name,
                             const BT::NodeConfiguration &conf)
    : cargo_behavior_tree::BtActionNode<cargo_msgs::action::Takeoff>(
          xml_tag_name, cargo_names::actions::behaviors::takeoff, conf) {}

void TakeoffAction::on_tick() {
    getInput("height", goal_.takeoff_height);
    getInput("speed", goal_.takeoff_speed);
}

void TakeoffAction::on_wait_for_result(
    std::shared_ptr<const cargo_msgs::action::Takeoff::Feedback> feedback) {}

} // namespace cargo_behavior_tree

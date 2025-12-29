#include "cargo_behavior_tree/action/land_action.hpp"

namespace cargo_behavior_tree {
LandAction::LandAction(const std::string &xml_tag_name,
                       const BT::NodeConfiguration &conf)
    : cargo_behavior_tree::BtActionNode<cargo_msgs::action::Land>(
          xml_tag_name, cargo_names::actions::behaviors::land, conf) {}

void LandAction::on_tick() { getInput("speed", goal_.land_speed); }

void LandAction::on_wait_for_result(
    std::shared_ptr<const cargo_msgs::action::Land::Feedback> feedback) {}

} // namespace cargo_behavior_tree

#pragma once

#include <memory>
#include <string>

#include "cargo_core/names/actions.hpp"
#include "cargo_msgs/action/land.hpp"

#include "cargo_behavior_tree/bt_action_node.hpp"

namespace cargo_behavior_tree {
class LandAction
    : public cargo_behavior_tree::BtActionNode<cargo_msgs::action::Land> {
  public:
    LandAction(const std::string &xml_tag_name,
               const BT::NodeConfiguration &conf);

    void on_tick() override;

    void on_wait_for_result(
        std::shared_ptr<const cargo_msgs::action::Land::Feedback> feedback);

    static BT::PortsList providedPorts() {
        return providedBasicPorts({BT::InputPort<double>("speed")});
    }
};

} // namespace cargo_behavior_tree

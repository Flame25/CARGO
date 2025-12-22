#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "cargo_behavior_tree/bt_action_node.hpp"

#include "cargo_core/names/actions.hpp"
#include "cargo_msgs/action/takeoff.hpp"

namespace cargo_behavior_tree {
class TakeoffAction
    : public cargo_behavior_tree::BtActionNode<cargo_msgs::action::Takeoff> {
  public:
    TakeoffAction(const std::string &xml_tag_name,
                  const BT::NodeConfiguration &conf);

    void on_tick() override;

    BT::NodeStatus on_success() {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        return BT::NodeStatus::SUCCESS;
    }

    void on_wait_for_result(
        std::shared_ptr<const cargo_msgs::action::Takeoff::Feedback> feedback);

    static BT::PortsList providedPorts() {
        return providedBasicPorts(
            {BT::InputPort<double>("height"), BT::InputPort<double>("speed")});
    }

  public:
    std::string action_name_;
};

} // namespace cargo_behavior_tree

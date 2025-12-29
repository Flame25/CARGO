#pragma once

#include "behaviortree_cpp/decorator_node.h"
#include "rclcpp/rclcpp.hpp"
#include <string>

namespace cargo_behavior_tree {

class WaitForTime : public BT::DecoratorNode {
  public:
    WaitForTime(const std::string &xml_tag_name,
                const BT::NodeConfiguration &conf);

    static BT::PortsList providedPorts() {
        // Define the input port for the wait duration in seconds
        return {BT::InputPort<double>("duration")};
    }

  private:
    BT::NodeStatus tick() override;

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Time start_time_;
    bool timer_started_ = false;
    double duration_ = 0.0;
};

} // namespace cargo_behavior_tree

#pragma once

#include <string>

#include "behaviortree_cpp/decorator_node.h"

#include "cargo_msgs/msg/alert_event.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cargo_behavior_tree {
class WaitForAlert : public BT::DecoratorNode {
  public:
    WaitForAlert(const std::string &xml_tag_name,
                 const BT::NodeConfiguration &conf);

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("topic_name"),
                BT::OutputPort("alert")};
    }

  private:
    BT::NodeStatus tick() override;

  private:
    void callback(cargo_msgs::msg::AlertEvent::SharedPtr msg);

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    rclcpp::Subscription<cargo_msgs::msg::AlertEvent>::SharedPtr sub_;
    std::string topic_name_;
    bool flag_ = false;
};

} // namespace cargo_behavior_tree

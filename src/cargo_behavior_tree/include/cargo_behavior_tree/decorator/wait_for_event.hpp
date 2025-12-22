#pragma once

#include <string>

#include "behaviortree_cpp/decorator_node.h"

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace cargo_behavior_tree {
class WaitForEvent : public BT::DecoratorNode {
  public:
    WaitForEvent(const std::string &xml_tag_name,
                 const BT::NodeConfiguration &conf);

    static BT::PortsList providedPorts() {
        return {BT::InputPort<std::string>("topic_name"),
                BT::OutputPort("result")};
    }

  private:
    BT::NodeStatus tick() override;

  private:
    void callback(std_msgs::msg::String::SharedPtr msg);

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    std::string topic_name_;
    bool flag_ = false;
};

} // namespace cargo_behavior_tree

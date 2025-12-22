#pragma once

#include <string>

#include "behaviortree_cpp/condition_node.h"

#include "cargo_core/names/topics.hpp"
#include "cargo_msgs/msg/platform_info.hpp"
#include "cargo_msgs/msg/platform_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cargo_behavior_tree {
class IsFlyingCondition : public BT::ConditionNode {
  public:
    IsFlyingCondition(const std::string &xml_tag_name,
                      const BT::NodeConfiguration &conf);

    IsFlyingCondition() = delete;

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts() { return {}; }

  private:
    void stateCallback(cargo_msgs::msg::PlatformInfo::SharedPtr msg) {
        is_flying_ =
            msg->status.state == cargo_msgs::msg::PlatformStatus::FLYING;
    }

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    rclcpp::Subscription<cargo_msgs::msg::PlatformInfo>::SharedPtr state_sub_;
    bool is_flying_ = false;
};

} // namespace cargo_behavior_tree

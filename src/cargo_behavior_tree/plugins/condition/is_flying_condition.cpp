#include "cargo_behavior_tree/condition/is_flying_condition.hpp"

namespace cargo_behavior_tree {
IsFlyingCondition::IsFlyingCondition(const std::string &xml_tag_name,
                                     const BT::NodeConfiguration &conf)
    : BT::ConditionNode(xml_tag_name, conf) {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(
        callback_group_, node_->get_node_base_interface());

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;
    state_sub_ = node_->create_subscription<cargo_msgs::msg::PlatformInfo>(
        cargo_names::topics::platform::info, cargo_names::topics::platform::qos,
        std::bind(&IsFlyingCondition::stateCallback, this,
                  std::placeholders::_1),
        sub_option);
}

BT::NodeStatus IsFlyingCondition::tick() {
    callback_group_executor_.spin_some();
    if (is_flying_) {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

} // namespace cargo_behavior_tree

#include "cargo_behavior_tree/decorator/wait_for_event.hpp"

namespace cargo_behavior_tree {
WaitForEvent::WaitForEvent(const std::string &xml_tag_name,
                           const BT::NodeConfiguration &conf)
    : BT::DecoratorNode(xml_tag_name, conf) {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    callback_group_ = node_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(
        callback_group_, node_->get_node_base_interface());

    getInput("topic_name", topic_name_);

    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;
    sub_ = node_->create_subscription<std_msgs::msg::String>(
        topic_name_, rclcpp::SystemDefaultsQoS(),
        std::bind(&WaitForEvent::callback, this, std::placeholders::_1),
        sub_option);
}

BT::NodeStatus WaitForEvent::tick() {
    callback_group_executor_.spin_some();
    if (flag_) {
        return child_node_->executeTick();
    }
    return BT::NodeStatus::RUNNING;
}

void WaitForEvent::callback(std_msgs::msg::String::SharedPtr msg) {
    setOutput("result", msg->data);
    flag_ = true;
}

} // namespace cargo_behavior_tree

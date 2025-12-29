#include "cargo_behavior_tree/decorator/wait_for_time.hpp"
#include "cargo_core/node.hpp" // Assuming you use cargo node structure, otherwise standard rclcpp

namespace cargo_behavior_tree {

WaitForTime::WaitForTime(const std::string &xml_tag_name,
                         const BT::NodeConfiguration &conf)
    : BT::DecoratorNode(xml_tag_name, conf) {
    // Get the ROS node pointer from the Behavior Tree config
    // Note: Standard cargo BT nodes pass the ROS node via the config blackboard
    auto blackboard = config().blackboard;
    node_ = blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus WaitForTime::tick() {
    if (!getInput("duration", duration_)) {
        throw BT::RuntimeError("Missing required input [duration]");
    }

    if (status() == BT::NodeStatus::IDLE) {
        start_time_ = node_->now();
        timer_started_ = true;
        setStatus(BT::NodeStatus::RUNNING); // Mark self as running
    }

    double elapsed = (node_->now() - start_time_).seconds();

    if (elapsed < duration_) {
        return BT::NodeStatus::RUNNING;
    } else {
        const BT::NodeStatus child_status = child_node_->executeTick();

        if (child_status != BT::NodeStatus::RUNNING) {
            timer_started_ = false;
            resetChild(); // Reset child for next iteration
        }

        return child_status;
    }
}

} // namespace cargo_behavior_tree

#include "cargo_behavior_tree/action/arm_service.hpp"

namespace cargo_behavior_tree {
ArmService::ArmService(const std::string &xml_tag_name,
                       const BT::NodeConfiguration &conf)
    : cargo_behavior_tree::BtServiceNode<std_srvs::srv::SetBool>(xml_tag_name,
                                                                 conf) {}

void ArmService::on_tick() { this->request_->data = true; }

BT::NodeStatus ArmService::on_completion(
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    return response->success ? BT::NodeStatus::SUCCESS
                             : BT::NodeStatus::FAILURE;
}

DisarmService::DisarmService(const std::string &xml_tag_name,
                             const BT::NodeConfiguration &conf)
    : cargo_behavior_tree::BtServiceNode<std_srvs::srv::SetBool>(xml_tag_name,
                                                                 conf) {}

void DisarmService::on_tick() { this->request_->data = false; }

BT::NodeStatus DisarmService::on_completion(
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    return response->success ? BT::NodeStatus::SUCCESS
                             : BT::NodeStatus::FAILURE;
}

} // namespace cargo_behavior_tree

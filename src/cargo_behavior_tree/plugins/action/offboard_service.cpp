#include "cargo_behavior_tree/action/offboard_service.hpp"

namespace cargo_behavior_tree {
OffboardService::OffboardService(const std::string &xml_tag_name,
                                 const BT::NodeConfiguration &conf)
    : cargo_behavior_tree::BtServiceNode<std_srvs::srv::SetBool>(xml_tag_name,
                                                                 conf) {}

void OffboardService::on_tick() { this->request_->data = true; }

BT::NodeStatus OffboardService::on_completion(
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    return response->success ? BT::NodeStatus::SUCCESS
                             : BT::NodeStatus::FAILURE;
}

} // namespace cargo_behavior_tree

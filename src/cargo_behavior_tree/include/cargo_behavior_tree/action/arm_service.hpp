#pragma once

#include <memory>
#include <string>

#include "cargo_behavior_tree/bt_service_node.hpp"

#include <std_srvs/srv/set_bool.hpp>

namespace cargo_behavior_tree {
class ArmService
    : public cargo_behavior_tree::BtServiceNode<std_srvs::srv::SetBool> {
  public:
    ArmService(const std::string &xml_tag_name,
               const BT::NodeConfiguration &conf);

    void on_tick() override;

    BT::NodeStatus
    on_completion(std::shared_ptr<std_srvs::srv::SetBool::Response> response);
};

class DisarmService
    : public cargo_behavior_tree::BtServiceNode<std_srvs::srv::SetBool> {
  public:
    DisarmService(const std::string &xml_tag_name,
                  const BT::NodeConfiguration &conf);

    void on_tick() override;

    BT::NodeStatus
    on_completion(std::shared_ptr<std_srvs::srv::SetBool::Response> response);
};

} // namespace cargo_behavior_tree

#pragma once

#include <memory>
#include <string>

#include "cargo_core/names/actions.hpp"
#include "cargo_msgs/action/go_to_waypoint.hpp"

#include "cargo_behavior_tree/bt_action_node.hpp"
#include "cargo_behavior_tree/port_specialization.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

namespace cargo_behavior_tree {
class GoToAction : public cargo_behavior_tree::BtActionNode<
                       cargo_msgs::action::GoToWaypoint> {
  public:
    GoToAction(const std::string &xml_tag_name,
               const BT::NodeConfiguration &conf);

    void on_tick();

    void on_wait_for_result(
        std::shared_ptr<const cargo_msgs::action::GoToWaypoint::Feedback>
            feedback);

    static BT::PortsList providedPorts() {
        return providedBasicPorts(
            {BT::InputPort<double>("max_speed"),
             BT::InputPort<double>("yaw_angle"),
             BT::InputPort<geometry_msgs::msg::PointStamped>("pose"),
             BT::InputPort<int>("yaw_mode")});
    }
};

} // namespace cargo_behavior_tree

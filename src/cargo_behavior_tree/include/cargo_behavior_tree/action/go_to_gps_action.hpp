#pragma once

#include <iterator>
#include <memory>
#include <string>

#include "cargo_behavior_tree/bt_action_node.hpp"
#include "cargo_core/names/actions.hpp"
#include "cargo_msgs/action/go_to_waypoint.hpp"
#include "cargo_msgs/srv/geopath_to_path.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace cargo_behavior_tree {
class GoToGpsAction : public cargo_behavior_tree::BtActionNode<
                          cargo_msgs::action::GoToWaypoint> {
  public:
    GoToGpsAction(const std::string &xml_tag_name,
                  const BT::NodeConfiguration &conf);

    void on_tick() override;

    void on_wait_for_result(
        std::shared_ptr<const cargo_msgs::action::GoToWaypoint::Feedback>
            feedback);

    static BT::PortsList providedPorts() {
        return providedBasicPorts({BT::InputPort<double>("max_speed"),
                                   BT::InputPort<double>("yaw_angle"),
                                   BT::InputPort<float>("latitude"),
                                   BT::InputPort<float>("longitude"),
                                   BT::InputPort<float>("altitude"),
                                   BT::InputPort<int>("yaw_mode")});
    }

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<cargo_msgs::srv::GeopathToPath>::SharedPtr client;
    geographic_msgs::msg::GeoPoseStamped geopose;
    geometry_msgs::msg::Point point;
    std::string service_name_;
};

} // namespace cargo_behavior_tree

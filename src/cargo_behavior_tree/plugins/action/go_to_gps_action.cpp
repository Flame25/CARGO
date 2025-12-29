#include "cargo_behavior_tree/action/go_to_gps_action.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cargo_behavior_tree {
GoToGpsAction::GoToGpsAction(const std::string &xml_tag_name,
                             const BT::NodeConfiguration &conf)
    : cargo_behavior_tree::BtActionNode<cargo_msgs::action::GoToWaypoint>(
          xml_tag_name, cargo_names::actions::behaviors::gotowaypoint, conf) {
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    client =
        node_->create_client<cargo_msgs::srv::GeopathToPath>("geopath_to_path");
}

void GoToGpsAction::on_tick() {
    getInput("altitude", geopose.pose.position.altitude);
    getInput("latitude", geopose.pose.position.latitude);
    getInput("longitude", geopose.pose.position.longitude);

    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node_->get_logger(),
                         "interrupted while waiting for the service. exiting.");
            return;
        }
        RCLCPP_INFO(node_->get_logger(),
                    "service: %s not available, waiting again...",
                    service_name_.c_str());
    }

    auto request = std::make_shared<cargo_msgs::srv::GeopathToPath::Request>();

    request->geo_path.poses.push_back(geopose);

    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result,
                                           std::chrono::seconds(1)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_WARN(node_->get_logger(),
                    "failed to receive response from service '%s'",
                    service_name_.c_str());
        return;
    }

    goal_.target_pose.point.x =
        (*(result.get()->path.poses.begin())).pose.position.x;
    goal_.target_pose.point.y =
        (*(result.get()->path.poses.begin())).pose.position.y;
    goal_.target_pose.point.z =
        (*(result.get()->path.poses.begin())).pose.position.z;

    getInput("max_speed", goal_.max_speed);
    getInput("yaw_angle", goal_.yaw.angle);
    getInput(
        "yaw_mode",
        goal_.yaw.mode); // TODO(pariaspe): runtime warning, called
                         // BT::convertFromString() for type [unsigned char]
}

void GoToGpsAction::on_wait_for_result(
    std::shared_ptr<const cargo_msgs::action::GoToWaypoint::Feedback>
        feedback) {}

} // namespace cargo_behavior_tree

#pragma once

#include <geographic_msgs/msg/geo_point.hpp>
#include <memory>
#include <regex>
#include <utility>

#include <cargo_core/names/services.hpp>
#include <cargo_core/utils/gps_utils.hpp>
#include <cargo_core/utils/tf_utils.hpp>
#include <cargo_msgs/srv/get_origin.hpp>
#include <cargo_msgs/srv/set_origin.hpp>

#include "cargo_state_estimator/plugin_base.hpp"

namespace ground_truth {

class Plugin : public cargo_state_estimator_plugin_base::StateEstimatorBase {
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
        twist_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;

    rclcpp::Service<cargo_msgs::srv::SetOrigin>::SharedPtr set_origin_srv_;
    rclcpp::Service<cargo_msgs::srv::GetOrigin>::SharedPtr get_origin_srv_;

    bool use_gps_ = false;
    bool set_origin_on_start_ = false;
    bool earth_to_map_set_ = false;
    double origin_lat_ = 0.0;
    double origin_lon_ = 0.0;
    double origin_alt_ = 0.0;

    geometry_msgs::msg::TransformStamped earth_to_map_;
    geographic_msgs::msg::GeoPoint::UniquePtr origin_;
    sensor_msgs::msg::NavSatFix::UniquePtr gps_pose_;

    bool using_gazebo_tf_ = false;

  public:
    Plugin() : cargo_state_estimator_plugin_base::StateEstimatorBase() {}
    void on_setup() override {
        node_ptr_->get_parameter("use_gps", use_gps_);
        node_ptr_->get_parameter("set_origin_on_start", set_origin_on_start_);

        pose_sub_ =
            node_ptr_->create_subscription<geometry_msgs::msg::PoseStamped>(
                cargo_names::topics::ground_truth::pose,
                cargo_names::topics::ground_truth::qos,
                std::bind(&Plugin::pose_callback, this, std::placeholders::_1));
        twist_sub_ =
            node_ptr_->create_subscription<geometry_msgs::msg::TwistStamped>(
                cargo_names::topics::ground_truth::twist,
                cargo_names::topics::ground_truth::qos,
                std::bind(&Plugin::twist_callback, this,
                          std::placeholders::_1));

        // publish static transform from earth to map and map to odom
        earth_to_map_ = cargo::tf::getTransformation(
            get_earth_frame(), get_map_frame(), 0, 0, 0, 0, 0, 0);
        if (!use_gps_) {
            // TODO(javilinos): MODIFY this to a initial earth to map transform
            // (reading initial position from parameters or msgs )
            publish_static_transform(earth_to_map_);
        } else {
            set_origin_srv_ =
                node_ptr_->create_service<cargo_msgs::srv::SetOrigin>(
                    cargo_names::services::gps::set_origin,
                    std::bind(&Plugin::setOriginCallback, this,
                              std::placeholders::_1, std::placeholders::_2));
            get_origin_srv_ =
                node_ptr_->create_service<cargo_msgs::srv::GetOrigin>(
                    cargo_names::services::gps::get_origin,
                    std::bind(&Plugin::getOriginCallback, this,
                              std::placeholders::_1, std::placeholders::_2));
            gps_sub_ =
                node_ptr_->create_subscription<sensor_msgs::msg::NavSatFix>(
                    cargo_names::topics::sensor_measurements::gps,
                    cargo_names::topics::sensor_measurements::qos,
                    std::bind(&Plugin::gps_callback, this,
                              std::placeholders::_1));

            if (set_origin_on_start_ &&
                node_ptr_->has_parameter("set_origin.lat") &&
                node_ptr_->has_parameter("set_origin.lon") &&
                node_ptr_->has_parameter("set_origin.alt")) {
                node_ptr_->get_parameter("set_origin.lat", origin_lat_);
                node_ptr_->get_parameter("set_origin.lon", origin_lon_);
                node_ptr_->get_parameter("set_origin.alt", origin_alt_);

                origin_ = std::make_unique<geographic_msgs::msg::GeoPoint>();
                origin_->latitude = origin_lat_;
                origin_->longitude = origin_lon_;
                origin_->altitude = origin_alt_;

                RCLCPP_INFO(node_ptr_->get_logger(), "Origin set to %f, %f, %f",
                            origin_lat_, origin_lon_, origin_alt_);
            } else {
                RCLCPP_INFO(node_ptr_->get_logger(),
                            "Waiting for origin to be set");
            }
        }

        geometry_msgs::msg::TransformStamped map_to_odom =
            cargo::tf::getTransformation(get_map_frame(), get_odom_frame(), 0,
                                         0, 0, 0, 0, 0);

        // TODO(javilinos): Remove for next release
        if (node_ptr_->has_parameter("use_gazebo_tf")) {
            node_ptr_->get_parameter("use_gazebo_tf", using_gazebo_tf_);
            if (using_gazebo_tf_) {
                RCLCPP_INFO(node_ptr_->get_logger(), "Using gazebo tfs");
                set_base_frame(
                    cargo::tf::generateTfName("", node_ptr_->get_namespace()));
            }
        }
        publish_static_transform(earth_to_map_);
        publish_static_transform(map_to_odom);
    }

  private:
    void
    generate_map_frame_from_gps(const geographic_msgs::msg::GeoPoint &origin,
                                const sensor_msgs::msg::NavSatFix &gps_pose) {
        cargo::gps::GpsHandler gps_handler;
        gps_handler.setOrigin(origin.latitude, origin.longitude,
                              origin.altitude);
        double x, y, z;
        gps_handler.LatLon2Local(gps_pose.latitude, gps_pose.longitude,
                                 gps_pose.altitude, x, y, z);
        earth_to_map_ = cargo::tf::getTransformation(
            get_earth_frame(), get_map_frame(), x, y, z, 0, 0, 0);
        publish_static_transform(earth_to_map_);
    }

    void generate_map_frame_from_ground_truth_pose(
        const geometry_msgs::msg::PoseStamped &pose) {
        earth_to_map_.transform.translation.x = pose.pose.position.x;
        earth_to_map_.transform.translation.y = pose.pose.position.y;
        earth_to_map_.transform.translation.z = pose.pose.position.z;
        earth_to_map_.transform.rotation.x = pose.pose.orientation.x;
        earth_to_map_.transform.rotation.y = pose.pose.orientation.y;
        earth_to_map_.transform.rotation.z = pose.pose.orientation.z;
        earth_to_map_.transform.rotation.w = pose.pose.orientation.w;
        publish_static_transform(earth_to_map_);
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // since it's ground_truth we consider that the frame obtained is the
        // world frame (earth) so we need to publish the transform from world to
        // base_link, with map and odom as the identity transform in order to
        // keep the continuity of the tf tree we will modify the odom->base_link
        // transform

        if (msg->header.frame_id != get_earth_frame()) {
            RCLCPP_ERROR(node_ptr_->get_logger(),
                         "Received pose in frame %s, expected %s",
                         msg->header.frame_id.c_str(),
                         get_earth_frame().c_str());
            return;
        }

        if (!earth_to_map_set_) {
            generate_map_frame_from_ground_truth_pose(*msg);
            earth_to_map_set_ = true;
        }

        earth_to_baselink.setOrigin(tf2::Vector3(
            msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
        earth_to_baselink.setRotation(
            tf2::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y,
                            msg->pose.orientation.z, msg->pose.orientation.w));

        tf2::fromMsg(earth_to_map_.transform, earth_to_map);

        convert_earth_to_baselink_2_odom_to_baselink_transform(
            earth_to_baselink, odom_to_baselink, earth_to_map);

        auto odom_to_baselink_msg = geometry_msgs::msg::TransformStamped();
        odom_to_baselink_msg.header.stamp = msg->header.stamp;
        odom_to_baselink_msg.header.frame_id = get_odom_frame();
        odom_to_baselink_msg.child_frame_id = get_base_frame();
        odom_to_baselink_msg.transform = tf2::toMsg(odom_to_baselink);

        publish_transform(odom_to_baselink_msg);

        // Pose should be published in the earth frame (world)
        // so we dont need to modify the frame_id
        publish_pose(*msg);
    }

    void twist_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        if (msg->header.frame_id != get_base_frame()) {
            RCLCPP_WARN_ONCE(node_ptr_->get_logger(),
                             "Received twist in frame %s, expected %s. "
                             "Changed to expected one",
                             msg->header.frame_id.c_str(),
                             get_base_frame().c_str());
            // TODO(javilinos): convert it to the base_link frame if needed
        }
        publish_twist(*msg);
    }

    void getOriginCallback(
        const cargo_msgs::srv::GetOrigin::Request::SharedPtr request,
        cargo_msgs::srv::GetOrigin::Response::SharedPtr response) {
        if (origin_) {
            response->origin = *origin_;
            response->success = true;
        } else {
            RCLCPP_WARN(node_ptr_->get_logger(), "Origin not set");
            response->success = false;
        }
    }

    void setOriginCallback(
        const cargo_msgs::srv::SetOrigin::Request::SharedPtr request,
        cargo_msgs::srv::SetOrigin::Response::SharedPtr response) {
        if (origin_) {
            RCLCPP_WARN(node_ptr_->get_logger(), "Origin already set");
            response->success = false;
        } else {
            origin_ = std::make_unique<geographic_msgs::msg::GeoPoint>(
                request->origin);
            RCLCPP_INFO(node_ptr_->get_logger(), "Origin set to %f, %f, %f",
                        origin_->latitude, origin_->longitude,
                        origin_->altitude);
            response->success = true;
            generate_map_frame_from_gps(request->origin, *gps_pose_);
        }
    }

    void gps_callback(sensor_msgs::msg::NavSatFix::UniquePtr msg) {
        // This sould only be called when the use_gps_origin is true
        if (gps_pose_) {
            gps_sub_.reset();
            return;
        }
        gps_pose_ = std::move(msg);

        if (set_origin_on_start_) {
            if (!origin_) {
                origin_ = std::make_unique<geographic_msgs::msg::GeoPoint>();
                origin_->latitude = gps_pose_->latitude;
                origin_->longitude = gps_pose_->longitude;
                origin_->altitude = gps_pose_->altitude;
                RCLCPP_WARN(node_ptr_->get_logger(),
                            "Careful, using GPS pose as origin");
                RCLCPP_INFO(node_ptr_->get_logger(), "Origin set to %f, %f, %f",
                            origin_lat_, origin_lon_, origin_alt_);
            }

            RCLCPP_INFO(node_ptr_->get_logger(),
                        "GPS Callback: Map GPS pose set to %f, %f, %f",
                        gps_pose_->latitude, gps_pose_->longitude,
                        gps_pose_->altitude);

            generate_map_frame_from_gps(*origin_, *gps_pose_);
            earth_to_map_set_ = true;
        }
    }
}; // class GroundTruth
} // namespace ground_truth

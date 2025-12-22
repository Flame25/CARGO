#pragma once

#include <rclcpp/rclcpp.hpp>

namespace cargo_names {
namespace topics {
namespace global {
const rclcpp::QoS qos = rclcpp::QoS(10);
const char alert_event[] = "alert_event";
} // namespace global
namespace sensor_measurements {
const rclcpp::QoS qos = rclcpp::SensorDataQoS();
const char base[] = "sensor_measurements/";
const char imu[] = "sensor_measurements/imu";
const char lidar[] = "sensor_measurements/lidar";
const char gps[] = "sensor_measurements/gps";
const char camera[] = "sensor_measurements/camera";
const char battery[] = "sensor_measurements/battery";
const char odom[] = "sensor_measurements/odom";
} // namespace sensor_measurements
namespace ground_truth {
const rclcpp::QoS qos = rclcpp::SensorDataQoS();
const char pose[] = "ground_truth/pose";
const char twist[] = "ground_truth/twist";
} // namespace ground_truth
namespace self_localization {
const rclcpp::QoS qos = rclcpp::SensorDataQoS();
const char odom[] = "self_localization/odom";
const char pose[] = "self_localization/pose";
const char twist[] = "self_localization/twist";
} // namespace self_localization
namespace motion_reference {
const rclcpp::QoS qos = rclcpp::SensorDataQoS();
const rclcpp::QoS qos_waypoint = rclcpp::QoS(10);
const rclcpp::QoS qos_trajectory = rclcpp::QoS(10);
const char thrust[] = "motion_reference/thrust";
const char pose[] = "motion_reference/pose";
const char twist[] = "motion_reference/twist";
const char trajectory[] = "motion_reference/trajectory";
const char modify_waypoint[] = "motion_reference/modify_waypoint";
const char traj_gen_info[] = "motion_reference/traj_gen_info";
} // namespace motion_reference
namespace actuator_command {
const rclcpp::QoS qos = rclcpp::SensorDataQoS();
const char pose[] = "actuator_command/pose";
const char twist[] = "actuator_command/twist";
const char thrust[] = "actuator_command/thrust";
const char trajectory[] = "actuator_command/trajectory";
} // namespace actuator_command
namespace platform {
const rclcpp::QoS qos = rclcpp::QoS(10);
const char info[] = "platform/info";
} // namespace platform
namespace controller {
const rclcpp::QoS qos_info = rclcpp::QoS(10);
const char info[] = "controller/info";
} // namespace controller
namespace follow_target {
const rclcpp::QoS qos_info = rclcpp::QoS(10);
const char info[] = "follow_target/info";
} // namespace follow_target
} // namespace topics
} // namespace cargo_names

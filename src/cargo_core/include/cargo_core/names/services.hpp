#pragma once

#include <rclcpp/rclcpp.hpp>

namespace cargo_names {
namespace services {
namespace platform {
const char set_arming_state[] = "set_arming_state";
const char set_offboard_mode[] = "set_offboard_mode";
const char set_platform_control_mode[] = "set_platform_control_mode";
const char takeoff[] = "platform_takeoff";
const char land[] = "platform_land";
const char set_platform_state_machine_event[] = "platform/state_machine_event";
const char list_control_modes[] = "platform/list_control_modes";
} // namespace platform
namespace controller {
const char set_control_mode[] = "controller/set_control_mode";
const char list_control_modes[] = "controller/list_control_modes";
} // namespace controller
namespace gps {
const char get_origin[] = "get_origin";
const char set_origin[] = "set_origin";
const char path_to_geopath[] = "";
const char geopath_to_path[] = "";
} // namespace gps
} // namespace services
} // namespace cargo_names

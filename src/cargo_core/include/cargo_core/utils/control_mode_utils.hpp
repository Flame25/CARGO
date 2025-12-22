#pragma once

#include <yaml-cpp/yaml.h>

#include <bitset>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "cargo_msgs/msg/control_mode.hpp"
#include "rclcpp/logging.hpp"

namespace cargo {
namespace control_mode {

// # ------------- mode codification (4 bits) ----------------------
// #
// # unset             = 0 = 0b00000000
// # hover             = 1 = 0b00010000
// # acro              = 2 = 0b00100000
// # attitude          = 3 = 0b00110000
// # speed             = 4 = 0b01000000
// # speed_in_a_plane  = 5 = 0b01010000
// # position          = 6 = 0b01100000
// # trajectory        = 7 = 0b01110000
// #
// #-------------- yaw codification --------------------------------
// #
// # angle             = 0 = 0b00000000
// # speed             = 1 = 0b00000100
// # none              = 2 = 0b00001000
// #
// # frame codification
// #
// # local_frame_flu   = 0 = 0b00000000
// # global_frame_enu  = 1 = 0b00000001
// # global_frame_lla  = 2 = 0b00000010
// # undefined_frame   = 3 = 0b00000011
// #
// #-----------------------------------------------------------------

#define MATCH_ALL 0b11111111
#define MATCH_CONTROL_MODE 0b11110000
#define MATCH_YAW_MODE 0b00001100
#define MATCH_REFERENCE_FRAME 0b00000011
#define UNSET_MODE_MASK 0b00000000
#define HOVER_MODE_MASK 0b00010000

uint8_t
convertCargoControlModeToUint8t(const cargo_msgs::msg::ControlMode &mode);
cargo_msgs::msg::ControlMode
convertUint8tToCargoControlMode(uint8_t control_mode_uint8t);

std::string controlModeToString(const uint8_t control_mode_uint8t);
std::string controlModeToString(const cargo_msgs::msg::ControlMode &mode);

constexpr uint8_t convertToUint8t(const cargo_msgs::msg::ControlMode &mode) {
    return (mode.control_mode << 4) | (mode.yaw_mode << 2) |
           mode.reference_frame;
}

constexpr uint8_t convertToUint8t(uint8_t control_mode_uint8t,
                                  uint8_t yaw_mode_uint8t,
                                  uint8_t reference_frame_uint8t) {
    return (control_mode_uint8t << 4) | (yaw_mode_uint8t << 2) |
           reference_frame_uint8t;
}

inline bool compareModes(const uint8_t mode1, const uint8_t mode2,
                         const uint8_t mask = MATCH_ALL) {
    return (mode1 & mask) == (mode2 & mask);
}

inline bool compareModes(const cargo_msgs::msg::ControlMode &mode1,
                         const cargo_msgs::msg::ControlMode &mode2,
                         const uint8_t mask = MATCH_ALL) {
    return compareModes(convertCargoControlModeToUint8t(mode1),
                        convertCargoControlModeToUint8t(mode2), mask);
}

inline bool isUnsetMode(const uint8_t control_mode_uint8t) {
    return compareModes(control_mode_uint8t, UNSET_MODE_MASK,
                        MATCH_CONTROL_MODE);
}

inline bool isUnsetMode(const cargo_msgs::msg::ControlMode &mode) {
    return mode.control_mode == cargo_msgs::msg::ControlMode::UNSET;
}

inline bool isHoverMode(const uint8_t control_mode_uint8t) {
    return compareModes(control_mode_uint8t, HOVER_MODE_MASK,
                        MATCH_CONTROL_MODE);
}

inline bool isHoverMode(const cargo_msgs::msg::ControlMode &mode) {
    return mode.control_mode == cargo_msgs::msg::ControlMode::HOVER;
}

void printControlMode(const cargo_msgs::msg::ControlMode &mode);
void printControlMode(uint8_t control_mode_uint8t);

} // namespace control_mode
} // namespace cargo

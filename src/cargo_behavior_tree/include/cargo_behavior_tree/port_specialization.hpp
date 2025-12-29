#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <vector>

#include "cargo_msgs/msg/pose_with_id.hpp"

// Template specialization to converts a string to Position2D.
namespace BT {
template <>
inline geometry_msgs::msg::Pose convertFromString(BT::StringView str) {
    // We expect real numbers separated by semicolons
    auto parts = splitString(str, ';');
    if (parts.size() != 3) {
        throw RuntimeError("invalid input)");
    } else {
        geometry_msgs::msg::Pose output;
        output.position.x = convertFromString<double>(parts[0]);
        output.position.y = convertFromString<double>(parts[1]);
        output.position.z = convertFromString<double>(parts[2]);
        return output;
    }
}

template <>
inline geometry_msgs::msg::PointStamped convertFromString(BT::StringView str) {
    // We expect real numbers separated by semicolons
    auto parts = splitString(str, ';');
    if (parts.size() != 3) {
        throw RuntimeError("invalid input)");
    } else {
        geometry_msgs::msg::PointStamped output;
        output.header.frame_id = "earth";
        output.point.x = convertFromString<double>(parts[0]);
        output.point.y = convertFromString<double>(parts[1]);
        output.point.z = convertFromString<double>(parts[2]);
        return output;
    }
}

// TODO(pariaspe): generalize
template <>
inline std::vector<cargo_msgs::msg::PoseWithID>
convertFromString(BT::StringView str) {
    // We expect real numbers separated by semicolons
    auto points = splitString(str, '|');
    auto parts = splitString(points[0], ';');
    if (parts.size() != 3) {
        throw RuntimeError("invalid input)");
    } else {
        std::vector<cargo_msgs::msg::PoseWithID> output;
        cargo_msgs::msg::PoseWithID mypose;
        mypose.id = "0";
        mypose.pose.position.x = convertFromString<double>(parts[0]);
        mypose.pose.position.y = convertFromString<double>(parts[1]);
        mypose.pose.position.z = convertFromString<double>(parts[2]);

        auto parts_2 = splitString(points[1], ';');
        cargo_msgs::msg::PoseWithID mypose_2;
        mypose_2.id = "1";
        mypose_2.pose.position.x = convertFromString<double>(parts_2[0]);
        mypose_2.pose.position.y = convertFromString<double>(parts_2[1]);
        mypose_2.pose.position.z = convertFromString<double>(parts_2[2]);

        output.emplace_back(mypose);
        output.emplace_back(mypose_2);
        return output;
    }
}

} // end namespace BT

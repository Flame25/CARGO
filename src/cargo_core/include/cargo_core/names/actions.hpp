#pragma once

#include <rclcpp/rclcpp.hpp>

namespace cargo_names {
namespace actions {
namespace behaviors {
const char takeoff[] = "TakeoffBehavior";
const char gotowaypoint[] = "GoToBehavior";
const char followreference[] = "FollowReferenceBehavior";
const char followpath[] = "FollowPathBehavior";
const char land[] = "LandBehavior";
const char trajectorygenerator[] = "TrajectoryGeneratorBehavior";
} // namespace behaviors
} // namespace actions
} // namespace cargo_names

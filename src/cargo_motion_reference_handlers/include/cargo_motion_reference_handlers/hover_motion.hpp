#pragma once

#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "cargo_core/node.hpp"
#include "cargo_motion_reference_handlers/basic_motion_references.hpp"

namespace cargo {
namespace motionReferenceHandlers {
/**
 * @brief The HoverMotion class is a motion reference handler that allows the
 *       robot to hover at the current position.
 */
class HoverMotion
    : public cargo::motionReferenceHandlers::BasicMotionReferenceHandler {
  public:
    /**
     * @brief HoverMotion Constructor.
     * @param node cargo::Node pointer.
     */
    explicit HoverMotion(cargo::Node *node_ptr, const std::string &ns = "");

    /**
     * @brief HoverMotion Destructor.
     */
    ~HoverMotion() {}

  public:
    /**
     * @brief Send hover motion command.
     * @returns true if the motion reference was sent successfully.
     */
    bool sendHover();
};

} // namespace motionReferenceHandlers
} // namespace cargo

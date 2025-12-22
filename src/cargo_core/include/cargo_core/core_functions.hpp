#pragma once
#include <memory>

#include "cargo_core/node.hpp"
#include "cargo_core/rate.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace cargo {
/**
 * @brief Executes the main loop of the node
 *
 * @param node node to execute the main loop
 * @param run_function function to be executed in the main loop. Node frequency
 * must be higher than
 * 0
 */

void spinLoop(std::shared_ptr<cargo::Node> node,
              std::function<void()> run_function = nullptr);

} // namespace cargo

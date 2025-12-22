#include "cargo_core/core_functions.hpp"
#include "cargo_core/example_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/executors.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // Create the node
    auto node = std::make_shared<ExampleNode>();

    node->preset_loop_frequency(30);
    // Spin the node (blocking)
    RCLCPP_INFO(node->get_logger(), "New frequency is: %f",
                node->get_loop_frequency());
    cargo::spinLoop(node);
    rclcpp::shutdown();

    return 0;
}

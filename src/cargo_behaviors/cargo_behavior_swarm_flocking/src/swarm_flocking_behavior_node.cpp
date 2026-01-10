#include "cargo_core/core_functions.hpp"
#include <cargo_behavior_swarm_flocking/swarm_flocking_behavior.hpp>

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SwarmFlockingBehavior>();
    // node->preset_loop_frequency(30);
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    // cargo::spinLoop(node);

    rclcpp::shutdown();
    return 0;
}

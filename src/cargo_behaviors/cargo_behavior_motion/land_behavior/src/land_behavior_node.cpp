#include "cargo_core/core_functions.hpp"
#include "land_behavior/land_behavior.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LandBehavior>();
    node->preset_loop_frequency(30);
    cargo::spinLoop(node);

    rclcpp::shutdown();
    return 0;
}

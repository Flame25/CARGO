#include "cargo_core/core_functions.hpp"
#include "takeoff_behavior/takeoff_behavior.hpp"

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TakeoffBehavior>();
    node->preset_loop_frequency(30);
    cargo::spinLoop(node);

    rclcpp::shutdown();
    return 0;
}

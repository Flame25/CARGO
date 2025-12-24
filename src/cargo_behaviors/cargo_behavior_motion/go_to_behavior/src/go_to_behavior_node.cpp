#include "cargo_core/core_functions.hpp"
#include "go_to_behavior/go_to_behavior.hpp"

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GoToBehavior>();
    node->preset_loop_frequency(30);
    cargo::spinLoop(node);

    rclcpp::shutdown();
    return 0;
}

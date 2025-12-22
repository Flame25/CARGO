#include "cargo_core/core_functions.hpp"
#include "cargo_motion_controller/controller_manager.hpp"

int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);

    auto node = std::make_shared<controller_manager::ControllerManager>();

    cargo::spinLoop(node);
    rclcpp::shutdown();
    return 0;
}

#include "cargo_core/core_functions.hpp"
#include "cargo_state_estimator.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cargo_state_estimator::StateEstimator>();
    cargo::spinLoop(node);
    rclcpp::shutdown();
    return 0;
}

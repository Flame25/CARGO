#include "cargo_core/core_functions.hpp"

namespace cargo {
void spinLoop(std::shared_ptr<cargo::Node> node,
              std::function<void()> run_function) {
    node->configure();
    node->activate();

    if (node->get_loop_frequency() <= 0) {
        rclcpp::spin(node->get_node_base_interface());
        return;
    }

    while (rclcpp::ok()) {
        rclcpp::spin_some(node->get_node_base_interface());
        if (run_function != nullptr) {
            run_function();
        }
        if (!node->sleep()) {
            // TODO(miferco97): fix this
            // if sleep returns false, it means that loop rate cannot keep up
            // with the desired rate RCLCPP_INFO(
            //   node->get_logger(),
            //   "Spin loop rate exceeded, stable frequency [%.3f Hz] cannot be
            //   assured ", node->get_loop_frequency());
        }
    }
    // TODO(miferco97): improve this
    node->deactivate();
    node->cleanup();
    node->shutdown();
}

} // namespace cargo

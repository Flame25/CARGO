#pragma once
#include <chrono>
#include <filesystem>
#include <memory>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/logging.hpp>

#include "cargo_core/node.hpp"
#include "cargo_core/utils/control_mode_utils.hpp"
#include "cargo_core/utils/yaml_utils.hpp"
#include "cargo_msgs/msg/controller_info.hpp"

#include "controller_handler.hpp"

namespace controller_manager {

class ControllerManager : public cargo::Node {
  public:
    explicit ControllerManager(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~ControllerManager();

  public:
    double cmd_freq_;

  private:
    double info_freq_;
    std::filesystem::path plugin_name_;
    std::filesystem::path available_modes_config_file_;

    std::shared_ptr<pluginlib::ClassLoader<
        cargo_motion_controller_plugin_base::ControllerBase>>
        loader_;
    std::shared_ptr<controller_handler::ControllerHandler> controller_handler_;
    std::shared_ptr<cargo_motion_controller_plugin_base::ControllerBase>
        controller_;
    rclcpp::Publisher<cargo_msgs::msg::ControllerInfo>::SharedPtr mode_pub_;
    rclcpp::TimerBase::SharedPtr mode_timer_;

  private:
    void configAvailableControlModes(const std::filesystem::path project_path);
    void modeTimerCallback();

    /**
     * @brief Modify the node options to allow undeclared parameters
     */
    static rclcpp::NodeOptions
    get_modified_options(const rclcpp::NodeOptions &options);
}; // class ControllerManager

} // namespace controller_manager

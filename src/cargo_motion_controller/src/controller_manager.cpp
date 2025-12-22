#include "cargo_motion_controller/controller_manager.hpp"

namespace controller_manager {

ControllerManager::ControllerManager(const rclcpp::NodeOptions &options)
    : cargo::Node("controller_manager", get_modified_options(options)) {
    try {
        this->get_parameter("plugin_name", plugin_name_);
    } catch (const rclcpp::ParameterTypeException &e) {
        RCLCPP_FATAL(
            this->get_logger(),
            "Launch argument <plugin_name> not defined or malformed: %s",
            e.what());
        this->~ControllerManager();
    }

    this->get_parameter("cmd_freq", cmd_freq_);
    if (cmd_freq_ <= 0.0f) {
        RCLCPP_ERROR(this->get_logger(),
                     "Param cmd_freq must be greater than 0.0");
        assert(info_freq_ > 0.0f);
        return;
    }

    this->get_parameter("info_freq", info_freq_);
    if (info_freq_ <= 0.0f) {
        RCLCPP_ERROR(this->get_logger(),
                     "Param info_freq must be greater than 0.0");
        assert(info_freq_ > 0.0f);
        return;
    }

    plugin_name_ += "::Plugin";
    // this->get_parameter("plugin_config_file", parameter_string_);
    this->get_parameter("plugin_available_modes_config_file",
                        available_modes_config_file_);

    loader_ = std::make_shared<pluginlib::ClassLoader<
        cargo_motion_controller_plugin_base::ControllerBase>>(
        "cargo_motion_controller",
        "cargo_motion_controller_plugin_base::ControllerBase");
    try {
        controller_ = loader_->createSharedInstance(plugin_name_);
        controller_->initialize(this);
        controller_->reset();
        auto parameters = this->list_parameters({}, 0);
        std::vector<rclcpp::Parameter> params;
        params.reserve(parameters.names.size());
        for (const auto &param : parameters.names) {
            params.emplace_back(this->get_parameter(param));
        }
        controller_->updateParams(params);
        controller_handler_ =
            std::make_shared<controller_handler::ControllerHandler>(controller_,
                                                                    this);
        RCLCPP_INFO(this->get_logger(), "PLUGIN LOADED [%s]",
                    plugin_name_.c_str());
    } catch (pluginlib::PluginlibException &ex) {
        RCLCPP_ERROR(this->get_logger(),
                     "The plugin failed to load for some reason. Error: %s\n",
                     ex.what());
        return;
    }

    // controller_handler_->initialize(this);
    if (available_modes_config_file_.empty()) {
        // Get the path of the package
        available_modes_config_file_ =
            loader_->getPluginManifestPath(plugin_name_);

        // Try search if file available_modes.yaml exists in
        // package_folder/config/
        available_modes_config_file_ =
            available_modes_config_file_.parent_path() / "config" /
            "available_modes.yaml";

        if (!std::filesystem::exists(available_modes_config_file_)) {
            // Try search if file available_modes.yaml exists in
            // package_folder/plugins/plugin_name/config/
            std::string plugin_name;
            this->get_parameter("plugin_name", plugin_name);
            available_modes_config_file_ =
                loader_->getPluginManifestPath(plugin_name_);
            available_modes_config_file_ =
                available_modes_config_file_.parent_path() / "plugins" /
                plugin_name / "config" / "available_modes.yaml";

            if (!std::filesystem::exists(available_modes_config_file_)) {
                RCLCPP_ERROR(this->get_logger(),
                             "Default modes file available_modes.yaml not "
                             "found in plugin config folder: %s",
                             available_modes_config_file_.c_str());
                return;
            }
        }
    }

    RCLCPP_DEBUG(this->get_logger(), "MODES FILE LOADED: %s",
                 available_modes_config_file_.parent_path().c_str());

    configAvailableControlModes(available_modes_config_file_.parent_path());

    mode_pub_ = this->create_publisher<cargo_msgs::msg::ControllerInfo>(
        cargo_names::topics::controller::info,
        cargo_names::topics::controller::qos_info);

    mode_timer_ = this->create_timer(
        std::chrono::duration<double>(1.0f / info_freq_),
        std::bind(&ControllerManager::modeTimerCallback, this));
}

ControllerManager::~ControllerManager() {}

void ControllerManager::configAvailableControlModes(
    const std::filesystem::path project_path) {
    auto available_input_modes = cargo::yaml::parse_uint_from_string(
        cargo::yaml::find_tag_from_project_exports_path<std::string>(
            project_path, "input_control_modes"));
    RCLCPP_INFO(this->get_logger(),
                "==========================================================");
    RCLCPP_INFO(this->get_logger(), "AVAILABLE INPUT MODES: ");
    for (auto mode : available_input_modes) {
        RCLCPP_INFO(this->get_logger(), "\t - %s",
                    cargo::control_mode::controlModeToString(mode).c_str());
    }
    auto available_output_modes = cargo::yaml::parse_uint_from_string(
        cargo::yaml::find_tag_from_project_exports_path<std::string>(
            project_path, "output_control_modes"));
    RCLCPP_INFO(this->get_logger(), "AVAILABLE OUTPUT MODES: ");
    for (auto mode : available_output_modes) {
        RCLCPP_INFO(this->get_logger(), "\t -%s",
                    cargo::control_mode::controlModeToString(mode).c_str());
    }

    RCLCPP_INFO(this->get_logger(),
                "==========================================================");

    controller_handler_->setInputControlModesAvailables(available_input_modes);
    controller_handler_->setOutputControlModesAvailables(
        available_output_modes);
}

void ControllerManager::modeTimerCallback() {
    cargo_msgs::msg::ControllerInfo msg;
    msg.header.stamp = this->now();
    controller_handler_->getMode(msg.input_control_mode,
                                 msg.output_control_mode);
    mode_pub_->publish(msg);
}

rclcpp::NodeOptions
ControllerManager::get_modified_options(const rclcpp::NodeOptions &options) {
    // Create a copy of the options and modify it
    rclcpp::NodeOptions modified_options = options;
    modified_options.allow_undeclared_parameters(true);
    modified_options.automatically_declare_parameters_from_overrides(true);
    return modified_options;
}

} // namespace controller_manager

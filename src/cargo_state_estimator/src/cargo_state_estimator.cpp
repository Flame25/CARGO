#include "cargo_state_estimator.hpp"
namespace cargo_state_estimator {

StateEstimator::StateEstimator(const rclcpp::NodeOptions &options)
    : cargo::Node("state_estimator", get_modified_options(options)) {
    tf_handler_ = std::make_shared<cargo::tf::TfHandler>(this);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tfstatic_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    try {
        this->get_parameter("plugin_name", plugin_name_);
    } catch (const rclcpp::ParameterTypeException &e) {
        RCLCPP_FATAL(
            this->get_logger(),
            "Launch argument <plugin_name> not defined or malformed: %s",
            e.what());
        this->~StateEstimator();
    }
    plugin_name_ += "::Plugin";
    loader_ = std::make_shared<pluginlib::ClassLoader<
        cargo_state_estimator_plugin_base::StateEstimatorBase>>(
        "cargo_state_estimator",
        "cargo_state_estimator_plugin_base::StateEstimatorBase");
    try {
        plugin_ptr_ = loader_->createSharedInstance(plugin_name_);
        plugin_ptr_->setup(this, tf_handler_, tf_broadcaster_,
                           tfstatic_broadcaster_);
    } catch (const pluginlib::PluginlibException &e) {
        RCLCPP_FATAL(this->get_logger(), "Failed to load plugin: %s", e.what());
        this->~StateEstimator();
    }
}

rclcpp::NodeOptions
StateEstimator::get_modified_options(const rclcpp::NodeOptions &options) {
    // Create a copy of the options and modify it
    rclcpp::NodeOptions modified_options = options;
    modified_options.allow_undeclared_parameters(true);
    modified_options.automatically_declare_parameters_from_overrides(true);
    return modified_options;
}

} // namespace cargo_state_estimator

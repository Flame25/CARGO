#include "go_to_behavior/go_to_behavior.hpp"

GoToBehavior::GoToBehavior(const rclcpp::NodeOptions &options)
    : cargo_behavior::BehaviorServer<cargo_msgs::action::GoToWaypoint>(
          cargo_names::actions::behaviors::gotowaypoint, options) {
    try {
        this->declare_parameter<std::string>("plugin_name");
    } catch (const rclcpp::ParameterTypeException &e) {
        RCLCPP_FATAL(
            this->get_logger(),
            "Launch argument <plugin_name> not defined or malformed: %s",
            e.what());
        this->~GoToBehavior();
    }
    try {
        this->declare_parameter<double>("go_to_speed");
    } catch (const rclcpp::ParameterTypeException &e) {
        RCLCPP_FATAL(this->get_logger(),
                     "Launch argument <go_to_speed> not defined or "
                     "malformed: %s",
                     e.what());
        this->~GoToBehavior();
    }
    try {
        this->declare_parameter<double>("go_to_threshold");
    } catch (const rclcpp::ParameterTypeException &e) {
        RCLCPP_FATAL(
            this->get_logger(),
            "Launch argument <go_to_threshold> not defined or malformed: %s",
            e.what());
        this->~GoToBehavior();
    }

    loader_ = std::make_shared<pluginlib::ClassLoader<go_to_base::GoToBase>>(
        "cargo_behavior_motion", "go_to_base::GoToBase");

    tf_handler_ = std::make_shared<cargo::tf::TfHandler>(this);

    try {
        std::string plugin_name =
            this->get_parameter("plugin_name").as_string();
        plugin_name += "::Plugin";
        go_to_plugin_ = loader_->createSharedInstance(plugin_name);

        go_to_base::go_to_plugin_params params;
        params.go_to_speed = this->get_parameter("go_to_speed").as_double();
        params.go_to_threshold =
            this->get_parameter("go_to_threshold").as_double();

        go_to_plugin_->initialize(this, tf_handler_, params);

        RCLCPP_INFO(this->get_logger(), "GO TO BEHAVIOR PLUGIN LOADED: %s",
                    plugin_name.c_str());
    } catch (pluginlib::PluginlibException &ex) {
        RCLCPP_ERROR(this->get_logger(),
                     "The plugin failed to load for some reason. Error: %s\n",
                     ex.what());
        this->~GoToBehavior();
    }

    base_link_frame_id_ = cargo::tf::generateTfName(this, "base_link");

    platform_info_sub_ =
        this->create_subscription<cargo_msgs::msg::PlatformInfo>(
            cargo_names::topics::platform::info,
            cargo_names::topics::platform::qos,
            std::bind(&GoToBehavior::platform_info_callback, this,
                      std::placeholders::_1));

    twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        cargo_names::topics::self_localization::twist,
        cargo_names::topics::self_localization::qos,
        std::bind(&GoToBehavior::state_callback, this, std::placeholders::_1));

    RCLCPP_DEBUG(this->get_logger(), "GoToWaypoint Behavior ready!");
}

GoToBehavior::~GoToBehavior() {}

void GoToBehavior::state_callback(
    const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg) {
    try {
        auto [pose_msg, twist_msg] = tf_handler_->getState(
            *_twist_msg, "earth", "earth", base_link_frame_id_);
        go_to_plugin_->state_callback(pose_msg, twist_msg);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get transform: %s",
                    ex.what());
    }
    return;
}

void GoToBehavior::platform_info_callback(
    const cargo_msgs::msg::PlatformInfo::SharedPtr msg) {
    go_to_plugin_->platform_info_callback(msg);
    return;
}

bool GoToBehavior::process_goal(
    std::shared_ptr<const cargo_msgs::action::GoToWaypoint::Goal> goal,
    cargo_msgs::action::GoToWaypoint::Goal &new_goal) {
    if (goal->target_pose.header.frame_id == "") {
        RCLCPP_ERROR(this->get_logger(), "Target pose frame_id is empty");
        return false;
    }

    if ((fabs(new_goal.target_pose.point.x) +
         fabs(new_goal.target_pose.point.y) +
         fabs(new_goal.target_pose.point.z)) == 0.0f) {
        RCLCPP_WARN(this->get_logger(), "GoToBehavior: Target point is zero");
    } else if (new_goal.target_pose.point.z <= 0.0f) {
        RCLCPP_WARN(this->get_logger(),
                    "GoToBehavior: Target height is below 0.0");
    }

    if (!tf_handler_->tryConvert(new_goal.target_pose, "earth")) {
        RCLCPP_ERROR(
            this->get_logger(),
            "GoToBehavior: can not get target position in earth frame");
        return false;
    }

    geometry_msgs::msg::QuaternionStamped q;
    q.header = goal->target_pose.header;
    cargo::frame::eulerToQuaternion(0.0f, 0.0f, new_goal.yaw.angle,
                                    q.quaternion);

    if (!tf_handler_->tryConvert(q, "earth")) {
        RCLCPP_ERROR(
            this->get_logger(),
            "GoToBehavior: can not get target orientation in earth frame");
        return false;
    }

    new_goal.yaw.angle = cargo::frame::getYawFromQuaternion(q.quaternion);

    new_goal.max_speed = (goal->max_speed != 0.0f)
                             ? goal->max_speed
                             : this->get_parameter("go_to_speed").as_double();

    return true;
}

bool GoToBehavior::on_activate(
    std::shared_ptr<const cargo_msgs::action::GoToWaypoint::Goal> goal) {
    cargo_msgs::action::GoToWaypoint::Goal new_goal = *goal;
    if (!process_goal(goal, new_goal)) {
        return false;
    }
    return go_to_plugin_->on_activate(
        std::make_shared<const cargo_msgs::action::GoToWaypoint::Goal>(
            new_goal));
}

bool GoToBehavior::on_modify(
    std::shared_ptr<const cargo_msgs::action::GoToWaypoint::Goal> goal) {
    cargo_msgs::action::GoToWaypoint::Goal new_goal = *goal;
    if (!process_goal(goal, new_goal)) {
        return false;
    }
    return go_to_plugin_->on_modify(
        std::make_shared<const cargo_msgs::action::GoToWaypoint::Goal>(
            new_goal));
}

bool GoToBehavior::on_deactivate(const std::shared_ptr<std::string> &message) {
    return go_to_plugin_->on_deactivate(message);
}

bool GoToBehavior::on_pause(const std::shared_ptr<std::string> &message) {
    return go_to_plugin_->on_pause(message);
}

bool GoToBehavior::on_resume(const std::shared_ptr<std::string> &message) {
    return go_to_plugin_->on_resume(message);
}

cargo_behavior::ExecutionStatus GoToBehavior::on_run(
    const std::shared_ptr<const cargo_msgs::action::GoToWaypoint::Goal> &goal,
    std::shared_ptr<cargo_msgs::action::GoToWaypoint::Feedback> &feedback_msg,
    std::shared_ptr<cargo_msgs::action::GoToWaypoint::Result> &result_msg) {
    return go_to_plugin_->on_run(goal, feedback_msg, result_msg);
}

void GoToBehavior::on_execution_end(
    const cargo_behavior::ExecutionStatus &state) {
    return go_to_plugin_->on_execution_end(state);
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(GoToBehavior)

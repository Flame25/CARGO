#include "land_behavior/land_behavior.hpp"

LandBehavior::LandBehavior(const rclcpp::NodeOptions &options)
    : cargo_behavior::BehaviorServer<cargo_msgs::action::Land>(
          cargo_names::actions::behaviors::land, options) {
    try {
        this->declare_parameter<std::string>("plugin_name");
    } catch (const rclcpp::ParameterTypeException &e) {
        RCLCPP_FATAL(this->get_logger(),
                     "Launch argument <plugin_name> not defined or "
                     "malformed: %s",
                     e.what());
        this->~LandBehavior();
    }
    try {
        this->declare_parameter<double>("land_speed");
    } catch (const rclcpp::ParameterTypeException &e) {
        RCLCPP_FATAL(this->get_logger(),
                     "Launch argument <land_speed> not defined or "
                     "malformed: %s",
                     e.what());
        this->~LandBehavior();
    }

    loader_ = std::make_shared<pluginlib::ClassLoader<land_base::LandBase>>(
        "cargo_behavior_motion", "land_base::LandBase");

    tf_handler_ = std::make_shared<cargo::tf::TfHandler>(this);

    try {
        std::string plugin_name =
            this->get_parameter("plugin_name").as_string();
        plugin_name += "::Plugin";
        land_plugin_ = loader_->createSharedInstance(plugin_name);

        land_base::land_plugin_params params;
        params.land_speed = this->get_parameter("land_speed").as_double();

        land_plugin_->initialize(this, tf_handler_, params);
        RCLCPP_INFO(this->get_logger(), "LAND BEHAVIOR PLUGIN LOADED: %s",
                    plugin_name.c_str());
    } catch (pluginlib::PluginlibException &ex) {
        RCLCPP_ERROR(this->get_logger(),
                     "The plugin failed to load for some reason. Error: %s\n",
                     ex.what());
        this->~LandBehavior();
    }

    base_link_frame_id_ = cargo::tf::generateTfName(this, "base_link");

    platform_disarm_cli_ = std::make_shared<
        cargo::SynchronousServiceClient<std_srvs::srv::SetBool>>(
        cargo_names::services::platform::set_arming_state, this);

    platform_land_cli_ = std::make_shared<cargo::SynchronousServiceClient<
        cargo_msgs::srv::SetPlatformStateMachineEvent>>(
        cargo_names::services::platform::set_platform_state_machine_event,
        this);

    twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        cargo_names::topics::self_localization::twist,
        cargo_names::topics::self_localization::qos,
        std::bind(&LandBehavior::state_callback, this, std::placeholders::_1));

    RCLCPP_DEBUG(this->get_logger(), "Land Behavior ready!");
}

LandBehavior::~LandBehavior() {}

void LandBehavior::state_callback(
    const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg) {
    try {
        auto [pose_msg, twist_msg] = tf_handler_->getState(
            *_twist_msg, "earth", "earth", base_link_frame_id_);
        land_plugin_->state_callback(pose_msg, twist_msg);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get transform: %s",
                    ex.what());
    }
    return;
}

bool LandBehavior::sendEventFSME(const int8_t _event) {
    cargo_msgs::srv::SetPlatformStateMachineEvent::Request set_platform_fsm_req;
    cargo_msgs::srv::SetPlatformStateMachineEvent::Response
        set_platform_fsm_resp;
    set_platform_fsm_req.event.event = _event;
    auto out = platform_land_cli_->sendRequest(set_platform_fsm_req,
                                               set_platform_fsm_resp, 3);
    if (out && set_platform_fsm_resp.success) {
        return true;
    }
    return false;
}

bool LandBehavior::sendDisarm() {
    RCLCPP_INFO(this->get_logger(), "Disarming platform");
    std_srvs::srv::SetBool::Request set_platform_disarm_req;
    std_srvs::srv::SetBool::Response set_platform_disarm_resp;
    set_platform_disarm_req.data = false;
    auto out = platform_disarm_cli_->sendRequest(set_platform_disarm_req,
                                                 set_platform_disarm_resp, 3);
    if (out && set_platform_disarm_resp.success) {
        return true;
    }
    return false;
}

bool LandBehavior::process_goal(
    std::shared_ptr<const cargo_msgs::action::Land::Goal> goal,
    cargo_msgs::action::Land::Goal &new_goal) {
    new_goal.land_speed =
        (goal->land_speed != 0.0f)
            ? -fabs(goal->land_speed)
            : -fabs(this->get_parameter("land_speed").as_double());

    if (!sendEventFSME(PSME::LAND)) {
        RCLCPP_ERROR(this->get_logger(),
                     "LandBehavior: Could not set FSM to land");
        return false;
    }
    return true;
}

bool LandBehavior::on_activate(
    std::shared_ptr<const cargo_msgs::action::Land::Goal> goal) {
    cargo_msgs::action::Land::Goal new_goal = *goal;
    if (!process_goal(goal, new_goal)) {
        return false;
    }
    return land_plugin_->on_activate(
        std::make_shared<const cargo_msgs::action::Land::Goal>(new_goal));
}

bool LandBehavior::on_modify(
    std::shared_ptr<const cargo_msgs::action::Land::Goal> goal) {
    cargo_msgs::action::Land::Goal new_goal = *goal;
    if (!process_goal(goal, new_goal)) {
        return false;
    }
    return land_plugin_->on_modify(
        std::make_shared<const cargo_msgs::action::Land::Goal>(new_goal));
}

bool LandBehavior::on_deactivate(const std::shared_ptr<std::string> &message) {
    return land_plugin_->on_deactivate(message);
}

bool LandBehavior::on_pause(const std::shared_ptr<std::string> &message) {
    return land_plugin_->on_pause(message);
}

bool LandBehavior::on_resume(const std::shared_ptr<std::string> &message) {
    return land_plugin_->on_resume(message);
}

cargo_behavior::ExecutionStatus LandBehavior::on_run(
    const std::shared_ptr<const cargo_msgs::action::Land::Goal> &goal,
    std::shared_ptr<cargo_msgs::action::Land::Feedback> &feedback_msg,
    std::shared_ptr<cargo_msgs::action::Land::Result> &result_msg) {
    return land_plugin_->on_run(goal, feedback_msg, result_msg);
}

void LandBehavior::on_execution_end(
    const cargo_behavior::ExecutionStatus &state) {
    if (state == cargo_behavior::ExecutionStatus::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "LandBehavior: Land successful");
        if (!sendEventFSME(PSME::LANDED)) {
            RCLCPP_ERROR(this->get_logger(),
                         "LandBehavior: Could not set FSM to Landed");
        }
        if (!sendDisarm()) {
            RCLCPP_ERROR(this->get_logger(), "LandBehavior: Could not disarm");
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "LandBehavior: Land failed");
        if (!sendEventFSME(PSME::EMERGENCY)) {
            RCLCPP_ERROR(this->get_logger(),
                         "LandBehavior: Could not set FSM to EMERGENCY");
        }
    }
    return land_plugin_->on_execution_end(state);
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(LandBehavior)

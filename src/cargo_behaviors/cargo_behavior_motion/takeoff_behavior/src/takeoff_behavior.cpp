#include "takeoff_behavior/takeoff_behavior.hpp"

TakeoffBehavior::TakeoffBehavior(const rclcpp::NodeOptions &options)
    : cargo_behavior::BehaviorServer<cargo_msgs::action::Takeoff>(
          cargo_names::actions::behaviors::takeoff, options) {
    try {
        this->declare_parameter<std::string>("plugin_name");
    } catch (const rclcpp::ParameterTypeException &e) {
        RCLCPP_FATAL(this->get_logger(),
                     "Launch argument <plugin_name> not defined or "
                     "malformed: %s",
                     e.what());
        this->~TakeoffBehavior();
    }
    try {
        this->declare_parameter<double>("takeoff_height");
    } catch (const rclcpp::ParameterTypeException &e) {
        RCLCPP_FATAL(this->get_logger(),
                     "Launch argument <takeoff_height> not defined or "
                     "malformed: %s",
                     e.what());
        this->~TakeoffBehavior();
    }
    try {
        this->declare_parameter<double>("takeoff_speed");
    } catch (const rclcpp::ParameterTypeException &e) {
        RCLCPP_FATAL(this->get_logger(),
                     "Launch argument <takeoff_speed> not defined or "
                     "malformed: %s",
                     e.what());
        this->~TakeoffBehavior();
    }
    try {
        this->declare_parameter<double>("takeoff_threshold");
    } catch (const rclcpp::ParameterTypeException &e) {
        RCLCPP_FATAL(this->get_logger(),
                     "Launch argument <takeoff_threshold> not defined or "
                     "malformed: %s",
                     e.what());
        this->~TakeoffBehavior();
    }

    loader_ =
        std::make_shared<pluginlib::ClassLoader<takeoff_base::TakeoffBase>>(
            "cargo_behavior_motion", "takeoff_base::TakeoffBase");

    tf_handler_ = std::make_shared<cargo::tf::TfHandler>(this);

    try {
        std::string plugin_name =
            this->get_parameter("plugin_name").as_string();
        plugin_name += "::Plugin";
        takeoff_plugin_ = loader_->createSharedInstance(plugin_name);

        takeoff_base::takeoff_plugin_params params;
        params.takeoff_height =
            this->get_parameter("takeoff_height").as_double();
        params.takeoff_speed = this->get_parameter("takeoff_speed").as_double();
        params.takeoff_threshold =
            this->get_parameter("takeoff_threshold").as_double();

        takeoff_plugin_->initialize(this, tf_handler_, params);

        RCLCPP_INFO(this->get_logger(), "TAKEOFF BEHAVIOR PLUGIN LOADED: %s",
                    plugin_name.c_str());
    } catch (pluginlib::PluginlibException &ex) {
        RCLCPP_ERROR(this->get_logger(),
                     "The plugin failed to load for some reason. Error: %s\n",
                     ex.what());
        this->~TakeoffBehavior();
    }

    base_link_frame_id_ = cargo::tf::generateTfName(this, "base_link");

    platform_cli_ = std::make_shared<cargo::SynchronousServiceClient<
        cargo_msgs::srv::SetPlatformStateMachineEvent>>(
        cargo_names::services::platform::set_platform_state_machine_event,
        this);

    twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        cargo_names::topics::self_localization::twist,
        cargo_names::topics::self_localization::qos,
        std::bind(&TakeoffBehavior::state_callback, this,
                  std::placeholders::_1));

    RCLCPP_DEBUG(this->get_logger(), "Takeoff Behavior ready!");
}

TakeoffBehavior::~TakeoffBehavior() {}

void TakeoffBehavior::state_callback(
    const geometry_msgs::msg::TwistStamped::SharedPtr _twist_msg) {
    try {
        auto [pose_msg, twist_msg] = tf_handler_->getState(
            *_twist_msg, "earth", "earth", base_link_frame_id_);
        takeoff_plugin_->state_callback(pose_msg, twist_msg);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not get transform: %s",
                    ex.what());
    }
    return;
}

bool TakeoffBehavior::sendEventFSME(const int8_t _event) {
    cargo_msgs::srv::SetPlatformStateMachineEvent::Request set_platform_fsm_req;
    cargo_msgs::srv::SetPlatformStateMachineEvent::Response
        set_platform_fsm_resp;
    set_platform_fsm_req.event.event = _event;
    auto out = platform_cli_->sendRequest(set_platform_fsm_req,
                                          set_platform_fsm_resp, 3);
    if (out && set_platform_fsm_resp.success) {
        return true;
    }
    return false;
}

bool TakeoffBehavior::process_goal(
    std::shared_ptr<const cargo_msgs::action::Takeoff::Goal> goal,
    cargo_msgs::action::Takeoff::Goal &new_goal) {
    if (goal->takeoff_height < 0.0f) {
        RCLCPP_ERROR(this->get_logger(),
                     "TakeoffBehavior: Invalid takeoff height");
        return false;
    }

    if (goal->takeoff_speed < 0.0f) {
        RCLCPP_WARN(this->get_logger(),
                    "TakeoffBehavior: Invalid takeoff speed, using default: %f",
                    this->get_parameter("takeoff_speed").as_double());
        return false;
    }
    new_goal.takeoff_speed =
        (goal->takeoff_speed != 0.0f)
            ? goal->takeoff_speed
            : this->get_parameter("takeoff_speed").as_double();

    if (!sendEventFSME(PSME::TAKE_OFF)) {
        RCLCPP_ERROR(this->get_logger(),
                     "TakeoffBehavior: Could not set FSM to takeoff");
        return false;
    }
    return true;
}

bool TakeoffBehavior::on_activate(
    std::shared_ptr<const cargo_msgs::action::Takeoff::Goal> goal) {
    cargo_msgs::action::Takeoff::Goal new_goal = *goal;
    if (!process_goal(goal, new_goal)) {
        return false;
    }
    return takeoff_plugin_->on_activate(
        std::make_shared<const cargo_msgs::action::Takeoff::Goal>(new_goal));
}

bool TakeoffBehavior::on_modify(
    std::shared_ptr<const cargo_msgs::action::Takeoff::Goal> goal) {
    cargo_msgs::action::Takeoff::Goal new_goal = *goal;
    if (!process_goal(goal, new_goal)) {
        return false;
    }
    return takeoff_plugin_->on_modify(
        std::make_shared<const cargo_msgs::action::Takeoff::Goal>(new_goal));
}

bool TakeoffBehavior::on_deactivate(
    const std::shared_ptr<std::string> &message) {
    return takeoff_plugin_->on_deactivate(message);
}

bool TakeoffBehavior::on_pause(const std::shared_ptr<std::string> &message) {
    return takeoff_plugin_->on_pause(message);
}

bool TakeoffBehavior::on_resume(const std::shared_ptr<std::string> &message) {
    return takeoff_plugin_->on_resume(message);
}

cargo_behavior::ExecutionStatus TakeoffBehavior::on_run(
    const std::shared_ptr<const cargo_msgs::action::Takeoff::Goal> &goal,
    std::shared_ptr<cargo_msgs::action::Takeoff::Feedback> &feedback_msg,
    std::shared_ptr<cargo_msgs::action::Takeoff::Result> &result_msg) {
    return takeoff_plugin_->on_run(goal, feedback_msg, result_msg);
}

void TakeoffBehavior::on_execution_end(
    const cargo_behavior::ExecutionStatus &state) {
    if (state == cargo_behavior::ExecutionStatus::SUCCESS) {
        if (!sendEventFSME(PSME::TOOK_OFF)) {
            RCLCPP_ERROR(this->get_logger(),
                         "TakeoffBehavior: Could not set FSM to Took OFF");
        }
    } else {
        if (!sendEventFSME(PSME::EMERGENCY)) {
            RCLCPP_ERROR(this->get_logger(),
                         "TakeoffBehavior: Could not set FSM to EMERGENCY");
        }
    }
    return takeoff_plugin_->on_execution_end(state);
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(TakeoffBehavior)

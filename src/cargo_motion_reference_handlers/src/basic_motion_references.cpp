
#include "cargo_motion_reference_handlers/basic_motion_references.hpp"
#include "cargo_core/names/services.hpp"
#include "cargo_core/names/topics.hpp"
#include "cargo_core/synchronous_service_client.hpp"
#include "cargo_core/utils/control_mode_utils.hpp"
#include "cargo_msgs/srv/set_control_mode.hpp"

namespace cargo {
namespace motionReferenceHandlers {
BasicMotionReferenceHandler::BasicMotionReferenceHandler(cargo::Node *cargo_ptr,
                                                         const std::string &ns)
    : node_ptr_(cargo_ptr), namespace_(ns) {
    if (number_of_instances_ == 0) {
        namespace_ = ns == "" ? ns : "/" + ns + "/";

        // Publisher
        command_traj_pub_ =
            node_ptr_->create_publisher<cargo_msgs::msg::TrajectorySetpoints>(
                namespace_ + cargo_names::topics::motion_reference::trajectory,
                cargo_names::topics::motion_reference::qos);

        command_pose_pub_ =
            node_ptr_->create_publisher<geometry_msgs::msg::PoseStamped>(
                namespace_ + cargo_names::topics::motion_reference::pose,
                cargo_names::topics::motion_reference::qos);

        command_twist_pub_ =
            node_ptr_->create_publisher<geometry_msgs::msg::TwistStamped>(
                namespace_ + cargo_names::topics::motion_reference::twist,
                cargo_names::topics::motion_reference::qos);

        command_thrust_pub_ =
            node_ptr_->create_publisher<cargo_msgs::msg::Thrust>(
                namespace_ + cargo_names::topics::motion_reference::thrust,
                cargo_names::topics::motion_reference::qos);

        // Subscriber
        controller_info_sub_ =
            node_ptr_->create_subscription<cargo_msgs::msg::ControllerInfo>(
                namespace_ + cargo_names::topics::controller::info,
                rclcpp::QoS(1),
                [](const cargo_msgs::msg::ControllerInfo::SharedPtr msg) {
                    current_mode_ = msg->input_control_mode;
                });

        // Set initial control mode
        desired_control_mode_.yaw_mode = cargo_msgs::msg::ControlMode::NONE;
        desired_control_mode_.control_mode =
            cargo_msgs::msg::ControlMode::UNSET;
        desired_control_mode_.reference_frame =
            cargo_msgs::msg::ControlMode::UNDEFINED_FRAME;
    }

    number_of_instances_++;
    RCLCPP_DEBUG(
        node_ptr_->get_logger(),
        "There are %d instances of BasicMotionReferenceHandler created",
        number_of_instances_);
}

BasicMotionReferenceHandler::~BasicMotionReferenceHandler() {
    number_of_instances_--;
    if (number_of_instances_ == 0 && node_ptr_ != nullptr) {
        RCLCPP_DEBUG(node_ptr_->get_logger(), "Deleting node_ptr_");
        controller_info_sub_.reset();
        command_traj_pub_.reset();
        command_pose_pub_.reset();
        command_twist_pub_.reset();
        command_thrust_pub_.reset();
    }
}

bool BasicMotionReferenceHandler::checkMode() {
    // TODO(rps): Check comparation
    // if (this->current_mode_ != desired_control_mode_)
    if ((this->current_mode_.control_mode ==
         desired_control_mode_.control_mode) &&
        (this->current_mode_.control_mode ==
         cargo_msgs::msg::ControlMode::HOVER)) {
        return true;
    }

    if (this->current_mode_.yaw_mode != desired_control_mode_.yaw_mode ||
        this->current_mode_.control_mode !=
            desired_control_mode_.control_mode) {
        if (!setMode(desired_control_mode_)) {
            return false;
        }
    }
    return true;
}

bool BasicMotionReferenceHandler::sendPoseCommand() {
    if (!checkMode()) {
        return false;
    }
    command_pose_pub_->publish(command_pose_msg_);
    return true;
}

bool BasicMotionReferenceHandler::sendTwistCommand() {
    if (!checkMode()) {
        return false;
    }
    command_twist_pub_->publish(command_twist_msg_);
    return true;
}

bool BasicMotionReferenceHandler::sendTrajectoryCommand() {
    if (!checkMode()) {
        return false;
    }
    command_traj_pub_->publish(command_trajectory_msg_);
    return true;
}

bool BasicMotionReferenceHandler::sendThrustCommand() {
    if (!checkMode()) {
        return false;
    }
    command_thrust_pub_->publish(command_thrust_msg_);
    return true;
}

bool BasicMotionReferenceHandler::setMode(
    const cargo_msgs::msg::ControlMode &mode) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Setting control mode to [%s]",
                cargo::control_mode::controlModeToString(mode).c_str());

    // Set request
    auto request = cargo_msgs::srv::SetControlMode::Request();
    auto response = cargo_msgs::srv::SetControlMode::Response();
    request.control_mode = mode;

    auto set_mode_cli =
        cargo::SynchronousServiceClient<cargo_msgs::srv::SetControlMode>(
            namespace_ + cargo_names::services::controller::set_control_mode,
            node_ptr_);

    bool out = set_mode_cli.sendRequest(request, response);

    if (out && response.success) {
        this->current_mode_ = mode;
        // Sleep for controller info callback to update
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return true;
    }
    RCLCPP_ERROR(
        node_ptr_->get_logger(),
        " Controller Control Mode was not able to be settled sucessfully");
    return false;
}

int BasicMotionReferenceHandler::number_of_instances_ = 0;

rclcpp::Subscription<cargo_msgs::msg::ControllerInfo>::SharedPtr
    BasicMotionReferenceHandler::controller_info_sub_ = nullptr;

cargo_msgs::msg::ControlMode BasicMotionReferenceHandler::current_mode_ =
    cargo_msgs::msg::ControlMode();

rclcpp::Publisher<cargo_msgs::msg::TrajectorySetpoints>::SharedPtr
    BasicMotionReferenceHandler::command_traj_pub_ = nullptr;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
    BasicMotionReferenceHandler::command_pose_pub_ = nullptr;
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
    BasicMotionReferenceHandler::command_twist_pub_ = nullptr;
rclcpp::Publisher<cargo_msgs::msg::Thrust>::SharedPtr
    BasicMotionReferenceHandler::command_thrust_pub_ = nullptr;

} // namespace motionReferenceHandlers
} // namespace cargo

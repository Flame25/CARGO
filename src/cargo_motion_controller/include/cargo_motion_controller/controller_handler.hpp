#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <rcl/time.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/timer.hpp>
#include <string>
#include <tf2/time.h>
#include <vector>

#include "cargo_core/names/services.hpp"
#include "cargo_core/names/topics.hpp"
#include "cargo_core/node.hpp"
#include "cargo_core/synchronous_service_client.hpp"
#include "cargo_core/utils/control_mode_utils.hpp"
#include "cargo_core/utils/tf_utils.hpp"
#include "cargo_msgs/msg/control_mode.hpp"
#include "cargo_msgs/msg/platform_info.hpp"
#include "cargo_msgs/msg/thrust.hpp"
#include "cargo_msgs/msg/trajectory_setpoints.hpp"
#include "cargo_msgs/srv/list_control_modes.hpp"
#include "cargo_msgs/srv/set_control_mode.hpp"

#include "controller_base.hpp"

namespace controller_handler {

#define MATCH_ALL 0b11111111
#define MATCH_MODE_AND_FRAME 0b11110011
#define MATCH_MODE 0b11110000
#define MATCH_MODE_AND_YAW 0b11111100

#define UNSET_MODE_MASK 0b00000000
#define HOVER_MODE_MASK 0b00010000

using namespace std::chrono_literals; // NOLINT

class ControllerHandler {
  public:
    ControllerHandler(
        std::shared_ptr<cargo_motion_controller_plugin_base::ControllerBase>
            controller,
        cargo::Node *node);

    virtual ~ControllerHandler() {}

    rcl_interfaces::msg::SetParametersResult
    parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

    void getMode(cargo_msgs::msg::ControlMode &mode_in,
                 cargo_msgs::msg::ControlMode &mode_out);
    void
    setInputControlModesAvailables(const std::vector<uint8_t> &available_modes);
    void setOutputControlModesAvailables(
        const std::vector<uint8_t> &available_modes);

    void reset();

  protected:
    cargo::Node *node_ptr_;

  private:
    // Control modes availables
    std::vector<uint8_t> controller_available_modes_in_;
    std::vector<uint8_t> controller_available_modes_out_;
    std::vector<uint8_t> platform_available_modes_in_;

    // Frame ids
    std::string enu_frame_id_ = "odom";
    std::string flu_frame_id_ = "base_link";
    std::string input_pose_frame_id_ = "odom";
    std::string input_twist_frame_id_ = "odom";
    std::string output_pose_frame_id_ = "odom";
    std::string output_twist_frame_id_ = "odom";

    // TF handler
    cargo::tf::TfHandler tf_handler_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
        twist_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        ref_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
        ref_twist_sub_;
    rclcpp::Subscription<cargo_msgs::msg::TrajectorySetpoints>::SharedPtr
        ref_traj_sub_;
    rclcpp::Subscription<cargo_msgs::msg::Thrust>::SharedPtr ref_thrust_sub_;
    rclcpp::Subscription<cargo_msgs::msg::PlatformInfo>::SharedPtr
        platform_info_sub_;

    // Publishers
    rclcpp::Publisher<cargo_msgs::msg::TrajectorySetpoints>::SharedPtr
        trajectory_pub_;
    rclcpp::Publisher<cargo_msgs::msg::Thrust>::SharedPtr thrust_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;

    // Services servers
    rclcpp::Service<cargo_msgs::srv::SetControlMode>::SharedPtr
        set_control_mode_srv_;

    // Services clients
    cargo::SynchronousServiceClient<cargo_msgs::srv::SetControlMode>::SharedPtr
        set_control_mode_client_;
    cargo::SynchronousServiceClient<cargo_msgs::srv::ListControlModes>::
        SharedPtr list_control_modes_client_;

    // Timers
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Internal variables
    bool control_mode_established_ = false;
    bool motion_reference_adquired_ = false;
    bool state_adquired_ = false;
    bool use_bypass_ = false;
    bool bypass_controller_ = false;

    uint8_t prefered_output_mode_ =
        0b00000000; // by default, no output mode is prefered

    rclcpp::Time last_time_;

    cargo_msgs::msg::PlatformInfo platform_info_;
    cargo_msgs::msg::ControlMode control_mode_in_;
    cargo_msgs::msg::ControlMode control_mode_out_;

    geometry_msgs::msg::PoseStamped state_pose_;
    geometry_msgs::msg::TwistStamped state_twist_;
    geometry_msgs::msg::PoseStamped ref_pose_;
    geometry_msgs::msg::TwistStamped ref_twist_;
    cargo_msgs::msg::TrajectorySetpoints ref_traj_;
    cargo_msgs::msg::Thrust ref_thrust_;
    geometry_msgs::msg::PoseStamped command_pose_;
    geometry_msgs::msg::TwistStamped command_twist_;
    cargo_msgs::msg::Thrust command_thrust_;

    // Controller plugin
    std::shared_ptr<cargo_motion_controller_plugin_base::ControllerBase>
        controller_ptr_;

  private:
    // Subscribers callbacks
    void stateCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void refPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void
    refTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void
    refTrajCallback(const cargo_msgs::msg::TrajectorySetpoints::SharedPtr msg);
    void refThrustCallback(const cargo_msgs::msg::Thrust::SharedPtr msg);
    void
    platformInfoCallback(const cargo_msgs::msg::PlatformInfo::SharedPtr msg);

    // Services servers callbacks
    void setControlModeSrvCall(
        const cargo_msgs::srv::SetControlMode::Request::SharedPtr request,
        cargo_msgs::srv::SetControlMode::Response::SharedPtr response);
    bool listPlatformAvailableControlModes();

    // Timer callbacks
    void controlTimerCallback();

    // Internal methods
    std::string getFrameIdByReferenceFrame(uint8_t reference_frame);

    bool
    findSuitableOutputControlModeForPlatformInputMode(uint8_t &output_mode,
                                                      const uint8_t input_mode);
    bool checkSuitabilityInputMode(uint8_t &input_mode,
                                   const uint8_t output_mode);
    bool setPlatformControlMode(const cargo_msgs::msg::ControlMode &mode);

    bool findSuitableControlModes(uint8_t &input_mode, uint8_t &output_mode);
    bool trySetPlatformHover();
    bool tryToBypassController(const uint8_t input_mode, uint8_t &output_mode);

    void sendCommand();
    void publishCommand();
}; //  class ControllerBase

} //  namespace controller_handler

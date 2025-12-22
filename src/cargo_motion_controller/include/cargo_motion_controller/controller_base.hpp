#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "cargo_core/node.hpp"
#include "cargo_msgs/msg/control_mode.hpp"
#include "cargo_msgs/msg/thrust.hpp"
#include "cargo_msgs/msg/trajectory_setpoints.hpp"

namespace cargo_motion_controller_plugin_base {

class ControllerBase {
  public:
    /*
     * @brief Constructor
     */
    ControllerBase() {}

    /*
     * @brief Initialize the controller plugin, since it is a plugin the
     * Constructor must not have parameters
     * @param node_ptr cargo::Node pointer to be used by the controller plugin
     */
    void initialize(cargo::Node *node_ptr) {
        node_ptr_ = node_ptr;
        ownInitialize();
    }

    /*
     * @brief Own initialize function to be implemented by the controller plugin
     */
    virtual void ownInitialize() {}

    /*
     * @brief Update the State obtained from the sensors to be used by the
     * controller plugin
     * @param pose_msg geometry_msgs::msg::PoseStamped message with the current
     * pose of the robot in the "odom" frame
     * @param twist_msg geometry_msgs::msg::TwistStamped message with the
     * current twist of the robot in the "base_link" frame
     */
    virtual void
    updateState(const geometry_msgs::msg::PoseStamped &pose_msg,
                const geometry_msgs::msg::TwistStamped &twist_msg) = 0;

    /*
     * @brief Update the pose reference to be used by the controller plugin
     * @param ref geometry_msgs::msg::PoseStamped message with the current pose
     * of the robot in the "odom" frame
     */
    virtual void updateReference(const geometry_msgs::msg::PoseStamped &ref) {}

    /*
     * @brief Update the speedreference to be used by the controller plugin
     * @param ref geometry_msgs::msg::TwistStamped message with the current
     * twist of the robot in the "base_link" frame
     */
    virtual void updateReference(const geometry_msgs::msg::TwistStamped &ref) {}
    /*
     * @brief Update the reference to be used by the controller plugin
     * @param ref cargo_msgs::msg::TrajectorySetpoints message with the current
     * reference of the robot in the "odom" frame
     */
    virtual void
    updateReference(const cargo_msgs::msg::TrajectorySetpoints &ref) {}

    /*
     * @brief Update the thrust reference to be used by the controller plugin
     * @param ref cargo_msgs::msg::Thrust message with the current thrust
     * reference of the robot
     */
    virtual void updateReference(const cargo_msgs::msg::Thrust &ref) {}

    /*
     * @brief Compute the output signal of the controller plugin
     * @param pose geometry_msgs::msg::PoseStamped message with the output pose
     * of the robot. The frame will depend on the output control mode
     * @param twist geometry_msgs::msg::TwistStamped message with the output
     * twist of the robot. The frame will depend on the output control mode
     * @param thrust cargo_msgs::msg::Thrust message with the output thrust of
     * the robot
     */
    virtual bool computeOutput(double dt, geometry_msgs::msg::PoseStamped &pose,
                               geometry_msgs::msg::TwistStamped &twist,
                               cargo_msgs::msg::Thrust &thrust) = 0;
    /*
     * @brief Update the control mode to be used by the controller plugin
     * @param mode_in cargo_msgs::msg::ControlMode message with the desired
     * input control mode
     * @param mode_out cargo_msgs::msg::ControlMode message with the desired
     * output control mode
     * @return bool true if the in-out control mode configuration is valid,
     * false otherwise
     */
    virtual bool setMode(const cargo_msgs::msg::ControlMode &mode_in,
                         const cargo_msgs::msg::ControlMode &mode_out) = 0;

    /*
     * @brief Update the parameters of the controller plugin
     * @param params std::vector<rclcpp::Parameter> vector with the parameters
     * of the Controller Manager Node
     * @return bool true if the parameters are updated correctly, false
     * otherwise
     */
    virtual bool
    updateParams(const std::vector<rclcpp::Parameter> &_params_list) = 0;

    /*
     * @brief Reset the internal state of the controller plugin
     */
    virtual void reset() = 0;

    /*
     * @brief Get the desired frame_id of the state and reference pose msgs
     * By default it is "odom"
     */
    virtual std::string getDesiredPoseFrameId() { return "odom"; }

    /*
     * @brief Get the desired frame_id of the state and reference twist msgs
     * By default it is "base_link"
     */
    virtual std::string getDesiredTwistFrameId() { return "base_link"; }

    /*
     * @brief Destructor
     */
    virtual ~ControllerBase() {}

  protected:
    /* @brief cargo::Node pointer to be used by the controller plugin */
    inline cargo::Node *getNodePtr() { return node_ptr_; }
    cargo::Node *node_ptr_;
}; //  class ControllerBase

} // namespace cargo_motion_controller_plugin_base

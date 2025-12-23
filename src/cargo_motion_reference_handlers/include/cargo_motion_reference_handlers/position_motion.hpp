#pragma once

#include <string>

#include <cargo_core/utils/tf_utils.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "cargo_core/node.hpp"
#include "cargo_motion_reference_handlers/basic_motion_references.hpp"

namespace cargo {
namespace motionReferenceHandlers {

/**
 * @brief The PositionMotion class is a motion reference handler that moves the
 *       robot to a given position.
 */
class PositionMotion
    : public cargo::motionReferenceHandlers::BasicMotionReferenceHandler {
  public:
    /**
     * @brief PositionMotion Constructor.
     * @param node cargo::Node pointer.
     */
    explicit PositionMotion(cargo::Node *node_ptr, const std::string &ns = "");

    /**
     * @brief PositionMotion Destructor.
     */
    ~PositionMotion() {}

  public:
    /**
     * @brief ownSendCommand sends the pose and twist messages.
     * @return true if commands was sent successfully, false otherwise.
     */
    bool ownSendCommand();

    /**
     * @brief sendPositionCommandWithYawAngle sends a position command to the
     *       robot.
     *       The yaw angle is given in radians.
     *       The linear velocity limitation is given in m/s.
     *       The position command and the velocity limitation are sent in the
     * input frame id.
     * @param frame_id_pose frame id of the position command.
     * @param x x coordinate of the position command.
     * @param y y coordinate of the position command.
     * @param z z coordinate of the position command.
     * @param yaw_angle yaw angle of the position command.
     * @param frame_id_twist frame id of the velocity limitation.
     * @param vx linear velocity limitation in x direction.
     * @param vy linear velocity limitation in y direction.
     * @param vz linear velocity limitation in z direction.
     * @return true if the command was sent successfully, false otherwise.
     */
    bool sendPositionCommandWithYawAngle(const std::string &frame_id_pose,
                                         const float &x, const float &y,
                                         const float &z, const float &yaw_angle,
                                         const std::string &frame_id_twist,
                                         const float &vx, const float &vy,
                                         const float &vz);

    /**
     * @brief sendPositionCommandWithYawAngle sends a position command to the
     *      robot.
     *      The yaw angle is given in a quaternion.
     *      The linear velocity is given in m/s.
     *      The position command and the velocity limitation are sent in the
     * input frame id.
     * @param frame_id_pose frame id of the position command.
     * @param x x coordinate of the position command.
     * @param y y coordinate of the position command.
     * @param z z coordinate of the position command.
     * @param q quaternion of the position command. (with the desired yaw
     * angle).
     * @param frame_id_twist frame id of the velocity limitation.
     * @param vx linear velocity limitation in x direction.
     * @param vy linear velocity limitation in y direction.
     * @param vz linear velocity limitation in z direction.
     * @return true if the command was sent successfully, false otherwise.
     */
    bool sendPositionCommandWithYawAngle(
        const std::string &frame_id_pose, const float &x, const float &y,
        const float &z, const geometry_msgs::msg::Quaternion &q,
        const std::string &frame_id_twist, const float &vx, const float &vy,
        const float &vz);
    /**
     * @brief sendPositionCommandWithYawAngle sends a position command to the
     *     robot.
     *     The position command is sent in the frame id frame.
     * @param pose geometry_msgs::msg::PoseStamped with the desired position and
     * yaw angle.
     * @param twist geometry_msgs::msg::TwistStamped with the desired linear
     * velocity.
     * @return true if the command was sent successfully, false otherwise.
     */
    bool sendPositionCommandWithYawAngle(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::TwistStamped &twist);

    /**
     * @brief sendPositionCommandWithYawSpeed sends a position command to the
     *     robot.
     *     The yaw speed is given in rad/s.
     *     The linear velocity is given in m/s.
     *     The position command and the velocity limitation are sent in the
     * input frame id.
     * @param frame_id_pose frame id of the position command.
     * @param x x coordinate of the position command.
     * @param y y coordinate of the position command.
     * @param z z coordinate of the position command.
     * @param yaw_speed yaw speed of the position command.
     * @param frame_id_twist frame id of the velocity limitation.
     * @param vx linear velocity limitation in x direction.
     * @param vy linear velocity limitation in y direction.
     * @param vz linear velocity limitation in z direction.
     * @return true if the command was sent successfully, false otherwise.
     */
    bool sendPositionCommandWithYawSpeed(const std::string &frame_id_pose,
                                         const float &x, const float &y,
                                         const float &z, const float &yaw_speed,
                                         const std::string &frame_id_twist,
                                         const float &vx, const float &vy,
                                         const float &vz);

    /**
     * @brief sendPositionCommandWithYawSpeed sends a position command to the
     *    robot.
     *    The position command is sent in the frame id frame.
     * @param x pose geometry_msgs::msg::PoseStamped with the desired position.
     * @param twist geometry_msgs::msg::TwistStamped with the desired linear
     * velocity and yaw speed.
     * @return true if the command was sent successfully, false otherwise.
     */

    bool sendPositionCommandWithYawSpeed(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::TwistStamped &twist);
};

} // namespace motionReferenceHandlers
} // namespace cargo

#pragma once

#include <math.h>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace cargo {
namespace frame {

/**
 * @brief Apply a quaternion rotation to a vector.
 *
 * @param quaternion tf2::Quaternion to apply.
 * @param vector Eigen::Vector3d Vector to rotate.
 * @return
 * @return Eigen::Vector3d Rotated vector.
 */
Eigen::Vector3d transform(const tf2::Quaternion &quaternion,
                          const Eigen::Vector3d &vector);

/**
 * @brief Apply a quaternion rotation to a vector.
 *
 * @param roll_angle Roll angle to apply.
 * @param pitch_angle Pitch angle to apply.
 * @param yaw_angle Yaw angle to apply.
 * @param vector Eigen::Vector3d Vector to rotate.
 * @return
 * @return Eigen::Vector3d Rotated vector.
 */
Eigen::Vector3d transform(const float roll_angle, const float pitch_angle,
                          const float yaw_angle, const Eigen::Vector3d &vector);

/**
 * @brief Apply a quaternion rotation to a vector.
 *
 * @param quaternion geometry_msgs::Quaternion to apply.
 * @param vector Eigen::Vector3d Vector to rotate.
 * @return
 * @return Eigen::Vector3d Rotated vector.
 */
Eigen::Vector3d transform(const geometry_msgs::msg::Quaternion &quaternion,
                          const Eigen::Vector3d &vector);

/**
 * @brief Apply a quaternion rotation to a vector.
 *
 * @param quaternion Eigen::Quaterniond to apply.
 * @param vector Eigen::Vector3d Vector to rotate.
 * @return
 * @return Eigen::Vector3d Rotated vector.
 */
Eigen::Vector3d transform(const Eigen::Quaterniond &quaternion,
                          const Eigen::Vector3d &vector); // NOLINT

/**
 * @brief Apply a inverse quaternion rotation to a vector.
 *
 * @param quaternion tf2::Quaternion to apply.
 * @param vector Eigen::Vector3d Vector to rotate.
 * @return
 * @return Eigen::Vector3d Rotated vector.
 */
Eigen::Vector3d transformInverse(const tf2::Quaternion &quaternion,
                                 const Eigen::Vector3d &vector);
/**
 * @brief Apply a inverse quaternion rotation to a vector.
 *
 * @param roll_angle Roll angle to apply.
 * @param pitch_angle Pitch angle to apply.
 * @param yaw_angle Yaw angle to apply.
 * @param vector Eigen::Vector3d Vector to rotate.
 * @return
 * @return Eigen::Vector3d Rotated vector.
 */
Eigen::Vector3d transformInverse(const float roll_angle,
                                 const float pitch_angle, const float yaw_angle,
                                 const Eigen::Vector3d &vector);

/**
 * @brief Apply a inverse quaternion rotation to a vector.
 *
 * @param quaternion geometry_msgs::Quaternion to apply.
 * @param vector Eigen::Vector3d Vector to rotate.
 * @return
 * @return Eigen::Vector3d Rotated vector.
 */
Eigen::Vector3d
transformInverse(const geometry_msgs::msg::Quaternion &quaternion,
                 const Eigen::Vector3d &vector);

/**
 * @brief Apply a inverse quaternion rotation to a vector.
 *
 * @param quaternion Eigen::Quaterniond to apply.
 * @param vector Eigen::Vector3d Vector to rotate.
 * @return
 * @return Eigen::Vector3d Rotated vector.
 */
Eigen::Vector3d transformInverse(const Eigen::Quaterniond &quaternion,
                                 const Eigen::Vector3d &vector);

/**
 * @brief Apply a inverse quaternion rotation to a vector.
 *
 * @param quaternion geometry_msgs::msg::Pose with the quaternion to apply.
 * @param vector Eigen::Vector3d Vector to rotate.
 * @return
 * @return Eigen::Vector3d Rotated vector.
 */
Eigen::Vector3d transformInverse(const Eigen::Quaterniond &quaternion,
                                 const Eigen::Vector3d &vector);

/**
 * @brief Convert quaternion to euler angles.
 *
 * @param quaternion tf2::Quaternion to convert.
 * @param roll double pointer to store roll angle.
 * @param pitch double pointer to store pitch angle.
 * @param yaw double pointer to store yaw angle.
 */
void quaternionToEuler(const tf2::Quaternion &quaternion, double &roll,
                       double &pitch, double &yaw);

/**
 * @brief Convert quaternion to euler angles.
 *
 * @param quaternion geometry_msgs::msg::Quaternion to convert.
 * @param roll double pointer to store roll angle.
 * @param pitch double pointer to store pitch angle.
 * @param yaw double pointer to store yaw angle.
 */
void quaternionToEuler(const geometry_msgs::msg::Quaternion &quaternion,
                       double &roll, double &pitch, double &yaw);

/**
 * @brief Convert quaternion to euler angles.
 *
 * @param quaternion Eigen::Quaternion to convert.
 * @param yaw double pointer to store yaw angle.
 */
void quaternionToEuler(const Eigen::Quaterniond &quaternion, double &roll,
                       double &pitch, double &yaw);

/**
 * @brief Convert euler angles to quaternion.
 *
 * @param roll double roll angle.
 * @param pitch double pitch angle.
 * @param yaw double yaw angle.
 * @param quaternion tf2::Quaternion pointer to store quaternion.
 */
void eulerToQuaternion(const double roll, const double pitch, const double yaw,
                       tf2::Quaternion &quaternion);

/**
 * @brief Convert euler angles to quaternion.
 *
 * @param roll double roll angle.
 * @param pitch double pitch angle.
 * @param yaw double yaw angle.
 * @param quaternion geometry_msgs::msg::Quaternion pointer to store quaternion.
 */
void eulerToQuaternion(const double roll, const double pitch, const double yaw,
                       geometry_msgs::msg::Quaternion &quaternion);

/**
 * @brief Convert euler angles to quaternion.
 *
 * @param roll double roll angle.
 * @param pitch double pitch angle.
 * @param yaw double yaw angle.
 * @param quaternion Eigen::Quaterniond pointer to store quaternion.
 */
void eulerToQuaternion(const double roll, const double pitch, const double yaw,
                       Eigen::Quaterniond &quaternion);

/**
 * @brief Convert quaternion to euler angles.
 *
 * @param quaternion tf2::Quaternion to convert.
 * @param yaw double pointer to store yaw angle.
 */
double getYawFromQuaternion(const tf2::Quaternion &quaternion);

/**
 * @brief Convert quaternion to euler angles.
 *
 * @param quaternion geometry_msgs::msg::Quaternion to convert.
 * @param yaw double pointer to store yaw angle.
 * @return Double yaw angle.
 */
double getYawFromQuaternion(const geometry_msgs::msg::Quaternion &quaternion);

/**
 * @brief Convert quaternion to euler angles.
 *
 * @param quaternion Eigen::Quaternion to convert.
 * @param yaw double pointer to store yaw angle.
 * @return Double yaw angle.
 */
double getYawFromQuaternion(const Eigen::Quaterniond &quaternion);

/**
 * @brief Compute the angle between of a given vector in 2D and the unitary
 * vector (1,0).
 *
 * @param x double x coordinate of the vector.
 * @param y double y coordinate of the vector.
 * @return Double yaw angle.
 */
double getVector2DAngle(const double x, const double y);

/**
 * @brief Wrap angle to [0, 2*pi].
 *
 * @param theta double angle.
 * @return Double wrapped angle.
 */
double wrapAngle0To2Pi(const double theta);

/**
 * @brief Wrap angle to [-pi, pi].
 *
 * @param theta double angle.
 * @return Double wrapped angle.
 */
double wrapAnglePiToPi(const double theta);

/**
 * @brief Compute the minumun angle between two angles. Maximun error is pi.
 *
 * @param theta1 double first angle.
 * @param theta2 double second angle.
 * @return Double yaw difference.
 */
double angleMinError(const double theta1, const double theta2);

} // namespace frame

} // namespace cargo

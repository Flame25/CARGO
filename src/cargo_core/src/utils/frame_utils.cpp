#include "cargo_core/utils/frame_utils.hpp"

namespace cargo {
namespace frame {

Eigen::Vector3d transform(const tf2::Quaternion &quaternion,
                          const Eigen::Vector3d &vector) {
    tf2::Matrix3x3 rot(quaternion);
    Eigen::Matrix3d rot_eigen;
    rot_eigen << rot[0][0], rot[0][1], rot[0][2], rot[1][0], rot[1][1],
        rot[1][2], rot[2][0], rot[2][1], rot[2][2];

    return rot_eigen * vector;
}

Eigen::Vector3d transform(const float roll_angle, const float pitch_angle,
                          const float yaw_angle,
                          const Eigen::Vector3d &vector) {
    tf2::Quaternion q;
    q.setRPY(roll_angle, pitch_angle, yaw_angle);
    return transform(q, vector);
}

Eigen::Vector3d transform(const geometry_msgs::msg::Quaternion &quaternion,
                          const Eigen::Vector3d &vector) {
    tf2::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    return transform(q, vector);
}

Eigen::Vector3d transform(const Eigen::Quaterniond &quaternion,
                          const Eigen::Vector3d &vector) {
    tf2::Quaternion q(quaternion.x(), quaternion.y(), quaternion.z(),
                      quaternion.w());
    return transform(q, vector);
}

Eigen::Vector3d transformInverse(const tf2::Quaternion &quaternion,
                                 const Eigen::Vector3d &vector) {
    return transform(quaternion.inverse(), vector);
}

Eigen::Vector3d transformInverse(const float roll_angle,
                                 const float pitch_angle, const float yaw_angle,
                                 const Eigen::Vector3d &vector) {
    tf2::Quaternion q;
    q.setRPY(roll_angle, pitch_angle, yaw_angle);
    return transformInverse(q, vector);
}

Eigen::Vector3d
transformInverse(const geometry_msgs::msg::Quaternion &quaternion,
                 const Eigen::Vector3d &vector) {
    tf2::Quaternion q(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    return transformInverse(q, vector);
}

Eigen::Vector3d transformInverse(const Eigen::Quaterniond &quaternion,
                                 const Eigen::Vector3d &vector) {
    tf2::Quaternion q(quaternion.x(), quaternion.y(), quaternion.z(),
                      quaternion.w());
    return transformInverse(q, vector);
}

void quaternionToEuler(const tf2::Quaternion &quaternion, double &roll,
                       double &pitch, double &yaw) {
    tf2::Matrix3x3 rotation_matrix(quaternion);
    rotation_matrix.getRPY(roll, pitch, yaw);
    return;
}

void quaternionToEuler(const geometry_msgs::msg::Quaternion &quaternion,
                       double &roll, double &pitch, double &yaw) {
    tf2::Quaternion tf_quaternion;
    tf2::fromMsg(quaternion, tf_quaternion);
    quaternionToEuler(tf_quaternion, roll, pitch, yaw);
    return;
}

void quaternionToEuler(const Eigen::Quaterniond &quaternion, double &roll,
                       double &pitch, double &yaw) {
    tf2::Quaternion tf_quaternion(quaternion.x(), quaternion.y(),
                                  quaternion.z(), quaternion.w());
    quaternionToEuler(tf_quaternion, roll, pitch, yaw);
    return;
}

void eulerToQuaternion(const double roll, const double pitch, const double yaw,
                       tf2::Quaternion &quaternion) {
    tf2::Matrix3x3 rotation_matrix;
    rotation_matrix.setRPY(roll, pitch, yaw);
    rotation_matrix.getRotation(quaternion);
    return;
}

void eulerToQuaternion(const double roll, const double pitch, const double yaw,
                       geometry_msgs::msg::Quaternion &quaternion) {
    tf2::Quaternion tf_quaternion;
    eulerToQuaternion(roll, pitch, yaw, tf_quaternion);
    tf2::convert(tf_quaternion, quaternion);
    return;
}

void eulerToQuaternion(const double roll, const double pitch, const double yaw,
                       Eigen::Quaterniond &quaternion) {
    tf2::Quaternion tf_quaternion;
    eulerToQuaternion(roll, pitch, yaw, tf_quaternion);
    quaternion = Eigen::Quaterniond(tf_quaternion.w(), tf_quaternion.x(),
                                    tf_quaternion.y(), tf_quaternion.z());
    return;
}

double getYawFromQuaternion(const tf2::Quaternion &quaternion) {
    double roll, pitch, yaw;
    quaternionToEuler(quaternion, roll, pitch, yaw);
    return yaw;
}

double getYawFromQuaternion(const geometry_msgs::msg::Quaternion &quaternion) {
    double roll, pitch, yaw;
    quaternionToEuler(quaternion, roll, pitch, yaw);
    return yaw;
}

double getYawFromQuaternion(const Eigen::Quaterniond &quaternion) {
    double roll, pitch, yaw;
    quaternionToEuler(quaternion, roll, pitch, yaw);
    return yaw;
}

double getVector2DAngle(const double x, const double y) { return atan2f(y, x); }

double wrapAngle0To2Pi(const double theta) {
    double theta_wrapped = fmod(theta, 2.0 * M_PI);
    if (theta_wrapped < 0.0) {
        theta_wrapped += 2.0 * M_PI;
    }
    return theta_wrapped;
}

double wrapAnglePiToPi(const double theta) {
    double theta_wrapped = wrapAngle0To2Pi(theta);
    if (theta_wrapped >= M_PI) {
        theta_wrapped -= 2.0 * M_PI;
    }
    return theta_wrapped;
}

double angleMinError(const double theta1, const double theta2) {
    double theta1_wrapped = wrapAngle0To2Pi(theta1);
    double theta2_wrapped = wrapAngle0To2Pi(theta2);
    double error = theta1_wrapped - theta2_wrapped;
    if (error > M_PI) {
        error -= 2.0 * M_PI;
    } else if (error < -M_PI) {
        error += 2.0 * M_PI;
    }
    return error;
}

} // namespace frame

} // namespace cargo

#include "cargo_core/utils/tf_utils.hpp"

namespace cargo {
namespace tf {

using namespace std::chrono_literals; // NOLINT

std::string generateTfName(rclcpp::Node *node, std::string _frame_name) {
    return generateTfName(node->get_namespace(), _frame_name);
}

std::string generateTfName(const std::string &_namespace,
                           const std::string &_frame_name) {
    if (!_frame_name.size()) {
        throw std::runtime_error("Empty frame name");
    }
    if (_frame_name[0] == '/') {
        return _frame_name.substr(1);
    }
    if (!_namespace.size()) {
        RCLCPP_WARN(rclcpp::get_logger("tf_utils"),
                    "The frame name [%s] is not absolute and the node "
                    "namespace is empty. This could "
                    "lead to conflicts.",
                    _frame_name.c_str());
        return _frame_name;
    }
    std::string ns = _namespace;
    if (ns[0] == '/') {
        ns = ns.substr(1);
    }

    // If _frame_name until first '/' is equal to _namespace, then _frame_name
    // is absolute
    auto pos = _frame_name.find('/');
    if (pos != std::string::npos) {
        if (_frame_name.substr(0, pos) == ns) {
            return _frame_name;
        }
    }

    if (ns.empty()) {
        return _frame_name;
    }
    return ns + "/" + _frame_name;
}

geometry_msgs::msg::TransformStamped
getTransformation(const std::string &_frame_id,
                  const std::string &_child_frame_id, double _translation_x,
                  double _translation_y, double _translation_z, double _roll,
                  double _pitch, double _yaw) {
    geometry_msgs::msg::TransformStamped transformation;

    transformation.header.frame_id = _frame_id;
    transformation.child_frame_id = _child_frame_id;
    transformation.transform.translation.x = _translation_x;
    transformation.transform.translation.y = _translation_y;
    transformation.transform.translation.z = _translation_z;
    tf2::Quaternion q;
    q.setRPY(_roll, _pitch, _yaw);
    transformation.transform.rotation.x = q.x();
    transformation.transform.rotation.y = q.y();
    transformation.transform.rotation.z = q.z();
    transformation.transform.rotation.w = q.w();

    return transformation;
}

TfHandler::TfHandler(cargo::Node *_node) : node_(_node) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(_node->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        _node->get_node_base_interface(), _node->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Read tf_timeout_threshold from the parameter server
    double tf_timeout_threshold = 0.05;
    if (!_node->has_parameter("tf_timeout_threshold")) {
        // Declare the parameter
        _node->declare_parameter("tf_timeout_threshold", tf_timeout_threshold);
    }
    _node->get_parameter("tf_timeout_threshold", tf_timeout_threshold);
    setTfTimeoutThreshold(tf_timeout_threshold);
}

void TfHandler::setTfTimeoutThreshold(double tf_timeout_threshold) {
    setTfTimeoutThreshold(
        std::chrono::nanoseconds(static_cast<int>(tf_timeout_threshold * 1e9)));
}

void TfHandler::setTfTimeoutThreshold(
    const std::chrono::nanoseconds &tf_timeout_threshold) {
    tf_timeout_threshold_ = tf_timeout_threshold;
}

double TfHandler::getTfTimeoutThreshold() const {
    return std::chrono::duration<double>(tf_timeout_threshold_).count();
}

std::shared_ptr<tf2_ros::Buffer> TfHandler::getTfBuffer() const {
    return tf_buffer_;
}

geometry_msgs::msg::PoseStamped TfHandler::getPoseStamped(
    const std::string &target_frame, const std::string &source_frame,
    const tf2::TimePoint &time, const std::chrono::nanoseconds timeout) {
    // if-else needed for galactic
    geometry_msgs::msg::TransformStamped transform;
    if (timeout != std::chrono::nanoseconds::zero()) {
        transform = tf_buffer_->lookupTransform(
            target_frame, tf2_ros::fromMsg(node_->get_clock()->now()),
            source_frame, time, "earth", timeout);
    } else {
        transform = tf_buffer_->lookupTransform(
            target_frame, tf2::TimePointZero, source_frame, tf2::TimePointZero,
            "earth", timeout);
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = target_frame;
    pose.header.stamp = transform.header.stamp;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation.x = transform.transform.rotation.x;
    pose.pose.orientation.y = transform.transform.rotation.y;
    pose.pose.orientation.z = transform.transform.rotation.z;
    pose.pose.orientation.w = transform.transform.rotation.w;
    return pose;
}

geometry_msgs::msg::PoseStamped TfHandler::getPoseStamped(
    const std::string &target_frame, const std::string &source_frame,
    const rclcpp::Time &time, const std::chrono::nanoseconds timeout) {
    return getPoseStamped(target_frame, source_frame, tf2_ros::fromMsg(time),
                          timeout);
}

geometry_msgs::msg::PoseStamped
TfHandler::getPoseStamped(const std::string &target_frame,
                          const std::string &source_frame,
                          const tf2::TimePoint &time) {
    return getPoseStamped(target_frame, source_frame, time,
                          tf_timeout_threshold_);
}

geometry_msgs::msg::PoseStamped
TfHandler::getPoseStamped(const std::string &target_frame,
                          const std::string &source_frame,
                          const rclcpp::Time &time) {
    return getPoseStamped(target_frame, source_frame, tf2_ros::fromMsg(time),
                          tf_timeout_threshold_);
}

geometry_msgs::msg::QuaternionStamped TfHandler::getQuaternionStamped(
    const std::string &target_frame, const std::string &source_frame,
    const tf2::TimePoint &time, const std::chrono::nanoseconds timeout) {
    geometry_msgs::msg::TransformStamped transform;
    if (timeout != std::chrono::nanoseconds::zero()) {
        transform = tf_buffer_->lookupTransform(
            target_frame, tf2_ros::fromMsg(node_->get_clock()->now()),
            source_frame, time, "earth", timeout);
    } else {
        transform = tf_buffer_->lookupTransform(
            target_frame, tf2::TimePointZero, source_frame, tf2::TimePointZero,
            "earth", timeout);
    }

    geometry_msgs::msg::QuaternionStamped quaternion;
    quaternion.header.frame_id = target_frame;
    quaternion.header.stamp = transform.header.stamp;
    quaternion.quaternion.x = transform.transform.rotation.x;
    quaternion.quaternion.y = transform.transform.rotation.y;
    quaternion.quaternion.z = transform.transform.rotation.z;
    quaternion.quaternion.w = transform.transform.rotation.w;
    return quaternion;
}

geometry_msgs::msg::QuaternionStamped TfHandler::getQuaternionStamped(
    const std::string &target_frame, const std::string &source_frame,
    const rclcpp::Time &time, const std::chrono::nanoseconds timeout) {
    return getQuaternionStamped(target_frame, source_frame,
                                tf2_ros::fromMsg(time), timeout);
}

geometry_msgs::msg::QuaternionStamped
TfHandler::getQuaternionStamped(const std::string &target_frame,
                                const std::string &source_frame,
                                const tf2::TimePoint &time) {
    return getQuaternionStamped(target_frame, source_frame, time,
                                tf_timeout_threshold_);
}

geometry_msgs::msg::QuaternionStamped
TfHandler::getQuaternionStamped(const std::string &target_frame,
                                const std::string &source_frame,
                                const rclcpp::Time &time) {
    return getQuaternionStamped(target_frame, source_frame,
                                tf2_ros::fromMsg(time), tf_timeout_threshold_);
}

geometry_msgs::msg::TransformStamped
TfHandler::getTransform(const std::string &target_frame,
                        const std::string &source_frame,
                        const tf2::TimePoint &time) {
    return tf_buffer_->lookupTransform(target_frame, source_frame, time);
}

std::pair<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::TwistStamped>
TfHandler::getState(const geometry_msgs::msg::TwistStamped &_twist,
                    const std::string &_twist_target_frame,
                    const std::string &_pose_target_frame,
                    const std::string &_pose_source_frame,
                    const std::chrono::nanoseconds _timeout) {
    geometry_msgs::msg::TwistStamped twist =
        convert(_twist, _twist_target_frame, _timeout);

    geometry_msgs::msg::PoseStamped pose =
        getPoseStamped(_pose_target_frame, _pose_source_frame,
                       tf2_ros::fromMsg(twist.header.stamp), _timeout);
    return std::make_pair(pose, twist);
}

std::pair<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::TwistStamped>
TfHandler::getState(const geometry_msgs::msg::TwistStamped &_twist,
                    const std::string &_twist_target_frame,
                    const std::string &_pose_target_frame,
                    const std::string &_pose_source_frame) {
    return getState(_twist, _twist_target_frame, _pose_target_frame,
                    _pose_source_frame, tf_timeout_threshold_);
}

} // namespace tf
} // namespace cargo

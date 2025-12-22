#include "cargo_core/names/actions.hpp"
#include "cargo_core/names/services.hpp"
#include "cargo_core/names/topics.hpp"
#include <pybind11/pybind11.h>

PYBIND11_MODULE(cargo_names, m) {
    // TOPICS
    auto m_topics = m.def_submodule("topics", "Topics name defaults");

    auto m_global = m_topics.def_submodule("global");
    m_global.attr("alert_event") = cargo_names::topics::global::alert_event;

    auto m_sensor_measurements = m_topics.def_submodule("sensor_measurements");
    m_sensor_measurements.attr("imu") =
        cargo_names::topics::sensor_measurements::imu;
    m_sensor_measurements.attr("lidar") =
        cargo_names::topics::sensor_measurements::lidar;
    m_sensor_measurements.attr("gps") =
        cargo_names::topics::sensor_measurements::gps;
    m_sensor_measurements.attr("camera") =
        cargo_names::topics::sensor_measurements::camera;
    m_sensor_measurements.attr("battery") =
        cargo_names::topics::sensor_measurements::battery;
    m_sensor_measurements.attr("odom") =
        cargo_names::topics::sensor_measurements::odom;

    auto m_ground_truth = m_topics.def_submodule("ground_truth");
    m_ground_truth.attr("pose") = cargo_names::topics::ground_truth::pose;
    m_ground_truth.attr("twist") = cargo_names::topics::ground_truth::twist;

    auto m_self_localization = m_topics.def_submodule("self_localization");
    m_self_localization.attr("odom") =
        cargo_names::topics::self_localization::odom;
    m_self_localization.attr("pose") =
        cargo_names::topics::self_localization::pose;
    m_self_localization.attr("twist") =
        cargo_names::topics::self_localization::twist;

    auto m_motion_reference = m_topics.def_submodule("motion_reference");
    m_motion_reference.attr("pose") =
        cargo_names::topics::motion_reference::pose;
    m_motion_reference.attr("twist") =
        cargo_names::topics::motion_reference::twist;
    m_motion_reference.attr("trajectory") =
        cargo_names::topics::motion_reference::trajectory;
    m_motion_reference.attr("modify_waypoint") =
        cargo_names::topics::motion_reference::modify_waypoint;
    m_motion_reference.attr("traj_gen_info") =
        cargo_names::topics::motion_reference::traj_gen_info;

    auto m_actuator_command = m_topics.def_submodule("actuator_command");
    m_actuator_command.attr("pose") =
        cargo_names::topics::actuator_command::pose;
    m_actuator_command.attr("twist") =
        cargo_names::topics::actuator_command::twist;
    m_actuator_command.attr("thrust") =
        cargo_names::topics::actuator_command::thrust;
    m_actuator_command.attr("trajectory") =
        cargo_names::topics::actuator_command::trajectory;

    auto m_platform = m_topics.def_submodule("platform");
    m_platform.attr("info") = cargo_names::topics::platform::info;

    auto m_controller = m_topics.def_submodule("controller");
    m_controller.attr("info") = cargo_names::topics::controller::info;

    auto m_follow_target = m_topics.def_submodule("follow_target");
    m_follow_target.attr("info") = cargo_names::topics::follow_target::info;

    // SERVICES
    auto m_services = m.def_submodule("services", "Services name defaults");

    auto m_platform_serv = m_services.def_submodule("platform");
    m_platform_serv.attr("set_arming_state") =
        cargo_names::services::platform::set_arming_state;
    m_platform_serv.attr("set_offboard_mode") =
        cargo_names::services::platform::set_offboard_mode;
    m_platform_serv.attr("set_platform_control_mode") =
        cargo_names::services::platform::set_platform_control_mode;
    m_platform_serv.attr("takeoff") = cargo_names::services::platform::takeoff;
    m_platform_serv.attr("land") = cargo_names::services::platform::land;
    m_platform_serv.attr("set_platform_state_machine_event") =
        cargo_names::services::platform::set_platform_state_machine_event;
    m_platform_serv.attr("list_control_modes") =
        cargo_names::services::platform::list_control_modes;

    auto m_controller_serv = m_services.def_submodule("controller");
    m_controller_serv.attr("set_control_mode") =
        cargo_names::services::controller::set_control_mode;
    m_controller_serv.attr("list_control_modes") =
        cargo_names::services::controller::list_control_modes;

    // auto m_motion_reference_serv =
    // m_services.def_submodule("motion_reference");
    // m_motion_reference_serv.attr("send_traj_wayp") =
    //     cargo_names::services::motion_reference::send_traj_wayp;
    // m_motion_reference_serv.attr("add_traj_wayp") =
    //     cargo_names::services::motion_reference::add_traj_wayp;
    // m_motion_reference_serv.attr("set_traj_speed") =
    //     cargo_names::services::motion_reference::set_traj_speed;

    // auto m_gps = m_services.def_submodule("gps");
    // m_gps.attr("get_origin") = cargo_names::services::gps::get_origin;
    // m_gps.attr("set_origin") = cargo_names::services::gps::set_origin;
    // m_gps.attr("path_to_geopath") =
    // cargo_names::services::gps::path_to_geopath;
    // m_gps.attr("geopath_to_path") =
    // cargo_names::services::gps::geopath_to_path;

    // auto m_behavior = m_services.def_submodule("behavior");
    // m_behavior.attr("package_pickup") =
    //     cargo_names::services::behavior::package_pickup;
    // m_behavior.attr("package_unpick") =
    //     cargo_names::services::behavior::package_unpick;
    // m_behavior.attr("dynamic_land") =
    //     cargo_names::services::behavior::dynamic_land;
    // m_behavior.attr("dynamic_follower") =
    //     cargo_names::services::behavior::dynamic_follower;
    //
    // m_services.attr("set_speed") = cargo_names::services::set_speed;

    // ACTIONS
    auto m_actions = m.def_submodule("actions", "Actions name defaults");
    m_actions.attr("takeoff") = cargo_names::actions::behaviors::takeoff;
    // m_actions.attr("gotowaypoint") =
    // cargo_names::actions::behaviors::gotowaypoint;
    // m_actions.attr("followreference") =
    // cargo_names::actions::behaviors::followreference;
    // m_actions.attr("followpath") =
    // cargo_names::actions::behaviors::followpath;
    m_actions.attr("land") = cargo_names::actions::behaviors::land;
    // m_actions.attr("trajectorygenerator") =
    // cargo_names::actions::behaviors::trajectorygenerator;
}

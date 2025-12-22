#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "cargo_core/node.hpp"
#include "cargo_msgs/msg/platform_state_machine_event.hpp"
#include "cargo_msgs/msg/platform_status.hpp"
#include "cargo_msgs/srv/set_platform_state_machine_event.hpp"

namespace cargo {
/**
 * @brief Event type.
 *
 */
using Event = cargo_msgs::msg::PlatformStateMachineEvent;

/**
 * @brief Data Structure for defining the state machine transitions.
 */
struct StateMachineTransition {
    std::string transition_name;
    int8_t from_state_id;
    int8_t transition_id;
    int8_t to_state_id;
};

/**
 * @brief This class implements the Platform State Machine,
 * which is in charge of handling the state of the platform using a FSM (Finite
 * State Machine). This state machine consist on 6 states:
 *   - DISARMED -> The platform is not armed.
 *   - LANDED -> The platform is armed and landed.
 *   - TAKING_OFF -> The platform is taking off.
 *   - FLYING -> The platform is on air.
 *   - LANDING -> The platform is landing.
 *   - EMERGENCY -> The platform is in emergency mode.
 *
 * The events that can trigger the state machine are:
 *   - ARM
 *   - DISARM
 *   - TAKE_OFF
 *   - TOOK_OFF
 *   - LAND
 *   - LANDED
 *   - EMERGENCY
 *  TODO(miferco97): add figure of the state machine
 *  \image html test.jpg
 */

class PlatformStateMachine {
  public:
    /**
     * @brief Constructor of the Platform State Machine.
     * @param node_ptr Pointer to an aerostack2 node.
     */
    explicit PlatformStateMachine(cargo::Node *node);
    ~PlatformStateMachine();

    /**
     * @brief This function is in charge of handling the state machine.
     * @param event The event that triggers the state machine.
     * @return true If the event is valid in current State.
     */
    bool processEvent(const int8_t &event);
    /**
     * @brief This function is in charge of handling the state machine.
     * @param event The event that triggers the state machine.
     * @return true If the event is valid in current State.
     */
    bool processEvent(const Event &event);

    /**
     * @brief Get the Transition object
     *
     * @param current_state
     * @param event
     * @return StateMachineTransition
     */
    StateMachineTransition getTransition(const int8_t &current_state,
                                         const int8_t &event);

    /**
     * @brief This function returns the current state of the state machine
     * @return The current Platform Status of the state machine
     */
    inline cargo_msgs::msg::PlatformStatus getState() { return state_; }

    /**
     * @brief Set the State of the FSM to the desired state. (THIS MAY BE USED
     * ONLY FOR TESTING PURPOSES)
     * @param state
     */
    inline void setState(cargo_msgs::msg::PlatformStatus state) {
        state_ = state;
    }
    inline void setState(const int8_t &state) { state_.state = state; }

  private:
    std::vector<StateMachineTransition> transitions_;
    cargo_msgs::msg::PlatformStatus state_;
    cargo::Node *node_ptr_;

    /**
     * @brief in this function the state machine is created based on the
     * transitions. its called in the constructor of the class.
     */
    void defineTransitions();

    rclcpp::Service<cargo_msgs::srv::SetPlatformStateMachineEvent>::SharedPtr
        state_machine_event_srv_;

    void setStateMachineEventSrvCallback(
        const std::shared_ptr<
            cargo_msgs::srv::SetPlatformStateMachineEvent::Request>
            request,
        std::shared_ptr<cargo_msgs::srv::SetPlatformStateMachineEvent::Response>
            response);

    std::string eventToString(int8_t event);

    std::string stateToString(int8_t state);
};

} // namespace cargo

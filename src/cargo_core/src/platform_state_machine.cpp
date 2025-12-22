#include "cargo_core/platform_state_machine.hpp"

namespace cargo {
PlatformStateMachine::PlatformStateMachine(cargo::Node *node)
    : node_ptr_(node) {
    state_.state = cargo_msgs::msg::PlatformStatus::DISARMED;
    defineTransitions();

    // Initialize the srv server
    state_machine_event_srv_ =
        node_ptr_
            ->create_service<cargo_msgs::srv::SetPlatformStateMachineEvent>(
                node_ptr_->generate_local_name("state_machine_event"),
                std::bind(
                    &PlatformStateMachine::setStateMachineEventSrvCallback,
                    this, std::placeholders::_1, std::placeholders::_2));
}

PlatformStateMachine::~PlatformStateMachine() {
    state_machine_event_srv_.reset();
}

bool PlatformStateMachine::processEvent(const int8_t &event) {
    // Get the current state
    int8_t current_state = state_.state;

    // Get the transition that matches the current state and the event
    StateMachineTransition transition = getTransition(current_state, event);

    // If the transition is valid, change the state
    if (transition.transition_id == -11) {
        RCLCPP_WARN(node_ptr_->get_logger(), "Invalid transition: %s -> %s",
                    stateToString(current_state).c_str(),
                    eventToString(event).c_str());
        return false;
    }

    state_.state = transition.to_state_id;
    RCLCPP_INFO(node_ptr_->get_logger(), "Transition [%s] : New State [%s]",
                transition.transition_name.c_str(),
                stateToString(transition.to_state_id).c_str());

    return true;
}

bool PlatformStateMachine::processEvent(const Event &event) {
    return processEvent(event.event);
}

StateMachineTransition
PlatformStateMachine::getTransition(const int8_t &current_state,
                                    const int8_t &event) {
    StateMachineTransition transition;
    transition.transition_id = -11;
    for (size_t i = 0; i < transitions_.size(); i++) {
        if (transitions_[i].from_state_id == current_state &&
            transitions_[i].transition_id == event) {
            transition = transitions_[i];
            break;
        }
    }
    return transition;
}

void PlatformStateMachine::setStateMachineEventSrvCallback(
    const std::shared_ptr<
        cargo_msgs::srv::SetPlatformStateMachineEvent::Request>
        request,
    std::shared_ptr<cargo_msgs::srv::SetPlatformStateMachineEvent::Response>
        response) {
    response->success = processEvent(request->event);
    response->current_state = state_;
}

void PlatformStateMachine::defineTransitions() {
    transitions_.clear();
    transitions_.reserve(11);

    // INTIAL_STATE -> [TRANSITION] -> FINAL_STATE

    // DISARMED -> [ARM] -> ARMED
    transitions_.emplace_back(StateMachineTransition{
        "ARM", cargo_msgs::msg::PlatformStatus::DISARMED, Event::ARM,
        cargo_msgs::msg::PlatformStatus::LANDED});

    // LANDED -> [DISARM] -> DISARMED
    transitions_.emplace_back(StateMachineTransition{
        "DISARM", cargo_msgs::msg::PlatformStatus::LANDED, Event::DISARM,
        cargo_msgs::msg::PlatformStatus::DISARMED});

    // LANDED -> [TAKE_OFF] -> TAKING_OFF
    transitions_.emplace_back(StateMachineTransition{
        "TAKE_OFF", cargo_msgs::msg::PlatformStatus::LANDED, Event::TAKE_OFF,
        cargo_msgs::msg::PlatformStatus::TAKING_OFF});

    // TAKING_OFF -> [TOOK_OFF] -> FLYING
    transitions_.emplace_back(StateMachineTransition{
        "TOOK_OFF", cargo_msgs::msg::PlatformStatus::TAKING_OFF,
        Event::TOOK_OFF, cargo_msgs::msg::PlatformStatus::FLYING});

    // FLYING -> [LAND] -> LANDING
    transitions_.emplace_back(StateMachineTransition{
        "LAND", cargo_msgs::msg::PlatformStatus::FLYING, Event::LAND,
        cargo_msgs::msg::PlatformStatus::LANDING});

    // LANDING -> [LANDED] -> LANDED
    transitions_.emplace_back(StateMachineTransition{
        "LANDED", cargo_msgs::msg::PlatformStatus::LANDING, Event::LANDED,
        cargo_msgs::msg::PlatformStatus::LANDED});

    // EMERGENCY TRANSITIONS
    transitions_.emplace_back(StateMachineTransition{
        "EMERGENCY", cargo_msgs::msg::PlatformStatus::DISARMED,
        Event::EMERGENCY, cargo_msgs::msg::PlatformStatus::EMERGENCY});
    transitions_.emplace_back(StateMachineTransition{
        "EMERGENCY", cargo_msgs::msg::PlatformStatus::LANDED, Event::EMERGENCY,
        cargo_msgs::msg::PlatformStatus::EMERGENCY});
    transitions_.emplace_back(StateMachineTransition{
        "EMERGENCY", cargo_msgs::msg::PlatformStatus::TAKING_OFF,
        Event::EMERGENCY, cargo_msgs::msg::PlatformStatus::EMERGENCY});
    transitions_.emplace_back(StateMachineTransition{
        "EMERGENCY", cargo_msgs::msg::PlatformStatus::FLYING, Event::EMERGENCY,
        cargo_msgs::msg::PlatformStatus::EMERGENCY});
    transitions_.emplace_back(StateMachineTransition{
        "EMERGENCY", cargo_msgs::msg::PlatformStatus::LANDING, Event::EMERGENCY,
        cargo_msgs::msg::PlatformStatus::EMERGENCY});
}

std::string PlatformStateMachine::eventToString(int8_t event) {
    switch (event) {
    case cargo::Event::EMERGENCY:
        return "EMERGENCY";
        break;
    case cargo::Event::ARM:
        return "ARM";
        break;
    case cargo::Event::DISARM:
        return "DISARM";
        break;
    case cargo::Event::TAKE_OFF:
        return "TAKE_OFF";
        break;
    case cargo::Event::TOOK_OFF:
        return "TOOK_OFF";
        break;
    case cargo::Event::LAND:
        return "LAND";
        break;
    case cargo::Event::LANDED:
        return "LANDED";
        break;
    default:
        return "UNKNOWN";
        break;
    }
}

std::string PlatformStateMachine::stateToString(int8_t state) {
    switch (state) {
    case cargo_msgs::msg::PlatformStatus::EMERGENCY:
        return "EMERGENCY";
        break;
    case cargo_msgs::msg::PlatformStatus::DISARMED:
        return "DISARMED";
        break;
    case cargo_msgs::msg::PlatformStatus::LANDED:
        return "LANDED";
        break;
    case cargo_msgs::msg::PlatformStatus::TAKING_OFF:
        return "TAKING_OFF";
        break;
    case cargo_msgs::msg::PlatformStatus::FLYING:
        return "FLYING";
        break;
    case cargo_msgs::msg::PlatformStatus::LANDING:
        return "LANDING";
        break;
    default:
        return "UNKNOWN";
        break;
    }
}

} // namespace cargo

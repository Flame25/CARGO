#include "cargo_core/node.hpp"
#include "std_msgs/msg/string.hpp"

class ExampleNode : public cargo::Node {
  public:
    // Constructor matching the base class
    ExampleNode() : cargo::Node("talker_node") {}
    virtual ~ExampleNode() = default;

  protected:
    // 1. SETUP: Initialize publishers, subscribers, and parameters here
    CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override {
        // Call the built-in generate_local_name to make sure topic is unique to
        // this node
        std::string topic_name = generate_local_name("message");

        publisher_ =
            this->create_publisher<std_msgs::msg::String>(topic_name, 10);

        RCLCPP_INFO(this->get_logger(), "Configured! Publishing to: %s",
                    topic_name.c_str());
        return CallbackReturn::SUCCESS;
    }

    // 2. ACTIVATE: Start timers or enable hardware here
    CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override {
        double freq = this->get_loop_frequency();

        // Safety check: if freq is invalid (-1), default to something safe like
        // 2 Hz
        if (freq <= 0)
            freq = 2.0;

        // 2. Convert Hz to Duration
        auto duration = std::chrono::duration<double>(1.0 / freq);

        // 3. Create the timer using this calculated duration
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(duration),
            std::bind(&ExampleNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Activated with frequency: %.2f Hz",
                    freq);

        return CallbackReturn::SUCCESS;
    }

    // 3. DEACTIVATE: Stop timers or release hardware
    CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &state) override {
        timer_->cancel(); // Stop the timer
        RCLCPP_INFO(this->get_logger(), "Deactivated!");
        return CallbackReturn::SUCCESS;
    }

  private:
    void timer_callback() {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello from Cargo Node!";
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

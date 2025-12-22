#pragma once

// This node act as a bridge between rclcpp standard node and rclpp lifecycle
// This design follows https://design.ros2.org/articles/node_lifecycle.html

#include <chrono>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <rclcpp/create_timer.hpp>
#include <rclcpp/timer.hpp>

#include "cargo_core/rate.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/publisher_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#define AS2_RCLCPP_NODE 1
#define AS2_LIFECYLCE_NODE 2

#define AS2_NODE_FATHER AS2_RCLCPP_NODE
// #define AS2_NODE_FATHER AS2_LIFECYLCE_NODE

#if AS2_NODE_FATHER == AS2_RCLCPP_NODE
#define AS2_NODE_FATHER_TYPE rclcpp::Node
#elif AS2_NODE_FATHER == AS2_LIFECYLCE_NODE
#define AS2_NODE_FATHER_TYPE rclcpp_lifecycle::LifecycleNode
#endif

namespace cargo {
/**
 * @brief Basic Aerostack2 Node, it heritages all the functionality of an
 * rclcpp::Node
 */

class Node : public AS2_NODE_FATHER_TYPE {
  private:
    void init() {
        this->declare_parameter<float>("node_frequency", -1.0);
        this->get_parameter("node_frequency", loop_frequency_);
        RCLCPP_DEBUG(this->get_logger(), "node [%s] base frequency= %f",
                     this->get_name(), loop_frequency_);

        if (loop_frequency_ > 0.0) {
            loop_rate_ptr_ = std::make_shared<Rate>(loop_frequency_);
        }
    }

  public:
    // typedef std::shared_ptr<as2::Node> SharedPtr;
    /**
     * @brief Construct a new Node object
     *
     * @param name Node name
     */

    Node(const std::string &name, const std::string &ns,
         const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : AS2_NODE_FATHER_TYPE(name, ns, options) {
        RCLCPP_INFO(this->get_logger(),
                    "Construct with name [%s] and namespace [%s]", name.c_str(),
                    ns.c_str());
        init();
    }

    explicit Node(const std::string &name,
                  const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : AS2_NODE_FATHER_TYPE(name, options) {
        RCLCPP_INFO(this->get_logger(), "Construct with name [%s]",
                    name.c_str());
        init();
    }

#if AS2_NODE_FATHER == AS2_LIFECYLCE_NODE
    template <typename MessageT, typename AllocatorT = std::allocator<void>>
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<MessageT, AllocatorT>>
    create_publisher(
        const std::string &topic_name, const rclcpp::QoS &qos,
        const rclcpp::PublisherOptionsWithAllocator<AllocatorT> &options =
            rclcpp::PublisherOptionsWithAllocator<AllocatorT>()) {
        using PublisherT =
            rclcpp_lifecycle::LifecyclePublisher<MessageT, AllocatorT>;
        // RCLCPP_DEBUG(this->get_logger(), "PUB %s", topic_name.c_str());
        std::shared_ptr<PublisherT> pub =
            rclcpp::create_publisher<MessageT, AllocatorT, PublisherT>(
                *this, topic_name, qos, options);
        pub->on_activate();
        return pub;
    }

#elif AS2_NODE_FATHER == AS2_RCLCPP_NODE

  public:
    void configure() { this->on_configure(rclcpp_lifecycle::State()); }
    void activate() { this->on_activate(rclcpp_lifecycle::State()); }
    void deactivate() { this->on_deactivate(rclcpp_lifecycle::State()); }
    void cleanup() { this->on_cleanup(rclcpp_lifecycle::State()); }
    void shutdown() { this->on_shutdown(rclcpp_lifecycle::State()); }
    void error() { this->on_error(rclcpp_lifecycle::State()); }
#endif

    /**
     * @brief transform an string into local topic name inside drone namespace
     * and node namespace
     *
     * @param name source string
     * @return std::string  result name
     */
    std::string generate_local_name(const std::string &name);

    /**
     * @brief transform an string into global topic name inside drone namespace
     *
     * @param name source string
     * @return std::string result name
     */
    std::string generate_global_name(const std::string &name);

  protected:
    /**
     * @brief Callback for the activate state
     * @param state
     * @return CallbackReturn
     *
     */

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::
        LifecycleNodeInterface::CallbackReturn;
    virtual CallbackReturn
    on_activate(const rclcpp_lifecycle::State & = rclcpp_lifecycle::State()) {
        RCLCPP_DEBUG(this->get_logger(), "node [%s] on_activate",
                     this->get_name());
        return CallbackReturn::SUCCESS;
    }

    /**
     * @brief Callback for the deactivate state
     * @param state
     * @return CallbackReturn
     */
    virtual CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & = rclcpp_lifecycle::State()) {
        RCLCPP_DEBUG(this->get_logger(), "node [%s] on_deactivate",
                     this->get_name());
        return CallbackReturn::SUCCESS;
    }

    /**
     * @brief Callback for the configure state
     * @param state
     * @return CallbackReturn
     */

    virtual CallbackReturn
    on_configure(const rclcpp_lifecycle::State & = rclcpp_lifecycle::State()) {
        RCLCPP_DEBUG(this->get_logger(), "node [%s] on_configure",
                     this->get_name());
        return CallbackReturn::SUCCESS;
    }

    /**
     * @brief Callback for the cleanup state
     * @param state
     * @return CallbackReturn
     */
    virtual CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State & = rclcpp_lifecycle::State()) {
        RCLCPP_DEBUG(this->get_logger(), "node [%s] on_cleanup",
                     this->get_name());
        return CallbackReturn::SUCCESS;
    }

    /**
     * @brief Callback for the shutdown state
     * @param state
     * @return CallbackReturn
     */
    virtual CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State & = rclcpp_lifecycle::State()) {
        RCLCPP_DEBUG(this->get_logger(), "node [%s] on_shutdown",
                     this->get_name());
        return CallbackReturn::SUCCESS;
    }

    /**
     * @brief Callback for the error state
     * @param state
     * @return CallbackReturn
     */
    virtual CallbackReturn
    on_error(const rclcpp_lifecycle::State & = rclcpp_lifecycle::State()) {
        RCLCPP_ERROR(this->get_logger(), "node [%s] on_error",
                     this->get_name());
        return CallbackReturn::SUCCESS;
    }

  private:
    /**
     * @brief frequency of the spin cycle of the node
     */
    double loop_frequency_;
    std::shared_ptr<cargo::Rate> loop_rate_ptr_;

  public:
    /**
     * @brief create a timer with the node clock
     *
     * @return rclcpp::TimerBase::SharedPtr rclcpp timer using node clock
     */
    template <typename DurationRepT, typename DurationT, typename CallbackT>
    rclcpp::TimerBase::SharedPtr
    create_timer(std::chrono::duration<DurationRepT, DurationT> period,
                 CallbackT callback,
                 rclcpp::CallbackGroup::SharedPtr group = nullptr) {
        return rclcpp::create_timer(this, this->get_clock(), period,
                                    std::move(callback), group);
    }

    /**
     * @brief sleeps the node to ensure node_frecuency desired
     *
     * @return true the node is sleeping
     * @return false the node is not sleeping, this means that desired frequency
     * is not reached
     */
    bool sleep() {
        if (loop_rate_ptr_) {
            return loop_rate_ptr_->sleep();
        } else {
            throw std::runtime_error(
                "Node::sleep() called but no node_frequency defined");
        }
    }

    /**
     * @brief Get the loop frequency object
     *
     * @return double frequency in Hz
     */
    inline double get_loop_frequency() { return loop_frequency_; }

    /**
     * @brief Set the loop frequency of the node
     *
     * @return true if using default frequency or set successfully
     */
    bool preset_loop_frequency(double frequency) {
        if (frequency <= 0) {
            return true; // default frequency is -1
        }
        if (loop_rate_ptr_) {
            RCLCPP_INFO(this->get_logger(),
                        "Preset Loop Frequency [%d Hz] was overwrite by "
                        "launcher params to %d",
                        (int)frequency, (int)loop_frequency_);
            return false;
        }
        loop_frequency_ = frequency;
        loop_rate_ptr_ = std::make_shared<Rate>(loop_frequency_);
        return true;
    }
};

} // namespace cargo

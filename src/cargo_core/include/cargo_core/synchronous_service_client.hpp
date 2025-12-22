#pragma once

#include <memory>
#include <string>

#include "cargo_core/node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cargo {

template <class ServiceT>
/**
 * @brief Class for handling synchronous service clients in ROS2
 *       without taking care about the spin() method
 */
class SynchronousServiceClient {
    typedef typename ServiceT::Request RequestT;
    typedef typename ServiceT::Response ResponseT;
    std::string service_name_;
    cargo::Node *node_;

    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    std::shared_ptr<rclcpp::Client<ServiceT>> service_client_;

  public:
    using SharedPtr = std::shared_ptr<SynchronousServiceClient<ServiceT>>;

    /**
     * @brief Constructor
     * @param service_name Name of the service
     */
    SynchronousServiceClient(std::string service_name, cargo::Node *node)
        : service_name_(service_name), node_(node) {
        callback_group_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive, false);
        callback_group_executor_.add_callback_group(
            callback_group_, node_->get_node_base_interface());
        service_client_ = node_->create_client<ServiceT>(
            service_name, rmw_qos_profile_services_default, callback_group_);
    }

    /**
     * @brief Send Request synchronously to the service
     * @param request Request to be sent to the service
     * @param respone Response received from the service
     * @param wait_time Time to wait for the service to be available in seconds
     * @return True if the service was called successfully, false otherwise
     */

    bool sendRequest(const RequestT &req, ResponseT &resp, int wait_time = 0) {
        auto resp_ptr = std::make_shared<ResponseT>(resp);
        if (!sendRequest(std::make_shared<RequestT>(req), resp_ptr,
                         wait_time)) {
            return false;
        }
        resp = *resp_ptr.get();
        return true;
    }

    /**
     * @brief Send Request synchronously to the service
     * @param request Request to be sent to the service
     * @param respone Response received from the service
     * @param wait_time Time to wait for the service to be available in seconds
     * @return True if the service was called successfully, false otherwise
     */
    bool sendRequest(const std::shared_ptr<RequestT> &req,
                     std::shared_ptr<ResponseT> &resp, int wait_time = 0) {
        if (wait_time <= 0) {
            while (
                !service_client_->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(
                        node_->get_logger(),
                        "interrupted while waiting for the service. exiting.");
                    return false;
                }
                RCLCPP_INFO(node_->get_logger(),
                            "service: %s not available, waiting again...",
                            service_name_.c_str());
            }
        } else {
            if (!service_client_->wait_for_service(
                    std::chrono::seconds(wait_time))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(
                        node_->get_logger(),
                        "interrupted while waiting for the service. exiting.");
                    return false;
                }
                RCLCPP_WARN(node_->get_logger(),
                            "service: %s not available, returning False ",
                            service_name_.c_str());
                return false;
            }
        }

        auto result = service_client_->async_send_request(req);
        if (callback_group_executor_.spin_until_future_complete(result) !=
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_WARN(node_->get_logger(),
                        "failed to receive response from service '%s'",
                        service_name_.c_str());
            return false;
        }

        resp = result.get();
        return true;
    } // namespace cargo

  protected:
}; // namespace cargo

} // namespace cargo

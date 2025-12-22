#include <chrono>
#include <std_srvs/srv/set_bool.hpp>

#include "cargo_behavior/behavior_server.hpp"
#include "cargo_core/names/services.hpp"
#include "takeoff_behavior/takeoff_base.hpp"

namespace takeoff_plugin_platform {

class Plugin : public takeoff_base::TakeoffBase {
  public:
    void ownInit() {
        platform_takeoff_cli_ =
            node_ptr_->create_client<std_srvs::srv::SetBool>(
                cargo_names::services::platform::takeoff);
        platform_takeoff_request_ =
            std::make_shared<std_srvs::srv::SetBool::Request>();
        platform_takeoff_request_->data = true;
        return;
    }

    bool own_activate(cargo_msgs::action::Takeoff::Goal &_goal) override {
        if (!platform_takeoff_cli_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_ptr_->get_logger(),
                         "Platform takeoff service not available");
            return false;
        }

        platform_takeoff_future_ =
            platform_takeoff_cli_->async_send_request(platform_takeoff_request_)
                .share();

        if (!platform_takeoff_future_.valid()) {
            RCLCPP_ERROR(node_ptr_->get_logger(), "Request could not be sent");
            return false;
        }

        RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff Activating");
        return true;
    }

    bool own_deactivate(const std::shared_ptr<std::string> &message) override {
        RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff can not be cancelled");
        return false;
    }

    void
    own_execution_end(const cargo_behavior::ExecutionStatus &state) override {
        RCLCPP_INFO(node_ptr_->get_logger(), "Takeoff end");
        return;
    }

    cargo_behavior::ExecutionStatus own_run() override {
        if (platform_takeoff_future_.valid() &&
            platform_takeoff_future_.wait_for(std::chrono::seconds(0)) ==
                std::future_status::ready) {
            auto result = platform_takeoff_future_.get();
            if (result->success) {
                result_.takeoff_success = true;
                return cargo_behavior::ExecutionStatus::SUCCESS;
            } else {
                result_.takeoff_success = false;
                return cargo_behavior::ExecutionStatus::FAILURE;
            }
        }
        return cargo_behavior::ExecutionStatus::RUNNING;
    }

  private:
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr platform_takeoff_cli_;

    std_srvs::srv::SetBool::Request::SharedPtr platform_takeoff_request_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture
        platform_takeoff_future_;
}; // Plugin class
} // namespace takeoff_plugin_platform

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(takeoff_plugin_platform::Plugin,
                       takeoff_base::TakeoffBase)

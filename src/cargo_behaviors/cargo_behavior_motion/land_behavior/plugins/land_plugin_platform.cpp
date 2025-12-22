#include <chrono>
#include <std_srvs/srv/set_bool.hpp>

#include "cargo_behavior/behavior_server.hpp"
#include "cargo_core/names/services.hpp"
#include "land_behavior/land_base.hpp"

namespace land_plugin_platform {

class Plugin : public land_base::LandBase {
  public:
    void ownInit() {
        platform_land_cli_ = node_ptr_->create_client<std_srvs::srv::SetBool>(
            cargo_names::services::platform::land);
        platform_land_request_ =
            std::make_shared<std_srvs::srv::SetBool::Request>();
        platform_land_request_->data = true;
        return;
    }

    bool own_activate(cargo_msgs::action::Land::Goal &_goal) override {
        if (!platform_land_cli_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node_ptr_->get_logger(),
                         "Platform land service not available");
            return false;
        }

        platform_land_future_ =
            platform_land_cli_->async_send_request(platform_land_request_)
                .share();

        if (!platform_land_future_.valid()) {
            RCLCPP_ERROR(node_ptr_->get_logger(), "Request could not be sent");
            return false;
        }
        return true;
    }

    bool own_deactivate(const std::shared_ptr<std::string> &message) override {
        RCLCPP_INFO(node_ptr_->get_logger(), "Land can not be cancelled");
        return false;
    }

    void
    own_execution_end(const cargo_behavior::ExecutionStatus &state) override {
        RCLCPP_INFO(node_ptr_->get_logger(), "Land end");
        if (state != cargo_behavior::ExecutionStatus::SUCCESS) {
            sendHover();
        }
        return;
    }

    cargo_behavior::ExecutionStatus own_run() override {
        if (platform_land_future_.valid() &&
            platform_land_future_.wait_for(std::chrono::seconds(0)) ==
                std::future_status::ready) {
            auto result = platform_land_future_.get();
            if (result->success) {
                result_.land_success = true;
                return cargo_behavior::ExecutionStatus::SUCCESS;
            } else {
                result_.land_success = false;
                return cargo_behavior::ExecutionStatus::FAILURE;
            }
        }
        return cargo_behavior::ExecutionStatus::RUNNING;
    }

  private:
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr platform_land_cli_;
    std_srvs::srv::SetBool::Request::SharedPtr platform_land_request_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture platform_land_future_;
}; // Plugin class
} // namespace land_plugin_platform

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(land_plugin_platform::Plugin, land_base::LandBase)

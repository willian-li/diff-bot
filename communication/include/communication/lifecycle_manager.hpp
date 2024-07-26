#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

using namespace std::chrono_literals;

class LifecycleServiceClient
{
public:
  explicit LifecycleServiceClient(
    const std::shared_ptr<rclcpp::Node> & node,
    const std::string & lifecycle_node)
  : node_(node), lifecycle_node_(lifecycle_node)
  {}

  void init()
  {
    client_get_state_ = node_->create_client<lifecycle_msgs::srv::GetState>(
      lifecycle_node_ + "/get_state");
    client_change_state_ = node_->create_client<lifecycle_msgs::srv::ChangeState>(
      lifecycle_node_ + "/change_state");

    RCLCPP_INFO(node_->get_logger(), "lifecycle_node_: %s ", lifecycle_node_.c_str());
  }

  unsigned int get_state(std::chrono::seconds time_out = 10s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    if (!client_get_state_->wait_for_service(time_out)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Service %s is not available.",
        client_get_state_->get_service_name());
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    auto future_result = client_get_state_->async_send_request(request).future.share();
    auto future_status = wait_for_result(future_result, time_out);

    if (future_status != std::future_status::ready) {
      RCLCPP_ERROR(node_->get_logger(), "Server time out while getting current state for node %s", lifecycle_node_.c_str());
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    if (future_result.get()) {
      RCLCPP_INFO(node_->get_logger(), "Node %s has current state %s.", lifecycle_node_.c_str(), future_result.get()->current_state.label.c_str());
      return future_result.get()->current_state.id;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get current state for node %s", lifecycle_node_.c_str());
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
  }

  bool change_state(std::uint8_t transition, std::chrono::seconds time_out = 3s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    if (!client_change_state_->wait_for_service(time_out)) {
      RCLCPP_ERROR(node_->get_logger(), "Service %s is not available.", client_change_state_->get_service_name());
      return false;
    }

    auto future_result = client_change_state_->async_send_request(request).future.share();
    auto future_status = wait_for_result(future_result, time_out);

    if (future_status != std::future_status::ready) {
      RCLCPP_ERROR(node_->get_logger(), "Server time out while getting current state for node %s", lifecycle_node_.c_str());
      return false;
    }

    if (future_result.get()->success) {
      RCLCPP_INFO(node_->get_logger(), "Transition %d successfully triggered.", static_cast<int>(transition));
      return true;
    } else {
      RCLCPP_WARN(node_->get_logger(), "Failed to trigger transition %u", static_cast<unsigned int>(transition));
      return false;
    }
  }

private:
  template<typename FutureT, typename WaitTimeT>
  std::future_status wait_for_result(FutureT & future, WaitTimeT time_to_wait)
  {
    auto end = std::chrono::steady_clock::now() + time_to_wait;
    std::chrono::milliseconds wait_period(100);
    std::future_status status = std::future_status::timeout;
    do {
      auto now = std::chrono::steady_clock::now();
      auto time_left = end - now;
      if (time_left <= std::chrono::seconds(0)) { break; }
      status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
    } while (rclcpp::ok() && status != std::future_status::ready);
    return status;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
  const std::string lifecycle_node_;
};
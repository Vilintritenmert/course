#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "underground_world/msg/enemy_down.hpp"
#include "underground_world/srv/payload_trigger.hpp"

namespace {

using underground_world::msg::EnemyDown;
using underground_world::srv::PayloadTrigger;

constexpr auto kTriggerService = "/payload/trigger";
constexpr auto kEnemyDownTopic = "/payload/enemy_down";

class PayloadActionNode final : public rclcpp::Node {
public:
  PayloadActionNode()
    : Node("payload_action_node")
  {
    enemy_down_pub_ = create_publisher<EnemyDown>(kEnemyDownTopic, rclcpp::QoS{10});
    trigger_srv_ = create_service<PayloadTrigger>(
      kTriggerService,
      [this](const std::shared_ptr<PayloadTrigger::Request> request,
             std::shared_ptr<PayloadTrigger::Response> response) { handle_trigger(*request, *response); });

    RCLCPP_INFO(get_logger(), "payload action service is ready");
  }

private:
  void handle_trigger(const PayloadTrigger::Request& request, PayloadTrigger::Response& response)
  {
    EnemyDown event;
    event.contact_id = request.contact_id;
    event.x = request.x;
    event.y = request.y;
    enemy_down_pub_->publish(event);

    response.accepted = true;
    response.reason = "enemy_down event published";

    RCLCPP_INFO(get_logger(),
                "trigger accepted contact_id=%d position=(%d,%d)",
                request.contact_id,
                request.x,
                request.y);
  }

  rclcpp::Publisher<EnemyDown>::SharedPtr enemy_down_pub_;
  rclcpp::Service<PayloadTrigger>::SharedPtr trigger_srv_;
};

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PayloadActionNode>());
  rclcpp::shutdown();
  return 0;
}

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <future>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "underground_world/msg/cell_observation.hpp"
#include "underground_world/msg/local_scan.hpp"
#include "underground_world/msg/move_command.hpp"
#include "underground_world/msg/robot_result.hpp"
#include "underground_world/msg/student_status.hpp"
#include "underground_world/srv/payload_trigger.hpp"
#include "underground_world/state_qos.hpp"

namespace {

using underground_world::msg::CellObservation;
using underground_world::msg::LocalScan;
using underground_world::msg::MoveCommand;
using underground_world::msg::RobotResult;
using underground_world::msg::StudentStatus;
using underground_world::srv::PayloadTrigger;

constexpr auto kScanTopic = "/robot/local_scan";
constexpr auto kMoveTopic = "/robot/cmd_move";
constexpr auto kResultTopic = "/robot/result";
constexpr auto kStatusTopic = "/student/status";
constexpr auto kTriggerService = "/payload/trigger";

struct Position {
  int x = 0;
  int y = 0;
};

bool operator==(const Position lhs, const Position rhs)
{
  return lhs.x == rhs.x && lhs.y == rhs.y;
}

bool operator<(const Position lhs, const Position rhs)
{
  return std::tie(lhs.y, lhs.x) < std::tie(rhs.y, rhs.x);
}

struct Step {
  Position delta;
  std::uint8_t direction;
};

const std::array<Step, 4>& steps()
{
  static constexpr std::array<Step, 4> kSteps{{
    Step{Position{0, -1}, MoveCommand::UP},
    Step{Position{1, 0}, MoveCommand::RIGHT},
    Step{Position{0, 1}, MoveCommand::DOWN},
    Step{Position{-1, 0}, MoveCommand::LEFT},
  }};
  return kSteps;
}

Position operator+(const Position lhs, const Position rhs)
{
  return Position{lhs.x + rhs.x, lhs.y + rhs.y};
}

Position position_of(const CellObservation& cell)
{
  return Position{cell.x, cell.y};
}

bool is_passable_cell(const std::string& cell_type)
{
  return cell_type == "." || cell_type == "S" || cell_type == "C" || cell_type == "x";
}

class MissionExplorerNode final : public rclcpp::Node {
public:
  MissionExplorerNode()
    : Node("mission_explorer_node")
  {
    const auto state_qos = underground_world::make_state_qos();
    move_pub_ = create_publisher<MoveCommand>(kMoveTopic, rclcpp::QoS{10});
    status_pub_ = create_publisher<StudentStatus>(kStatusTopic, state_qos);
    trigger_client_ = create_client<PayloadTrigger>(kTriggerService);

    scan_sub_ =
      create_subscription<LocalScan>(kScanTopic, state_qos, [this](const LocalScan::SharedPtr msg) { on_scan(*msg); });
    result_sub_ =
      create_subscription<RobotResult>(kResultTopic, state_qos, [this](const RobotResult::SharedPtr msg) { on_result(*msg); });
    tick_timer_ = create_wall_timer(std::chrono::milliseconds{100}, [this]() { tick(); });

    publish_status(StudentStatus::EXPLORING);
    RCLCPP_INFO(get_logger(), "mission explorer is ready");
  }

private:
  void on_scan(const LocalScan& scan)
  {
    current_ = Position{scan.robot_x, scan.robot_y};
    have_scan_ = true;

    for (const auto& cell : scan.cells) {
      const auto position = position_of(cell);
      known_cells_[position] = cell.cell_type;
      if (is_passable_cell(cell.cell_type)) {
        passable_cells_.insert(position);
      }
      if (cell.cell_type == "C") {
        visible_contacts_[cell.contact_id] = position;
      }
      if (cell.cell_type == "x") {
        processed_contacts_.insert(cell.contact_id);
        requested_contacts_.erase(cell.contact_id);
        visible_contacts_.erase(cell.contact_id);
      }
    }

    if (awaiting_move_ && current_ == expected_after_move_) {
      awaiting_move_ = false;
    }

    plan_needed_ = true;
  }

  void on_result(const RobotResult& result)
  {
    if (result.mission_result == "SUCCESS") {
      finished_ = true;
      publish_status(StudentStatus::DONE);
      RCLCPP_INFO(get_logger(), "mission completed: %s", result.reason.c_str());
    }
    else if (result.mission_result != "RUNNING") {
      finished_ = true;
      publish_status(StudentStatus::FAILED);
      RCLCPP_ERROR(get_logger(), "mission failed in world node: %s", result.reason.c_str());
    }
  }

  void tick()
  {
    if (finished_) {
      return;
    }

    poll_trigger_response();

    if (!have_scan_ || awaiting_move_ || trigger_in_flight_) {
      return;
    }

    if (!requested_contacts_.empty()) {
      publish_status(StudentStatus::ENGAGING);
      return;
    }

    if (engage_visible_contact()) {
      return;
    }

    if (!plan_needed_) {
      return;
    }

    plan_needed_ = false;
    if (const auto direction = next_exploration_move(); direction.has_value()) {
      publish_move(*direction);
      return;
    }

    publish_status(StudentStatus::DONE);
    finished_ = true;
    RCLCPP_INFO(get_logger(), "no unexplored reachable frontier remains");
  }

  void poll_trigger_response()
  {
    if (!trigger_in_flight_) {
      return;
    }

    const auto state = trigger_future_.wait_for(std::chrono::seconds{0});
    if (state != std::future_status::ready) {
      return;
    }

    const auto response = trigger_future_.get();
    if (!response->accepted) {
      publish_status(StudentStatus::FAILED);
      finished_ = true;
      trigger_in_flight_ = false;
      RCLCPP_ERROR(get_logger(), "payload trigger rejected: %s", response->reason.c_str());
      return;
    }

    requested_contacts_.insert(trigger_contact_id_);
    trigger_in_flight_ = false;
    publish_status(StudentStatus::EXPLORING);
    plan_needed_ = true;
  }

  bool engage_visible_contact()
  {
    const auto candidate = std::find_if(visible_contacts_.begin(), visible_contacts_.end(), [this](const auto& item) {
      return processed_contacts_.find(item.first) == processed_contacts_.end() &&
             requested_contacts_.find(item.first) == requested_contacts_.end();
    });
    if (candidate == visible_contacts_.end()) {
      return false;
    }

    if (!trigger_client_->service_is_ready()) {
      publish_status(StudentStatus::ENGAGING);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "waiting for payload trigger service");
      return true;
    }

    auto request = std::make_shared<PayloadTrigger::Request>();
    request->contact_id = candidate->first;
    request->x = candidate->second.x;
    request->y = candidate->second.y;

    trigger_contact_id_ = candidate->first;
    trigger_future_ = trigger_client_->async_send_request(request);
    trigger_in_flight_ = true;
    publish_status(StudentStatus::ENGAGING);
    RCLCPP_INFO(get_logger(),
                "triggering contact_id=%d position=(%d,%d)",
                request->contact_id,
                request->x,
                request->y);
    return true;
  }

  std::optional<std::uint8_t> next_exploration_move() const
  {
    const auto path = path_to_nearest_frontier();
    if (path.size() < 2) {
      return std::nullopt;
    }

    const auto next = path.at(1);
    for (const auto step : steps()) {
      if (current_ + step.delta == next) {
        return step.direction;
      }
    }
    return std::nullopt;
  }

  std::vector<Position> path_to_nearest_frontier() const
  {
    std::queue<Position> pending;
    std::set<Position> visited;
    std::map<Position, Position> parent;

    pending.push(current_);
    visited.insert(current_);

    while (!pending.empty()) {
      const auto here = pending.front();
      pending.pop();

      if (has_unknown_cardinal_neighbor(here)) {
        return reconstruct_path(parent, here);
      }

      for (const auto step : steps()) {
        const auto next = here + step.delta;
        if (!can_traverse(next) || visited.find(next) != visited.end()) {
          continue;
        }
        visited.insert(next);
        parent[next] = here;
        pending.push(next);
      }
    }

    return {};
  }

  bool has_unknown_cardinal_neighbor(const Position position) const
  {
    for (const auto step : steps()) {
      const auto neighbor = position + step.delta;
      if (known_cells_.find(neighbor) == known_cells_.end()) {
        return true;
      }
    }
    return false;
  }

  bool can_traverse(const Position position) const
  {
    const auto iter = known_cells_.find(position);
    if (iter == known_cells_.end() || !is_passable_cell(iter->second)) {
      return false;
    }
    if (iter->second == "C") {
      const auto contact_iter = std::find_if(visible_contacts_.begin(), visible_contacts_.end(), [position](const auto& item) {
        return item.second == position;
      });
      return contact_iter == visible_contacts_.end() || processed_contacts_.find(contact_iter->first) != processed_contacts_.end();
    }
    return true;
  }

  static std::vector<Position> reconstruct_path(const std::map<Position, Position>& parent, const Position goal)
  {
    std::vector<Position> path{goal};
    auto current = goal;
    while (true) {
      const auto iter = parent.find(current);
      if (iter == parent.end()) {
        break;
      }
      current = iter->second;
      path.push_back(current);
    }
    std::reverse(path.begin(), path.end());
    return path;
  }

  void publish_move(const std::uint8_t direction)
  {
    MoveCommand msg;
    msg.direction = direction;
    move_pub_->publish(msg);

    for (const auto step : steps()) {
      if (step.direction == direction) {
        expected_after_move_ = current_ + step.delta;
        break;
      }
    }
    awaiting_move_ = true;
    publish_status(StudentStatus::EXPLORING);
  }

  void publish_status(const std::uint8_t state)
  {
    StudentStatus msg;
    msg.state = state;
    status_pub_->publish(msg);
  }

  rclcpp::Publisher<MoveCommand>::SharedPtr move_pub_;
  rclcpp::Publisher<StudentStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<LocalScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<RobotResult>::SharedPtr result_sub_;
  rclcpp::Client<PayloadTrigger>::SharedPtr trigger_client_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::Client<PayloadTrigger>::SharedFuture trigger_future_;

  std::map<Position, std::string> known_cells_;
  std::set<Position> passable_cells_;
  std::map<int, Position> visible_contacts_;
  std::set<int> requested_contacts_;
  std::set<int> processed_contacts_;
  Position current_;
  Position expected_after_move_;
  int trigger_contact_id_ = 0;
  bool have_scan_ = false;
  bool awaiting_move_ = false;
  bool trigger_in_flight_ = false;
  bool plan_needed_ = true;
  bool finished_ = false;
};

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionExplorerNode>());
  rclcpp::shutdown();
  return 0;
}

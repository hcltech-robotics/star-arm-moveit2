#include <memory>
#include <mutex>
#include <string>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

class JointTrajectoryActionBridgeNode : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  JointTrajectoryActionBridgeNode()
  : Node("joint_trajectory_action_bridge")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/arm_controller/joint_trajectory");
    action_name_ = declare_parameter<std::string>("action_name", "/arm_controller/follow_joint_trajectory");
    queue_depth_ = declare_parameter<int>("queue_depth", 10);

    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        get_node_base_interface(),
        get_node_graph_interface(),
        get_node_logging_interface(),
        get_node_waitables_interface(),
        action_name_);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(queue_depth_)))
                   .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
        input_topic_, qos,
        [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) { onTrajectory(msg); });

    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {
      if (!action_client_->wait_for_action_server(std::chrono::seconds(0)))
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "Waiting for action server %s for trajectory bridging", action_name_.c_str());
      }
    });
  }

private:
  void onTrajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_traj_ = *msg;
    has_pending_ = true;
    if (!goal_in_flight_)
    {
      sendPendingLocked();
    }
  }

  void sendPendingLocked()
  {
    if (!has_pending_)
    {
      return;
    }
    if (!action_client_->wait_for_action_server(std::chrono::seconds(0)))
    {
      return;
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory = latest_traj_;
    has_pending_ = false;
    goal_in_flight_ = true;

    rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions options;
    options.goal_response_callback = [this](std::shared_ptr<GoalHandle> goal_handle) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!goal_handle)
      {
        goal_in_flight_ = false;
        RCLCPP_WARN(get_logger(), "Trajectory goal rejected by %s", action_name_.c_str());
        if (has_pending_)
        {
          sendPendingLocked();
        }
      }
    };
    options.result_callback = [this](const GoalHandle::WrappedResult&) {
      std::lock_guard<std::mutex> lock(mutex_);
      goal_in_flight_ = false;
      if (has_pending_)
      {
        sendPendingLocked();
      }
    };

    action_client_->async_send_goal(goal, options);
  }

  std::string input_topic_;
  std::string action_name_;
  int queue_depth_{ 10 };

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  trajectory_msgs::msg::JointTrajectory latest_traj_;
  bool has_pending_{ false };
  bool goal_in_flight_{ false };
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointTrajectoryActionBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

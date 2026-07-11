#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <controller_manager_msgs/srv/switch_controller.hpp>

#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;

using Vec3 = Eigen::Vector3d;
using Quat = Eigen::Quaterniond;

namespace
{
rclcpp::QoS latchedQos()
{
  rclcpp::QoS qos(1);
  qos.transient_local();
  qos.reliable();
  return qos;
}
}  // namespace

class VerticalMixerPtp : public rclcpp::Node
{
public:
  VerticalMixerPtp()
  : Node("vertical_mixer_ptp")
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    // IK/plan for the end_effector_link (not tool0), per the tuning decision.
    tool_frame_ = declare_parameter<std::string>("tool_frame", "end_effector_link");
    move_group_name_ = declare_parameter<std::string>("move_group", "ur_manipulator");

    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names",
      {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"});

    waypoints_topic_ = declare_parameter<std::string>(
      "waypoints_topic", "/ur10e_vision/waypoints");
    arm_states_topic_ = declare_parameter<std::string>("arm_states_topic", "/joint_states");
    compliance_controller_ = declare_parameter<std::string>(
      "compliance_controller", "cartesian_compliance_controller");
    trajectory_controller_ = declare_parameter<std::string>(
      "trajectory_controller", "joint_trajectory_controller");

    standoff_dist_ = declare_parameter<double>("standoff_dist", 0.20);

    planning_time_       = declare_parameter<double>("planning_time", 8.0);
    plan_velocity_scale_ = declare_parameter<double>("plan_velocity_scale", 0.35);
    plan_accel_scale_    = declare_parameter<double>("plan_accel_scale", 0.35);
    pipeline_id_         = declare_parameter<std::string>("pipeline_id", "move_group");

    require_mixer_still_ = declare_parameter<bool>("require_mixer_still", true);
    mixer_states_topic_  = declare_parameter<std::string>(
      "mixer_states_topic", "/mixer/joint_states");
    mixer_still_speed_   = declare_parameter<double>("mixer_still_speed", 0.02);   // rad/s
    mixer_still_timeout_ = declare_parameter<double>("mixer_still_timeout", 20.0); // s
    mixer_stale_sec_     = declare_parameter<double>("mixer_stale_sec", 0.5);      // s

    wp_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      waypoints_topic_, latchedQos(),
      std::bind(&VerticalMixerPtp::wpCb, this, _1));

    arm_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      arm_states_topic_, rclcpp::QoS(10),
      std::bind(&VerticalMixerPtp::armCb, this, _1));

    mixer_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      mixer_states_topic_, rclcpp::SensorDataQoS(),
      std::bind(&VerticalMixerPtp::mixerCb, this, _1));

    ik_client_ = create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");
    move_client_ = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
      this, "/move_action");

    timer_ = create_wall_timer(50ms, std::bind(&VerticalMixerPtp::update, this));

    RCLCPP_INFO(get_logger(),
      "vertical_mixer_ptp up. IK link=%s, standoff=%.2f m. Waiting for %s.",
      tool_frame_.c_str(), standoff_dist_, waypoints_topic_.c_str());
  }

  ~VerticalMixerPtp() override
  {
    if (move_thread_.joinable())
      move_thread_.join();
  }

private:
  enum class Mode { WAIT_DATA, MOVING, DONE };
  Mode mode_{Mode::WAIT_DATA};   // only mutated from the main (timer) thread


  void wpCb(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (have_goal_ || msg->poses.empty())
      return;

    const auto & p = msg->poses.front();
    const Vec3 wp(p.position.x, p.position.y, p.position.z);
    goal_q_ = Quat(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    goal_q_.normalize();

    const Vec3 approach = goal_q_ * Vec3::UnitZ();
    goal_pos_ = wp - standoff_dist_ * approach;

    have_goal_ = true;
    RCLCPP_INFO(get_logger(),
      "First waypoint [%.3f %.3f %.3f] -> standoff goal [%.3f %.3f %.3f] (%.2f m along normal).",
      wp.x(), wp.y(), wp.z(), goal_pos_.x(), goal_pos_.y(), goal_pos_.z(), standoff_dist_);
  }

  void armCb(const sensor_msgs::msg::JointState::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(arm_mtx_);
    for (size_t i = 0; i < m->name.size() && i < m->position.size(); ++i)
      arm_pos_[m->name[i]] = m->position[i];
  }

  void mixerCb(const sensor_msgs::msg::JointState::SharedPtr m)
  {
    std::lock_guard<std::mutex> lk(mixer_mtx_);

    double vmax = 0.0;
    for (double v : m->velocity)
      vmax = std::max(vmax, std::abs(v));

    // Position-delta fallback for setups where velocity is left unpopulated.
    const rclcpp::Time stamp(m->header.stamp);
    if (have_prev_ && m->position.size() == prev_pos_.size() && !m->position.empty())
    {
      const double dt = (stamp - prev_stamp_).seconds();
      if (dt > 1e-4)
        for (size_t i = 0; i < m->position.size(); ++i)
          vmax = std::max(vmax, std::abs(m->position[i] - prev_pos_[i]) / dt);
    }
    prev_pos_ = m->position;
    prev_stamp_ = stamp;
    have_prev_ = true;

    mixer_speed_ = vmax;
    mixer_rx_ = std::chrono::steady_clock::now();   // wall-clock receipt time
    mixer_seen_ = true;
  }

  bool mixerIsStationary()
  {
    std::lock_guard<std::mutex> lk(mixer_mtx_);
    if (!mixer_seen_)
      return false;
    const double age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - mixer_rx_).count();
    if (age > mixer_stale_sec_)
      return false;
    return mixer_speed_ < mixer_still_speed_;
  }

  void update()
  {
    if (mode_ == Mode::WAIT_DATA)
    {
      if (have_goal_)
        launchMove();
      return;
    }

    if (mode_ == Mode::MOVING && move_done_)
    {
      if (move_thread_.joinable())
        move_thread_.join();
      RCLCPP_INFO(get_logger(), move_ok_ ? "PTP move complete -> DONE"
                                         : "PTP move failed -> DONE");
      mode_ = Mode::DONE;
    }
  }

  void launchMove()
  {
    if (move_launched_)
      return;
    move_launched_ = true;
    mode_ = Mode::MOVING;
    RCLCPP_INFO(get_logger(),
      "Data ready -> PTP to standoff goal on %s", trajectory_controller_.c_str());
    move_thread_ = std::thread(&VerticalMixerPtp::runMove, this);
  }

  void runMove()
  {
    // 1) Ensure the trajectory controller is active (leave compliance inactive).
    if (!switchControllers({trajectory_controller_}, {compliance_controller_}))
    {
      RCLCPP_ERROR(get_logger(), "MOVE: switch to %s failed", trajectory_controller_.c_str());
      finish(false); return;
    }

    if (require_mixer_still_ && !waitForMixerStill())
    {
      finish(false); return;
    }

    std::vector<double> seed;
    if (!currentArmSeed(seed))
    {
      RCLCPP_ERROR(get_logger(), "MOVE: no /joint_states seed for %s yet",
        arm_states_topic_.c_str());
      finish(false); return;
    }

    std::vector<double> goal_joints;
    if (!computeSeededIk(seed, goal_joints))
    {
      RCLCPP_ERROR(get_logger(), "MOVE: seeded IK for the standoff goal failed");
      finish(false); return;
    }
    double travel = 0.0;
    for (size_t i = 0; i < seed.size(); ++i)
      travel += std::abs(goal_joints[i] - seed[i]);
    RCLCPP_INFO(get_logger(), "MOVE: nearest-branch IK ok (joint travel %.2f rad) -> planning",
      travel);

    finish(sendJointGoal(goal_joints));
  }

  bool currentArmSeed(std::vector<double> & seed)
  {
    // Wait briefly for the first /joint_states.
    const auto t0 = std::chrono::steady_clock::now();
    while (true)
    {
      {
        std::lock_guard<std::mutex> lk(arm_mtx_);
        bool have_all = true;
        seed.clear();
        for (const auto & j : joint_names_)
        {
          auto it = arm_pos_.find(j);
          if (it == arm_pos_.end()) { have_all = false; break; }
          seed.push_back(it->second);
        }
        if (have_all)
          return true;
      }
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > 3.0)
        return false;
      std::this_thread::sleep_for(50ms);
    }
  }

  bool computeSeededIk(const std::vector<double> & seed, std::vector<double> & goal_joints)
  {
    if (!ik_client_->wait_for_service(5s))
    {
      RCLCPP_ERROR(get_logger(), "/compute_ik unavailable (move_group up?)");
      return false;
    }

    auto req = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    auto & ik = req->ik_request;
    ik.group_name = move_group_name_;
    ik.ik_link_name = tool_frame_;
    ik.pose_stamped.header.frame_id = base_frame_;
    ik.pose_stamped.pose.position.x = goal_pos_.x();
    ik.pose_stamped.pose.position.y = goal_pos_.y();
    ik.pose_stamped.pose.position.z = goal_pos_.z();
    ik.pose_stamped.pose.orientation.x = goal_q_.x();
    ik.pose_stamped.pose.orientation.y = goal_q_.y();
    ik.pose_stamped.pose.orientation.z = goal_q_.z();
    ik.pose_stamped.pose.orientation.w = goal_q_.w();
    ik.robot_state.joint_state.name = joint_names_;      // seed = current arm state
    ik.robot_state.joint_state.position = seed;
    ik.avoid_collisions = true;
    ik.timeout = rclcpp::Duration(2s);

    auto future = ik_client_->async_send_request(req);
    if (future.wait_for(8s) != std::future_status::ready)
    {
      RCLCPP_ERROR(get_logger(), "/compute_ik call timed out");
      return false;
    }
    auto resp = future.get();
    if (resp->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "/compute_ik error_code %d", resp->error_code.val);
      return false;
    }

    // Extract the manipulator joints in our declared order.
    std::map<std::string, double> sol;
    const auto & js = resp->solution.joint_state;
    for (size_t i = 0; i < js.name.size() && i < js.position.size(); ++i)
      sol[js.name[i]] = js.position[i];
    goal_joints.clear();
    for (const auto & j : joint_names_)
    {
      auto it = sol.find(j);
      if (it == sol.end())
      {
        RCLCPP_ERROR(get_logger(), "IK solution missing joint %s", j.c_str());
        return false;
      }
      goal_joints.push_back(it->second);
    }
    return true;
  }

  bool sendJointGoal(const std::vector<double> & goal_joints)
  {
    if (!move_client_->wait_for_action_server(10s))
    {
      RCLCPP_ERROR(get_logger(), "/move_action unavailable");
      return false;
    }

    moveit_msgs::action::MoveGroup::Goal goal;
    auto & req = goal.request;
    req.group_name = move_group_name_;
    req.num_planning_attempts = 1;
    req.allowed_planning_time = planning_time_;
    req.max_velocity_scaling_factor = plan_velocity_scale_;
    req.max_acceleration_scaling_factor = plan_accel_scale_;
    req.pipeline_id = pipeline_id_;

    moveit_msgs::msg::Constraints c;
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      moveit_msgs::msg::JointConstraint jc;
      jc.joint_name = joint_names_[i];
      jc.position = goal_joints[i];
      jc.tolerance_above = 0.001;
      jc.tolerance_below = 0.001;
      jc.weight = 1.0;
      c.joint_constraints.push_back(jc);
    }
    req.goal_constraints.push_back(c);
    goal.planning_options.plan_only = false;   // plan AND execute (on JTC)

    auto gh_future = move_client_->async_send_goal(goal);
    if (gh_future.wait_for(15s) != std::future_status::ready)
    {
      RCLCPP_ERROR(get_logger(), "/move_action goal send timed out");
      return false;
    }
    auto gh = gh_future.get();
    if (!gh)
    {
      RCLCPP_ERROR(get_logger(), "/move_action goal rejected");
      return false;
    }

    auto res_future = move_client_->async_get_result(gh);
    if (res_future.wait_for(60s) != std::future_status::ready)
    {
      RCLCPP_ERROR(get_logger(), "/move_action result timed out");
      return false;
    }
    const auto wrapped = res_future.get();
    const int code = wrapped.result->error_code.val;
    if (code != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "/move_action failed, error_code %d", code);
      return false;
    }
    return true;
  }

  bool waitForMixerStill()
  {
    RCLCPP_INFO(get_logger(),
      "Waiting for mixer to settle (< %.3f rad/s) before planning...", mixer_still_speed_);
    const auto t0 = std::chrono::steady_clock::now();
    while (!mixerIsStationary())
    {
      const double waited = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();
      if (waited > mixer_still_timeout_)
      {
        RCLCPP_ERROR(get_logger(),
          "Mixer did not settle within %.1f s -> aborting (won't plan against a "
          "stale/moving collision snapshot). Stop the mixer, or set "
          "require_mixer_still:=false to override.", mixer_still_timeout_);
        return false;
      }
      std::this_thread::sleep_for(100ms);
    }
    RCLCPP_INFO(get_logger(), "Mixer stationary -> planning.");
    return true;
  }

  bool switchControllers(const std::vector<std::string> & activate,
                         const std::vector<std::string> & deactivate)
  {
    using SwitchController = controller_manager_msgs::srv::SwitchController;

    auto client = create_client<SwitchController>(
      "/controller_manager/switch_controller");
    if (!client->wait_for_service(5s))
    {
      RCLCPP_ERROR(get_logger(), "switch_controller service unavailable");
      return false;
    }

    auto req = std::make_shared<SwitchController::Request>();
    req->activate_controllers = activate;
    req->deactivate_controllers = deactivate;
    // BEST_EFFORT: no-op instead of a STRICT rejection if already in this state.
    req->strictness = SwitchController::Request::BEST_EFFORT;
    req->activate_asap = true;

    auto future = client->async_send_request(req);
    if (future.wait_for(10s) != std::future_status::ready)
    {
      RCLCPP_ERROR(get_logger(), "switch_controller call timed out");
      return false;
    }
    return future.get()->ok;
  }

  void finish(bool ok)
  {
    move_ok_ = ok;
    move_done_ = true;
  }

  std::string base_frame_, tool_frame_, move_group_name_;
  std::vector<std::string> joint_names_;
  std::string waypoints_topic_, arm_states_topic_, mixer_states_topic_;
  std::string compliance_controller_, trajectory_controller_, pipeline_id_;
  double standoff_dist_;
  double planning_time_, plan_velocity_scale_, plan_accel_scale_;
  bool require_mixer_still_;
  double mixer_still_speed_, mixer_still_timeout_, mixer_stale_sec_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr wp_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr mixer_sub_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
  rclcpp_action::Client<moveit_msgs::action::MoveGroup>::SharedPtr move_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool have_goal_{false};
  Vec3 goal_pos_;
  Quat goal_q_;

  // arm seed (guarded by arm_mtx_)
  std::mutex arm_mtx_;
  std::map<std::string, double> arm_pos_;

  // mixer telemetry (guarded by mixer_mtx_)
  std::mutex mixer_mtx_;
  double mixer_speed_{0.0};
  bool mixer_seen_{false};
  std::chrono::steady_clock::time_point mixer_rx_;
  std::vector<double> prev_pos_;
  rclcpp::Time prev_stamp_{0, 0, RCL_ROS_TIME};
  bool have_prev_{false};

  std::thread move_thread_;
  std::atomic<bool> move_launched_{false};
  std::atomic<bool> move_done_{false};
  std::atomic<bool> move_ok_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VerticalMixerPtp>());
  rclcpp::shutdown();
  return 0;
}

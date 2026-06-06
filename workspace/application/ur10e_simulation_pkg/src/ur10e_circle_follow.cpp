#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <memory>
#include <thread>
#include <atomic>
#include <vector>
#include <map>
#include <string>
#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;

class CircleForce : public rclcpp::Node
{
public:
  CircleForce()
  : Node("retract_rotate_approach_circle")
  {
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&CircleForce::jointCb, this, _1));

    robot_desc_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_description",
      rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&CircleForce::robotDescCb, this, _1));

    ft_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/cartesian_compliance_controller/ft_sensor_wrench", 10,
      std::bind(&CircleForce::ftCb, this, _1));

    target_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/cartesian_compliance_controller/target_frame", 10);

    target_wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      "/cartesian_compliance_controller/target_wrench", 10);

    timer_ = create_wall_timer(
      20ms, std::bind(&CircleForce::update, this));

    RCLCPP_INFO(get_logger(),
      "Retract → Rotate(MoveIt/JTC) → Approach → Circle → STOP");
  }

  ~CircleForce() override
  {
    if (rotate_thread_.joinable())
      rotate_thread_.join();
  }

private:
  enum class Mode { RETRACT, ROTATE, APPROACH, CIRCLE, STOP };
  Mode mode_{Mode::RETRACT};   // only ever mutated from the main (executor) thread

  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;


  void jointCb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    joint_state_ = *msg;
    got_joints_ = true;
  }

  void robotDescCb(const std_msgs::msg::String::SharedPtr msg)
  {
    robot_description_ = msg->data;
    got_urdf_ = true;
  }

  void ftCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    got_ft_ = true;
    measured_fz_ = msg->wrench.force.z;

    if (mode_ == Mode::APPROACH &&
        std::abs(measured_fz_) > contact_force_threshold_)
    {
      radius_ = std::hypot(current_x_, current_y_);
      theta_ = std::atan2(current_y_, current_x_);
      traveled_angle_ = 0.0;
      force_integral_ = 0.0;

      circle_start_time_ = now();
      last_update_time_ = now();

      mode_ = Mode::CIRCLE;

      RCLCPP_WARN(get_logger(),
        "Contact detected → CIRCLE | Radius = %.4f", radius_);
    }
  }

  void update()
  {
    if (!got_joints_ || !got_urdf_ || !got_ft_)
      return;

    if (!kdl_ready_)
    {
      initKDL();
      readStartPose();
      kdl_ready_ = true;
    }

    // While ROTATE is in flight the joint_trajectory_controller / MoveIt owns the
    // arm, so we must NOT stream Cartesian targets to the (now inactive)
    // compliance controller. The rotate runs on a worker thread; we only
    // finalise the transition here, on the main thread.
    if (mode_ == Mode::ROTATE)
    {
      if (rotate_done_)
      {
        if (rotate_thread_.joinable())
          rotate_thread_.join();

        if (!rotate_ok_)
        {
          RCLCPP_ERROR(get_logger(), "Rotate failed → STOP");
          mode_ = Mode::STOP;
        }
        else
        {
          // The flip changed the tool orientation (and re-affirms position);
          // re-read FK so APPROACH streams from the new pose without a jump.
          refreshPoseAfterRotate();
          approach_start_time_ = now();
          mode_ = Mode::APPROACH;
        }
      }
      return;
    }

    publishTargetPose();
    publishTargetWrench();
  }

  void initKDL()
  {
    urdf::Model model;
    model.initString(robot_description_);

    KDL::Tree tree;
    kdl_parser::treeFromUrdfModel(model, tree);
    tree.getChain("base_link", "tool0", chain_);

    fk_solver_ =
      std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
    joints_.resize(chain_.getNrOfJoints());
  }

  void readStartPose()
  {
    for (size_t i = 0; i < 6; ++i)
      joints_(i) = joint_state_.position[i];

    KDL::Frame frame;
    fk_solver_->JntToCart(joints_, frame);

    start_x_ = frame.p.x();
    start_y_ = frame.p.y();
    start_z_ = frame.p.z();

    current_x_ = start_x_;
    current_y_ = start_y_;

    frame.M.GetQuaternion(qx_, qy_, qz_, qw_);

    double n = std::hypot(start_x_, start_y_);
    dir_x_ = -start_x_ / n;
    dir_y_ = -start_y_ / n;

    retract_start_time_ = now();
  }

  // Re-read the live pose from FK after the joint-space flip so the compliance
  // controller resumes from exactly where the arm now is (rotated orientation,
  // unchanged X/Y/Z since wrist_3 spins about the tool Z through tool0).
  void refreshPoseAfterRotate()
  {
    for (size_t i = 0; i < 6; ++i)
      joints_(i) = joint_state_.position[i];

    KDL::Frame frame;
    fk_solver_->JntToCart(joints_, frame);

    start_x_ = frame.p.x();
    start_y_ = frame.p.y();
    start_z_ = frame.p.z();

    current_x_ = start_x_;
    current_y_ = start_y_;

    frame.M.GetQuaternion(qx_, qy_, qz_, qw_);

    double n = std::hypot(start_x_, start_y_);
    dir_x_ = -start_x_ / n;
    dir_y_ = -start_y_ / n;
  }

  double Scurve(double t)
  {
    t = std::clamp(t, 0.0, 1.0);
    return 3*t*t - 2*t*t*t;
  }

  // ---- ROTATE: switch to JTC, flip the tool via MoveIt, switch back ----------

  void launchRotate()
  {
    if (rotate_launched_)
      return;
    rotate_launched_ = true;
    mode_ = Mode::ROTATE;
    RCLCPP_WARN(get_logger(),
      "RETRACT done → ROTATE (compliance → joint_trajectory_controller via MoveIt)");
    rotate_thread_ = std::thread(&CircleForce::runRotate, this);
  }

  void runRotate()
  {
    if (!switchControllers({"joint_trajectory_controller"},
                           {"cartesian_compliance_controller"}))
    {
      RCLCPP_ERROR(get_logger(), "ROTATE: switch to joint_trajectory_controller failed");
      rotate_ok_ = false;
      rotate_done_ = true;
      return;
    }

    if (!move_group_)
    {
      move_group_ =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), "ur_manipulator");
      move_group_->setPlanningTime(10.0);
      move_group_->setMaxVelocityScalingFactor(0.6);
      move_group_->setMaxAccelerationScalingFactor(0.6);
    }

    // Group order: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
    std::vector<double> jv = move_group_->getCurrentJointValues();
    if (jv.size() < 6)
    {
      RCLCPP_ERROR(get_logger(), "ROTATE: could not read current joint values");
      rotate_ok_ = false;
      rotate_done_ = true;
      return;
    }

    std::map<std::string, double> target;
    target["shoulder_pan_joint"]  = jv[0];
    target["shoulder_lift_joint"] = jv[1];
    target["elbow_joint"]         = jv[2];
    target["wrist_1_joint"]       = jv[3];
    target["wrist_2_joint"]       = jv[4];
    target["wrist_3_joint"]       = jv[5] + M_PI;   // 180° tool flip about tool Z

    move_group_->setJointValueTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "ROTATE: MoveIt planning failed");
      rotate_ok_ = false;
      rotate_done_ = true;
      return;
    }
    RCLCPP_INFO(get_logger(),
      "ROTATE: plan OK → executing on joint_trajectory_controller");

    if (!sendTrajectory(plan.trajectory_.joint_trajectory))
    {
      RCLCPP_ERROR(get_logger(), "ROTATE: trajectory execution failed");
      rotate_ok_ = false;
      rotate_done_ = true;
      return;
    }

    if (!switchControllers({"cartesian_compliance_controller"},
                           {"joint_trajectory_controller"}))
    {
      RCLCPP_ERROR(get_logger(), "ROTATE: switch back to compliance failed");
      rotate_ok_ = false;
      rotate_done_ = true;
      return;
    }

    RCLCPP_WARN(get_logger(), "ROTATE complete → APPROACH");
    rotate_ok_ = true;
    rotate_done_ = true;
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
    req->strictness = SwitchController::Request::STRICT;
    req->activate_asap = true;

    // Called from the rotate worker thread; the main executor spins the node and
    // delivers the response, so waiting here does not deadlock.
    auto future = client->async_send_request(req);
    if (future.wait_for(10s) != std::future_status::ready)
    {
      RCLCPP_ERROR(get_logger(), "switch_controller call timed out");
      return false;
    }
    return future.get()->ok;
  }

  bool sendTrajectory(const trajectory_msgs::msg::JointTrajectory & traj)
  {
    auto ac = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/joint_trajectory_controller/follow_joint_trajectory");
    if (!ac->wait_for_action_server(10s))
    {
      RCLCPP_ERROR(get_logger(), "joint_trajectory_controller action server unavailable");
      return false;
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory = traj;

    auto goal_future = ac->async_send_goal(goal);
    if (goal_future.wait_for(10s) != std::future_status::ready)
    {
      RCLCPP_ERROR(get_logger(), "JTC goal send timed out");
      return false;
    }
    auto goal_handle = goal_future.get();
    if (!goal_handle)
    {
      RCLCPP_ERROR(get_logger(), "JTC goal rejected");
      return false;
    }

    auto result_future = ac->async_get_result(goal_handle);
    if (result_future.wait_for(30s) != std::future_status::ready)
    {
      RCLCPP_ERROR(get_logger(), "JTC result timed out");
      return false;
    }
    return result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED;
  }

  void publishTargetPose()
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = "base_link";

    if (mode_ == Mode::RETRACT)
    {
      double t = (now() - retract_start_time_).seconds() / retract_duration_;
      double s = Scurve(t);

      current_x_ = start_x_ + s * retract_distance_ * dir_x_;
      current_y_ = start_y_ + s * retract_distance_ * dir_y_;
      double z = start_z_ + s * move_up_distance_;

      pose.pose.position.x = current_x_;
      pose.pose.position.y = current_y_;
      pose.pose.position.z = z;

      if (t >= 1.0)
      {
        start_x_ = current_x_;
        start_y_ = current_y_;
        start_z_ = z;

        launchRotate();   // hands the arm to MoveIt/JTC for the tool flip
      }
    }

    else if (mode_ == Mode::APPROACH)
    {
      double t =
        (now() - approach_start_time_).seconds() / approach_duration_;
      t = std::clamp(t, 0.0, 1.0);

      current_x_ = start_x_ - t * approach_distance_ * dir_x_;
      current_y_ = start_y_ - t * approach_distance_ * dir_y_;

      pose.pose.position.x = current_x_;
      pose.pose.position.y = current_y_;
      pose.pose.position.z = start_z_;
    }

    else if (mode_ == Mode::CIRCLE)
    {
      double dt = (now() - last_update_time_).seconds();
      last_update_time_ = now();
      dt = std::clamp(dt, 0.0, 0.1);

      double elapsed = (now() - circle_start_time_).seconds();
      double total_time = max_angle_rad_ / angular_speed_;
      double ramp_time = 0.15 * total_time;

      double alpha = 1.0;

      if (elapsed < ramp_time)
        alpha = Scurve(elapsed / ramp_time);
      else if (elapsed > total_time - ramp_time)
        alpha = Scurve((total_time - elapsed) / ramp_time);

      double omega = -angular_speed_ * alpha;

      double force_error = measured_fz_ - desired_circle_force_;
      force_integral_ += force_error * dt;
      force_integral_ =
        std::clamp(force_integral_, -integral_limit_, integral_limit_);

      double radius_dot =
        kp_radius_ * force_error +
        ki_radius_ * force_integral_;

      radius_dot =
        std::clamp(radius_dot, -max_radius_rate_, max_radius_rate_);

      radius_ += radius_dot * dt;

      double dtheta = omega * dt;
      theta_ += dtheta;
      traveled_angle_ += std::abs(dtheta);

      if (traveled_angle_ >= max_angle_rad_)
      {
        mode_ = Mode::STOP;
        RCLCPP_WARN(get_logger(), "Circle complete → STOP");
      }

      KDL::Vector z_axis(std::cos(theta_), std::sin(theta_), 0.0);
      z_axis.Normalize();
      KDL::Vector up(0, 0, 1);

      KDL::Vector x_axis = up * z_axis;
      if (x_axis.Norm() < 1e-6)
        x_axis = KDL::Vector(1, 0, 0);
      x_axis.Normalize();

      KDL::Vector y_axis = z_axis * x_axis;
      y_axis.Normalize();

      KDL::Rotation R(x_axis, y_axis, z_axis);
      R.GetQuaternion(qx_, qy_, qz_, qw_);

      current_x_ = radius_ * std::cos(theta_);
      current_y_ = radius_ * std::sin(theta_);

      pose.pose.position.x = current_x_;
      pose.pose.position.y = current_y_;
      pose.pose.position.z = start_z_;
    }

    else
    {
      pose.pose.position.x = current_x_;
      pose.pose.position.y = current_y_;
      pose.pose.position.z = start_z_;
    }

    pose.pose.orientation.x = qx_;
    pose.pose.orientation.y = qy_;
    pose.pose.orientation.z = qz_;
    pose.pose.orientation.w = qw_;

    target_pose_pub_->publish(pose);
  }

  void publishTargetWrench()
  {
    geometry_msgs::msg::WrenchStamped wrench;
    wrench.header.stamp = now();
    wrench.header.frame_id = "tool0";

    wrench.wrench.force.z =
      (mode_ == Mode::STOP) ? 0.0 :
      (mode_ == Mode::APPROACH ? -5.0 : -3.0);

    target_wrench_pub_->publish(wrench);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_desc_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ft_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr target_wrench_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::JointState joint_state_;
  std::string robot_description_;

  bool got_joints_{false}, got_urdf_{false}, got_ft_{false}, kdl_ready_{false};

  KDL::Chain chain_;
  KDL::JntArray joints_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

  // ROTATE worker (off the executor thread so the controller-switch service and
  // MoveIt blocking calls do not deadlock the main spin).
  std::thread rotate_thread_;
  std::atomic<bool> rotate_launched_{false};
  std::atomic<bool> rotate_done_{false};
  std::atomic<bool> rotate_ok_{false};
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  rclcpp::Time retract_start_time_, approach_start_time_;
  rclcpp::Time circle_start_time_, last_update_time_;

  double start_x_, start_y_, start_z_;
  double current_x_, current_y_;
  double dir_x_, dir_y_;

  double radius_{0.0}, theta_{0.0}, traveled_angle_{0.0};
  double measured_fz_{0.0}, force_integral_{0.0};

  double qx_, qy_, qz_, qw_;

  const double retract_duration_{3.0};
  const double retract_distance_{0.15};
  const double move_up_distance_{0.10};

  const double approach_distance_{0.35};
  const double approach_duration_{7.0};

  const double angular_speed_{1.0};
  const double contact_force_threshold_{2.0};
  const double desired_circle_force_{-10.0};

  const double kp_radius_{0.5};
  const double ki_radius_{0.005};
  const double integral_limit_{2.0};
  const double max_radius_rate_{0.01};
  const double max_angle_rad_{7.29};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CircleForce>());
  rclcpp::shutdown();
  return 0;
}

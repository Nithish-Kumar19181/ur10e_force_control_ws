#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>
#include <algorithm>
#include <atomic>
#include <thread>
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;

class CircleForce : public rclcpp::Node
{
public:
  CircleForce() : Node("circle_follow_node")
  {
    // Declare all parameters (YAML overrides these defaults)
    declare_parameter("z_direction",              1.0);
    declare_parameter("vessel_height",            0.85);
    declare_parameter("vessel_inner_radius_min",  0.20);
    declare_parameter("vessel_inner_radius_max",  0.60);
    declare_parameter("blade_width",              0.03);
    declare_parameter("layer_overlap_fraction",   0.90);
    declare_parameter("start_z_offset",           0.00);
    declare_parameter("resume_layer",             0);
    declare_parameter("approach_distance",        0.35);
    declare_parameter("approach_duration",        7.0);
    declare_parameter("approach_timeout",         20.0);
    declare_parameter("angular_speed",            0.25);
    declare_parameter("max_angle_rad",            6.2832);
    declare_parameter("contact_force_threshold",  2.0);
    declare_parameter("desired_circle_force",    -10.0);
    declare_parameter("circle_ramp_fraction",     0.15);
    declare_parameter("high_force_threshold",     15.0);
    declare_parameter("max_force_clamp",          25.0);
    declare_parameter("min_speed_fraction",       0.15);
    declare_parameter("kp_radius",                0.5);
    declare_parameter("ki_radius",                0.005);
    declare_parameter("integral_limit",           2.0);
    declare_parameter("max_radius_rate",          0.01);
    declare_parameter("approach_force_z",        -5.0);
    declare_parameter("circle_force_z",          -3.0);
    declare_parameter("retract_distance",         0.15);
    declare_parameter("retract_duration",         3.0);
    declare_parameter("z_step_duration",          2.0);
    declare_parameter("final_retract_distance",   0.30);
    declare_parameter("final_retract_duration",   4.0);
    declare_parameter("safe_z_height",            0.20);
    declare_parameter("safe_z_duration",          3.0);
    declare_parameter("home_joint_shoulder_pan",   0.40960574676343686);
    declare_parameter("home_joint_shoulder_lift", -1.6747645009139471);
    declare_parameter("home_joint_elbow",         -2.401686248302344);
    declare_parameter("home_joint_wrist_1",        0.9661313426465519);
    declare_parameter("home_joint_wrist_2",        1.1646508483476774);
    declare_parameter("home_joint_wrist_3",       -3.1578720449159454);

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

    timer_ = create_wall_timer(20ms, std::bind(&CircleForce::update, this));

    RCLCPP_INFO(get_logger(), "CircleForce node started — waiting for sensors");
  }

private:
  enum class Mode {
    IDLE, APPROACH, CIRCLE,
    RADIAL_RETRACT, Z_STEP,
    FINAL_RETRACT, SAFE_Z_MOVE,
    HOMING, DONE
  };

  // ── Callbacks ─────────────────────────────────────────────────────────────

  void jointCb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    joint_state_ = *msg;
    got_joints_  = true;
  }

  void robotDescCb(const std_msgs::msg::String::SharedPtr msg)
  {
    robot_description_ = msg->data;
    got_urdf_ = true;
  }

  void ftCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    got_ft_      = true;
    measured_fz_ = msg->wrench.force.z;

    if (mode_ == Mode::APPROACH &&
        std::abs(measured_fz_) > contact_force_threshold_)
    {
      radius_         = std::hypot(current_x_, current_y_);
      theta_          = std::atan2(current_y_, current_x_);
      traveled_angle_ = 0.0;
      force_integral_ = 0.0;

      circle_start_time_ = now();
      last_update_time_  = now();
      mode_ = Mode::CIRCLE;

      RCLCPP_WARN(get_logger(),
        "Contact! Layer %d/%d [%s] | R=%.4f  Z=%.4f",
        current_layer_ + 1, total_layers_,
        circle_direction_cw_ ? "CW" : "CCW",
        radius_, current_layer_z_);
    }
  }

  // ── Main loop (50 Hz) ─────────────────────────────────────────────────────

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

    publishTargetPose();
    publishTargetWrench();
  }

  // ── KDL initialisation ────────────────────────────────────────────────────

  void initKDL()
  {
    urdf::Model model;
    model.initString(robot_description_);

    KDL::Tree tree;
    kdl_parser::treeFromUrdfModel(model, tree);
    tree.getChain("base_link", "tool0", chain_);

    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
    joints_.resize(chain_.getNrOfJoints());
  }

  void readStartPose()
  {
    // Load all parameters into member variables
    z_direction_             = get_parameter("z_direction").as_double();
    vessel_height_           = get_parameter("vessel_height").as_double();
    vessel_r_min_            = get_parameter("vessel_inner_radius_min").as_double();
    vessel_r_max_            = get_parameter("vessel_inner_radius_max").as_double();
    blade_width_             = get_parameter("blade_width").as_double();
    layer_overlap_fraction_  = get_parameter("layer_overlap_fraction").as_double();
    start_z_offset_          = get_parameter("start_z_offset").as_double();
    approach_distance_       = get_parameter("approach_distance").as_double();
    approach_duration_       = get_parameter("approach_duration").as_double();
    approach_timeout_        = get_parameter("approach_timeout").as_double();
    angular_speed_           = get_parameter("angular_speed").as_double();
    max_angle_rad_           = get_parameter("max_angle_rad").as_double();
    contact_force_threshold_ = get_parameter("contact_force_threshold").as_double();
    desired_circle_force_    = get_parameter("desired_circle_force").as_double();
    circle_ramp_fraction_    = get_parameter("circle_ramp_fraction").as_double();
    high_force_threshold_    = get_parameter("high_force_threshold").as_double();
    max_force_clamp_         = get_parameter("max_force_clamp").as_double();
    min_speed_fraction_      = get_parameter("min_speed_fraction").as_double();
    kp_radius_               = get_parameter("kp_radius").as_double();
    ki_radius_               = get_parameter("ki_radius").as_double();
    integral_limit_          = get_parameter("integral_limit").as_double();
    max_radius_rate_         = get_parameter("max_radius_rate").as_double();
    approach_force_z_        = get_parameter("approach_force_z").as_double();
    circle_force_z_          = get_parameter("circle_force_z").as_double();
    retract_distance_        = get_parameter("retract_distance").as_double();
    retract_duration_        = get_parameter("retract_duration").as_double();
    z_step_duration_         = get_parameter("z_step_duration").as_double();
    final_retract_distance_  = get_parameter("final_retract_distance").as_double();
    final_retract_duration_  = get_parameter("final_retract_duration").as_double();
    safe_z_height_           = get_parameter("safe_z_height").as_double();
    safe_z_duration_         = get_parameter("safe_z_duration").as_double();

    // FK home pose
    for (size_t i = 0; i < 6; ++i)
      joints_(i) = joint_state_.position[i];

    KDL::Frame frame;
    fk_solver_->JntToCart(joints_, frame);

    double fk_x = frame.p.x();
    double fk_y = frame.p.y();
    double fk_z = frame.p.z();
    frame.M.GetQuaternion(qx_, qy_, qz_, qw_);

    current_x_ = fk_x;
    current_y_ = fk_y;

    // Layer geometry
    layer_step_z_  = blade_width_ * layer_overlap_fraction_ * z_direction_;
    total_layers_  = std::max(1, (int)(vessel_height_ / std::abs(layer_step_z_)));

    int resume = std::clamp(static_cast<int>(get_parameter("resume_layer").as_int()), 0, total_layers_ - 1);
    current_layer_ = resume;

    current_layer_z_ = fk_z + start_z_offset_ * z_direction_
                      + current_layer_ * layer_step_z_;
    circle_direction_cw_ = (current_layer_ % 2 == 0);

    // Approach direction: inward unit vector (convention: APPROACH subtracts this to move outward)
    double n  = std::hypot(fk_x, fk_y);
    dir_x_ = (n > 1e-6) ? -fk_x / n : 0.0;
    dir_y_ = (n > 1e-6) ? -fk_y / n : 0.0;

    approach_start_x_    = fk_x;
    approach_start_y_    = fk_y;
    approach_start_time_ = now();
    mode_ = Mode::APPROACH;

    RCLCPP_INFO(get_logger(),
      "Initialized: %d layers, step=%.4f m, layer 0 Z=%.4f, dir=%s",
      total_layers_, layer_step_z_, current_layer_z_,
      circle_direction_cw_ ? "CW" : "CCW");

    if (resume > 0)
      RCLCPP_WARN(get_logger(),
        "Resuming from layer %d/%d  Z=%.4f",
        current_layer_ + 1, total_layers_, current_layer_z_);
  }

  // ── Motion helpers ────────────────────────────────────────────────────────

  double Scurve(double t)
  {
    t = std::clamp(t, 0.0, 1.0);
    return 3.0*t*t - 2.0*t*t*t;
  }

  // Re-enter APPROACH from the current (retracted) position toward the wall
  void enterApproach()
  {
    double n = std::hypot(current_x_, current_y_);
    if (n > 1e-6) {
      dir_x_ = -current_x_ / n;
      dir_y_ = -current_y_ / n;
    }
    approach_start_x_    = current_x_;
    approach_start_y_    = current_y_;
    approach_start_time_ = now();
    force_integral_      = 0.0;
    mode_ = Mode::APPROACH;
  }

  void enterRadialRetract()
  {
    double n = std::hypot(current_x_, current_y_);
    retract_dir_x_      = (n > 1e-6) ? current_x_ / n : 1.0;
    retract_dir_y_      = (n > 1e-6) ? current_y_ / n : 0.0;
    retract_start_x_    = current_x_;
    retract_start_y_    = current_y_;
    retract_start_time_ = now();
    mode_ = Mode::RADIAL_RETRACT;
    RCLCPP_INFO(get_logger(), "Layer %d/%d complete → RETRACT",
      current_layer_ + 1, total_layers_);
  }

  void enterZStep()
  {
    current_layer_++;
    if (current_layer_ >= total_layers_) {
      enterFinalRetract();
      return;
    }
    circle_direction_cw_ = (current_layer_ % 2 == 0);
    z_step_start_z_    = current_layer_z_;
    z_step_target_z_   = current_layer_z_ + layer_step_z_;
    z_step_start_time_ = now();
    mode_ = Mode::Z_STEP;
    RCLCPP_INFO(get_logger(),
      "Z step: layer %d/%d  Z %.4f → %.4f  [%s]",
      current_layer_ + 1, total_layers_,
      z_step_start_z_, z_step_target_z_,
      circle_direction_cw_ ? "CW" : "CCW");
  }

  void enterFinalRetract()
  {
    double n = std::hypot(current_x_, current_y_);
    final_retract_dir_x_      = (n > 1e-6) ? current_x_ / n : 1.0;
    final_retract_dir_y_      = (n > 1e-6) ? current_y_ / n : 0.0;
    final_retract_start_x_    = current_x_;
    final_retract_start_y_    = current_y_;
    final_retract_start_time_ = now();
    mode_ = Mode::FINAL_RETRACT;
    RCLCPP_WARN(get_logger(), "All %d layers complete → FINAL RETRACT", total_layers_);
  }

  void enterSafeZMove()
  {
    safe_z_start_z_    = current_layer_z_;
    safe_z_start_time_ = now();
    mode_ = Mode::SAFE_Z_MOVE;
    RCLCPP_INFO(get_logger(), "Safe Z move: %.4f → %.4f", safe_z_start_z_, safe_z_height_);
  }

  // MoveIt homing in detached thread so the 50 Hz loop never blocks
  void triggerHome()
  {
    // Capture shared_ptr so the node stays alive for the thread's duration
    auto self = shared_from_this();
    home_thread_ = std::thread([this, self]() {
      try {
        auto mg = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          self, "ur_manipulator");
        mg->setPlanningTime(15.0);
        mg->setMaxVelocityScalingFactor(0.2);
        mg->setMaxAccelerationScalingFactor(0.2);

        std::map<std::string, double> home_joints;
        home_joints["shoulder_pan_joint"]  = get_parameter("home_joint_shoulder_pan").as_double();
        home_joints["shoulder_lift_joint"] = get_parameter("home_joint_shoulder_lift").as_double();
        home_joints["elbow_joint"]         = get_parameter("home_joint_elbow").as_double();
        home_joints["wrist_1_joint"]       = get_parameter("home_joint_wrist_1").as_double();
        home_joints["wrist_2_joint"]       = get_parameter("home_joint_wrist_2").as_double();
        home_joints["wrist_3_joint"]       = get_parameter("home_joint_wrist_3").as_double();

        mg->setJointValueTarget(home_joints);
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        if (mg->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_ERROR(get_logger(), "Home planning failed");
          home_failed_ = true;
          return;
        }
        if (mg->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_ERROR(get_logger(), "Home execution failed");
          home_failed_ = true;
          return;
        }
        home_complete_ = true;
        RCLCPP_INFO(get_logger(), "Homing complete");
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Home thread exception: %s", e.what());
        home_failed_ = true;
      }
    });
    home_thread_.detach();
  }

  // ── Pose setpoint publisher ────────────────────────────────────────────────

  void publishTargetPose()
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp    = now();
    pose.header.frame_id = "base_link";

    switch (mode_)
    {
    case Mode::IDLE:
      return;

    case Mode::APPROACH:
    {
      double t = (now() - approach_start_time_).seconds() / approach_duration_;
      double s = Scurve(t);
      current_x_ = approach_start_x_ - s * approach_distance_ * dir_x_;
      current_y_ = approach_start_y_ - s * approach_distance_ * dir_y_;
      pose.pose.position.x = current_x_;
      pose.pose.position.y = current_y_;
      pose.pose.position.z = current_layer_z_;

      if ((now() - approach_start_time_).seconds() > approach_timeout_) {
        RCLCPP_ERROR(get_logger(),
          "Approach timeout at layer %d — no contact. STOPPING.", current_layer_ + 1);
        mode_ = Mode::DONE;
      }
      break;
    }

    case Mode::CIRCLE:
    {
      double dt = (now() - last_update_time_).seconds();
      last_update_time_ = now();
      dt = std::clamp(dt, 0.0, 0.1);

      double elapsed    = (now() - circle_start_time_).seconds();
      double total_time = max_angle_rad_ / angular_speed_;
      double ramp_time  = circle_ramp_fraction_ * total_time;

      // Speed ramp at start and end of arc
      double alpha = 1.0;
      if (elapsed < ramp_time)
        alpha = Scurve(elapsed / ramp_time);
      else if (elapsed > total_time - ramp_time)
        alpha = Scurve((total_time - elapsed) / ramp_time);

      // Force spike: linearly reduce omega between high_force_threshold and max_force_clamp
      double fz_abs      = std::abs(measured_fz_);
      double speed_scale = 1.0;
      if (fz_abs > high_force_threshold_) {
        double range  = std::max(max_force_clamp_ - high_force_threshold_, 1e-3);
        double excess = fz_abs - high_force_threshold_;
        speed_scale   = 1.0 - (1.0 - min_speed_fraction_) * std::min(excess / range, 1.0);
      }

      double sign  = circle_direction_cw_ ? -1.0 : 1.0;
      double omega = sign * angular_speed_ * alpha * speed_scale;

      // Radius PI: adapts to surface irregularity and off-centre placement
      double force_error = measured_fz_ - desired_circle_force_;
      force_integral_ += force_error * dt;
      force_integral_  = std::clamp(force_integral_, -integral_limit_, integral_limit_);
      double radius_dot = kp_radius_ * force_error + ki_radius_ * force_integral_;
      radius_dot = std::clamp(radius_dot, -max_radius_rate_, max_radius_rate_);
      radius_ += radius_dot * dt;
      radius_  = std::clamp(radius_, vessel_r_min_, vessel_r_max_);

      double dtheta = omega * dt;
      theta_          += dtheta;
      traveled_angle_ += std::abs(dtheta);

      // Update position before using it in enterRadialRetract
      current_x_ = radius_ * std::cos(theta_);
      current_y_ = radius_ * std::sin(theta_);

      // Tool Z-axis points radially outward
      KDL::Vector z_axis(std::cos(theta_), std::sin(theta_), 0.0);
      z_axis.Normalize();
      KDL::Vector up(0, 0, 1);
      KDL::Vector x_axis = up * z_axis;
      if (x_axis.Norm() < 1e-6) x_axis = KDL::Vector(1, 0, 0);
      x_axis.Normalize();
      KDL::Vector y_axis = z_axis * x_axis;
      y_axis.Normalize();
      KDL::Rotation R(x_axis, y_axis, z_axis);
      R.GetQuaternion(qx_, qy_, qz_, qw_);

      if (traveled_angle_ >= max_angle_rad_)
        enterRadialRetract();

      pose.pose.position.x = current_x_;
      pose.pose.position.y = current_y_;
      pose.pose.position.z = current_layer_z_;
      break;
    }

    case Mode::RADIAL_RETRACT:
    {
      double t = (now() - retract_start_time_).seconds() / retract_duration_;
      double s = Scurve(t);
      current_x_ = retract_start_x_ - s * retract_distance_ * retract_dir_x_;
      current_y_ = retract_start_y_ - s * retract_distance_ * retract_dir_y_;
      pose.pose.position.x = current_x_;
      pose.pose.position.y = current_y_;
      pose.pose.position.z = current_layer_z_;
      if (t >= 1.0)
        enterZStep();
      break;
    }

    case Mode::Z_STEP:
    {
      double t = (now() - z_step_start_time_).seconds() / z_step_duration_;
      double s = Scurve(t);
      pose.pose.position.x = current_x_;
      pose.pose.position.y = current_y_;
      pose.pose.position.z = z_step_start_z_ + s * (z_step_target_z_ - z_step_start_z_);
      if (t >= 1.0) {
        current_layer_z_ = z_step_target_z_;
        enterApproach();
      }
      break;
    }

    case Mode::FINAL_RETRACT:
    {
      double t = (now() - final_retract_start_time_).seconds() / final_retract_duration_;
      double s = Scurve(t);
      current_x_ = final_retract_start_x_ - s * final_retract_distance_ * final_retract_dir_x_;
      current_y_ = final_retract_start_y_ - s * final_retract_distance_ * final_retract_dir_y_;
      pose.pose.position.x = current_x_;
      pose.pose.position.y = current_y_;
      pose.pose.position.z = current_layer_z_;
      if (t >= 1.0)
        enterSafeZMove();
      break;
    }

    case Mode::SAFE_Z_MOVE:
    {
      double t = (now() - safe_z_start_time_).seconds() / safe_z_duration_;
      double s = Scurve(t);
      pose.pose.position.x = current_x_;
      pose.pose.position.y = current_y_;
      pose.pose.position.z = safe_z_start_z_ + s * (safe_z_height_ - safe_z_start_z_);
      if (t >= 1.0) {
        mode_ = Mode::HOMING;
        RCLCPP_INFO(get_logger(), "Safe Z reached → HOMING");
      }
      break;
    }

    case Mode::HOMING:
    {
      if (!homing_triggered_) {
        triggerHome();
        homing_triggered_ = true;
      }
      if (home_complete_) {
        mode_ = Mode::DONE;
        RCLCPP_WARN(get_logger(), "DONE — vessel cleaning complete");
      } else if (home_failed_) {
        home_retry_count_++;
        if (home_retry_count_ <= max_home_retries_) {
          RCLCPP_WARN(get_logger(),
            "Homing failed — retry %d/%d", home_retry_count_, max_home_retries_);
          home_failed_       = false;
          homing_triggered_  = false;
        } else {
          RCLCPP_ERROR(get_logger(),
            "Homing failed after %d attempts. Manual intervention required.",
            max_home_retries_);
        }
      }
      pose.pose.position.x = current_x_;
      pose.pose.position.y = current_y_;
      pose.pose.position.z = safe_z_height_;
      break;
    }

    case Mode::DONE:
      pose.pose.position.x = current_x_;
      pose.pose.position.y = current_y_;
      pose.pose.position.z = safe_z_height_;
      break;
    }

    pose.pose.orientation.x = qx_;
    pose.pose.orientation.y = qy_;
    pose.pose.orientation.z = qz_;
    pose.pose.orientation.w = qw_;
    target_pose_pub_->publish(pose);
  }

  // ── Wrench setpoint publisher ──────────────────────────────────────────────

  void publishTargetWrench()
  {
    geometry_msgs::msg::WrenchStamped wrench;
    wrench.header.stamp    = now();
    wrench.header.frame_id = "tool0";

    switch (mode_) {
      case Mode::APPROACH: wrench.wrench.force.z = approach_force_z_; break;
      case Mode::CIRCLE:   wrench.wrench.force.z = circle_force_z_;   break;
      default:             wrench.wrench.force.z = 0.0; break;
    }
    target_wrench_pub_->publish(wrench);
  }

  // ── ROS interfaces ─────────────────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr      joint_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr             robot_desc_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ft_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr      target_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr    target_wrench_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ── Sensor data ────────────────────────────────────────────────────────────
  sensor_msgs::msg::JointState joint_state_;
  std::string robot_description_;
  bool got_joints_{false}, got_urdf_{false}, got_ft_{false}, kdl_ready_{false};

  // ── KDL ────────────────────────────────────────────────────────────────────
  KDL::Chain chain_;
  KDL::JntArray joints_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

  // ── State machine ──────────────────────────────────────────────────────────
  Mode mode_{Mode::IDLE};
  int  current_layer_{0};
  int  total_layers_{0};
  bool circle_direction_cw_{true};
  bool homing_triggered_{false};
  int  home_retry_count_{0};
  static constexpr int max_home_retries_{3};

  // ── Timing ─────────────────────────────────────────────────────────────────
  rclcpp::Time approach_start_time_;
  rclcpp::Time circle_start_time_, last_update_time_;
  rclcpp::Time retract_start_time_;
  rclcpp::Time z_step_start_time_;
  rclcpp::Time final_retract_start_time_;
  rclcpp::Time safe_z_start_time_;

  // ── Position tracking ──────────────────────────────────────────────────────
  double current_x_{0.0}, current_y_{0.0};
  double current_layer_z_{0.0};
  double layer_step_z_{0.0};
  double dir_x_{0.0}, dir_y_{0.0};            // inward unit vector (approach sign convention)
  double approach_start_x_{0.0}, approach_start_y_{0.0};
  double retract_start_x_{0.0}, retract_start_y_{0.0};
  double retract_dir_x_{0.0}, retract_dir_y_{0.0};
  double z_step_start_z_{0.0}, z_step_target_z_{0.0};
  double final_retract_start_x_{0.0}, final_retract_start_y_{0.0};
  double final_retract_dir_x_{0.0}, final_retract_dir_y_{0.0};
  double safe_z_start_z_{0.0};

  // ── Force control ──────────────────────────────────────────────────────────
  double radius_{0.0}, theta_{0.0}, traveled_angle_{0.0};
  double measured_fz_{0.0}, force_integral_{0.0};

  // ── Orientation ────────────────────────────────────────────────────────────
  double qx_{0.0}, qy_{0.0}, qz_{0.0}, qw_{1.0};

  // ── MoveIt threading ───────────────────────────────────────────────────────
  std::thread home_thread_;
  std::atomic<bool> home_complete_{false};
  std::atomic<bool> home_failed_{false};

  // ── Parameters (loaded in readStartPose) ──────────────────────────────────
  double z_direction_{1.0};
  double vessel_height_{0.85};
  double vessel_r_min_{0.20}, vessel_r_max_{0.60};
  double blade_width_{0.03};
  double layer_overlap_fraction_{0.90};
  double start_z_offset_{0.0};
  double approach_distance_{0.35};
  double approach_duration_{7.0};
  double approach_timeout_{20.0};
  double angular_speed_{0.25};
  double max_angle_rad_{6.2832};
  double contact_force_threshold_{2.0};
  double desired_circle_force_{-10.0};
  double circle_ramp_fraction_{0.15};
  double high_force_threshold_{15.0};
  double max_force_clamp_{25.0};
  double min_speed_fraction_{0.15};
  double kp_radius_{0.5};
  double ki_radius_{0.005};
  double integral_limit_{2.0};
  double max_radius_rate_{0.01};
  double approach_force_z_{-5.0};
  double circle_force_z_{-3.0};
  double retract_distance_{0.15};
  double retract_duration_{3.0};
  double z_step_duration_{2.0};
  double final_retract_distance_{0.30};
  double final_retract_duration_{4.0};
  double safe_z_height_{0.20};
  double safe_z_duration_{3.0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CircleForce>());
  rclcpp::shutdown();
  return 0;
}

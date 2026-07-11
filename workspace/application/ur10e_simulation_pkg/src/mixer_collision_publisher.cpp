// mixer_collision_publisher
//
// Adds the planetary vertical mixer to the MoveIt planning scene as mesh
// collision objects and keeps their poses in sync with the mixer as it rotates.
//
// The mixer TF frames are published by /mixer/robot_state_publisher with the
// `mixer_` prefix (mixer_base_link, mixer_shroud_link, mixer_sun_link,
// mixer_planet_1_link, mixer_planet_2_link) and connect to the arm's planning
// frame through the static world->mixer_world transform. MoveIt bakes collision
// poses into the world at insertion time and does NOT track TF afterwards, so
// this node re-reads each link's transform on a timer and streams pose updates:
//
//   * first time a link's TF is available  -> CollisionObject ADD  (mesh + pose)
//   * every tick after                     -> CollisionObject MOVE (pose only)
//
// The per-link mesh offset reproduces the URDF <collision><origin>, so the
// world placement is TF(planning<-mixer_link) * mesh_offset.

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/mesh.hpp>

#include <geometric_shapes/shapes.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace
{
geometry_msgs::msg::Pose makePose(double x, double y, double z)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation.w = 1.0;   // all mixer collision origins have rpy = 0
  return p;
}
}  // namespace

class MixerCollisionPublisher : public rclcpp::Node
{
public:
  MixerCollisionPublisher()
  : Node("mixer_collision_publisher")
  {
    planning_frame_  = declare_parameter<std::string>("planning_frame", "world");
    scene_topic_     = declare_parameter<std::string>("planning_scene_topic", "/planning_scene");
    mesh_package_    = declare_parameter<std::string>("mesh_package", "ur10e_simulation_pkg");
    tf_prefix_       = declare_parameter<std::string>("tf_prefix", "mixer_");
    const double rate = declare_parameter<double>("update_rate_hz", 15.0);

    // One entry per mixer collision body: object id, mixer link (sans prefix),
    // mesh file, and the URDF <collision><origin> offset in that link's frame.
    defineLinks();
    loadMeshes();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    scene_pub_ = create_publisher<moveit_msgs::msg::PlanningScene>(scene_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MixerCollisionPublisher::tick, this));

    RCLCPP_INFO(get_logger(),
      "mixer_collision_publisher up: %zu mixer bodies -> planning frame '%s' at %.1f Hz",
      links_.size(), planning_frame_.c_str(), rate);
  }

private:
  struct Link
  {
    std::string id;                     // collision object id in the scene
    std::string frame;                  // full TF frame (with prefix)
    std::string mesh_file;              // filename under <mesh_package>/meshes
    geometry_msgs::msg::Pose offset;    // mesh pose in the link frame (URDF origin)
    shape_msgs::msg::Mesh mesh;         // loaded once
    bool loaded{false};
    bool added{false};                  // ADD sent once, then MOVE
  };

  void defineLinks()
  {
    auto add = [&](const std::string & id, const std::string & link,
                   const std::string & mesh, const geometry_msgs::msg::Pose & off) {
      Link l;
      l.id = id;
      l.frame = tf_prefix_ + link;
      l.mesh_file = mesh;
      l.offset = off;
      links_.push_back(l);
    };

    add("mixer_structure", "base_link",     "structure.stl",    makePose(0, 0, 0));
    add("mixer_shroud",    "shroud_link",   "shroud.stl",       makePose(0, 0, 0));
    add("mixer_sun",       "sun_link",      "middle_blade.stl", makePose(0, 0, 0));
    add("mixer_planet_1",  "planet_1_link", "blade_1.stl",      makePose(-0.155943,  0.456764, 0));
    add("mixer_planet_2",  "planet_2_link", "blade_2.stl",      makePose( 0.155847, -0.456481, 0));
  }

  void loadMeshes()
  {
    for (auto & l : links_)
    {
      const std::string uri = "package://" + mesh_package_ + "/meshes/" + l.mesh_file;
      std::unique_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource(uri));
      if (!mesh)
      {
        RCLCPP_ERROR(get_logger(), "Failed to load mesh %s", uri.c_str());
        continue;
      }
      shapes::ShapeMsg shape_msg;
      if (!shapes::constructMsgFromShape(mesh.get(), shape_msg))
      {
        RCLCPP_ERROR(get_logger(), "Failed to convert mesh %s to message", uri.c_str());
        continue;
      }
      l.mesh = boost::get<shape_msgs::msg::Mesh>(shape_msg);
      l.loaded = true;
      RCLCPP_INFO(get_logger(), "Loaded %s (%zu triangles)",
        l.mesh_file.c_str(), l.mesh.triangles.size());
    }
  }

  void tick()
  {
    // Don't publish (and don't burn the one-time ADD) until move_group is
    // actually subscribed to the planning-scene topic; otherwise the ADD is
    // dropped and every later MOVE references a nonexistent object.
    if (scene_pub_->get_subscription_count() == 0)
      return;

    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff = true;

    bool any = false;
    for (auto & l : links_)
    {
      if (!l.loaded)
        continue;

      geometry_msgs::msg::Pose pose;
      if (!lookupPose(l.frame, pose))
        continue;   // TF not ready yet; try again next tick

      moveit_msgs::msg::CollisionObject co;
      co.header.frame_id = planning_frame_;
      co.header.stamp = now();
      co.id = l.id;
      co.pose = pose;                 // object frame = mixer link in planning frame
      co.mesh_poses.push_back(l.offset);   // mesh sits at pose * offset (URDF origin)

      if (!l.added)
      {
        co.meshes.push_back(l.mesh);  // geometry sent only on the first ADD
        co.operation = moveit_msgs::msg::CollisionObject::ADD;
        l.added = true;
      }
      else
      {
        co.operation = moveit_msgs::msg::CollisionObject::MOVE;
      }

      scene.world.collision_objects.push_back(co);
      any = true;
    }

    if (any)
      scene_pub_->publish(scene);
  }

  bool lookupPose(const std::string & frame, geometry_msgs::msg::Pose & pose)
  {
    try
    {
      const auto tf = tf_buffer_->lookupTransform(
        planning_frame_, frame, tf2::TimePointZero, tf2::durationFromSec(0.05));
      pose.position.x = tf.transform.translation.x;
      pose.position.y = tf.transform.translation.y;
      pose.position.z = tf.transform.translation.z;
      pose.orientation = tf.transform.rotation;
      return true;
    }
    catch (const std::exception & e)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "No TF %s<-%s yet: %s", planning_frame_.c_str(), frame.c_str(), e.what());
      return false;
    }
  }

  std::string planning_frame_, scene_topic_, mesh_package_, tf_prefix_;
  std::vector<Link> links_;

  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MixerCollisionPublisher>());
  rclcpp::shutdown();
  return 0;
}

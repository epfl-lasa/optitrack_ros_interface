#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

#include "optitrack_ros2_interface/pose_transform.hpp"

namespace optitrack_ros2_interface {

using geometry_msgs::msg::PoseStamped;

/// Republishes OptiTrack rigid-body poses expressed in a robot base frame
/// instead of the mocap world frame.
///
/// Subscribes to <vrpn_ns>/<name_base>/pose once, and to <vrpn_ns>/<object>/pose
/// for every entry of list_object. Each object pose is transformed the moment it
/// arrives and published on <vrpn_ns>/<object>/pose_from_<name_base>.
class OptitrackTransformPublisher : public rclcpp::Node {
public:
  explicit OptitrackTransformPublisher(const rclcpp::NodeOptions& options)
      : rclcpp::Node("optitrack_ros2_interface", options) {
    name_base_ = declare_parameter<std::string>("name_base", "");
    base_frame_ = declare_parameter<std::string>("base_frame", "");
    vrpn_namespace_ = declare_parameter<std::string>("vrpn_namespace", "");
    output_suffix_ = declare_parameter<std::string>("output_suffix", "_from_");
    require_base_ = declare_parameter<bool>("require_base", true);
    const auto objects = declare_parameter<std::vector<std::string>>(
        "list_object", std::vector<std::string>{});
    const int qos_depth = declare_parameter<int>("qos_depth", 1);

    if (name_base_.empty()) {
      throw std::runtime_error("parameter 'name_base' is empty -- set it in config/optitrack_cfg.yaml");
    }
    if (objects.empty()) {
      throw std::runtime_error("parameter 'list_object' is empty -- set it in config/optitrack_cfg.yaml");
    }
    // An empty base_frame means "reuse the base rigid-body name", which is what
    // this node published before base_frame existed as a parameter.
    if (base_frame_.empty()) { base_frame_ = name_base_; }

    // Echo every resolved parameter. rclcpp silently ignores parameter
    // overrides for names that were never declared, and silently ignores
    // params-file entries whose top-level key does not match this node's name.
    // Neither produces a warning, so printing what actually resolved is the
    // only way to notice a typo before it costs you a run.
    RCLCPP_INFO(get_logger(), "name_base      = '%s'", name_base_.c_str());
    RCLCPP_INFO(get_logger(), "base_frame     = '%s'", base_frame_.c_str());
    RCLCPP_INFO(get_logger(), "vrpn_namespace = '%s'", vrpn_namespace_.c_str());
    RCLCPP_INFO(get_logger(), "require_base   = %s", require_base_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "list_object    = %zu entries", objects.size());

    rclcpp::QoS qos(rclcpp::KeepLast(static_cast<size_t>(qos_depth)));
    qos.best_effort();  // vrpn_mocap publishes with SensorDataQoS by default

    // One subscription to the base, shared by every object. The previous
    // version created one per object -- with N objects that is N independent
    // DDS DataReaders on the same topic, so every base sample was delivered and
    // deserialised N times and announced N extra endpoints to each remote
    // participant.
    const std::string base_topic = topicFor(name_base_);
    RCLCPP_INFO(get_logger(), "base topic: %s", base_topic.c_str());
    base_sub_ = create_subscription<PoseStamped>(
        base_topic, qos,
        [this](PoseStamped::ConstSharedPtr msg) { onBase(std::move(msg)); });

    objects_.reserve(objects.size());
    for (const auto& name : objects) {
      auto entry = std::make_unique<Entry>();
      entry->name = name;
      const std::string in_topic = topicFor(name);
      const std::string out_topic = in_topic + output_suffix_ + name_base_;
      entry->pub = create_publisher<PoseStamped>(out_topic, qos);

      // Held by unique_ptr inside the vector, so the pointer stays valid even if
      // the vector reallocates. Captured by value -- the previous version
      // captured a main() local by reference.
      Entry* raw = entry.get();
      entry->sub = create_subscription<PoseStamped>(
          in_topic, qos,
          [this, raw](PoseStamped::ConstSharedPtr msg) { onObject(*raw, std::move(msg)); });

      RCLCPP_INFO(get_logger(), "%s -> %s", in_topic.c_str(), out_topic.c_str());
      objects_.push_back(std::move(entry));
    }

    RCLCPP_WARN(get_logger(), "WARNING! Verify that Y-up frame is applied on motive");
  }

private:
  struct Entry {
    std::string name;
    rclcpp::Subscription<PoseStamped>::SharedPtr sub;
    rclcpp::Publisher<PoseStamped>::SharedPtr pub;
  };

  std::string topicFor(const std::string& rigid_body) const {
    const std::string ns = vrpn_namespace_.empty() ? std::string() : "/" + vrpn_namespace_;
    return ns + "/" + rigid_body + "/pose";
  }

  void onBase(PoseStamped::ConstSharedPtr msg) {
    base_pose_ = optitrack::makeIsometry(
        msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
        msg->pose.orientation.x, msg->pose.orientation.y,
        msg->pose.orientation.z, msg->pose.orientation.w);
    base_received_ = true;
  }

  void onObject(const Entry& entry, PoseStamped::ConstSharedPtr msg) {
    // Without this guard, an object sample arriving before the first base sample
    // is transformed against a default-constructed Eigen pose -- which is
    // uninitialised memory, not identity -- and published as plausible-looking
    // garbage. That is the normal case at startup, and the permanent case when
    // the base rigid body is misnamed or untracked.
    if (require_base_ && !base_received_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "no pose received yet for base '%s'; not publishing '%s'. "
                           "Is the rigid body tracked, and is the name correct?",
                           name_base_.c_str(), entry.name.c_str());
      return;
    }

    const Eigen::Isometry3d object = optitrack::makeIsometry(
        msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
        msg->pose.orientation.x, msg->pose.orientation.y,
        msg->pose.orientation.z, msg->pose.orientation.w);
    const Eigen::Isometry3d out = optitrack::relativePose(base_pose_, object);
    const Eigen::Quaterniond q(out.linear());

    PoseStamped result;
    result.header.stamp = msg->header.stamp;  // preserve the source timestamp
    result.header.frame_id = base_frame_;
    result.pose.position.x = out.translation().x();
    result.pose.position.y = out.translation().y();
    result.pose.position.z = out.translation().z();
    result.pose.orientation.x = q.x();
    result.pose.orientation.y = q.y();
    result.pose.orientation.z = q.z();
    result.pose.orientation.w = q.w();
    entry.pub->publish(result);  // event driven -- no polling loop
  }

  std::string name_base_;
  std::string base_frame_;
  std::string vrpn_namespace_;
  std::string output_suffix_;
  bool require_base_{true};

  // Thread safety: the default single-threaded executor and the default
  // (mutually exclusive) callback group guarantee onBase and onObject never run
  // concurrently, so base_pose_ needs no mutex. Switching to a
  // MultiThreadedExecutor or a reentrant callback group makes this a data race.
  Eigen::Isometry3d base_pose_{Eigen::Isometry3d::Identity()};
  bool base_received_{false};

  rclcpp::Subscription<PoseStamped>::SharedPtr base_sub_;
  std::vector<std::unique_ptr<Entry>> objects_;
};

}  // namespace optitrack_ros2_interface

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<optitrack_ros2_interface::OptitrackTransformPublisher>(
      rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}

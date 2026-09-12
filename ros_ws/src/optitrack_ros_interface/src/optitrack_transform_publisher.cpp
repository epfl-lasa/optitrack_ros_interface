#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"

#include "optitrack_ros_interface/pose_transform.hpp"

namespace optitrack_ros_interface {

/// Republishes OptiTrack rigid-body poses expressed in a robot base frame
/// instead of the mocap world frame. ROS1 counterpart of the ROS2 node; the
/// maths is shared via pose_transform.hpp, the plumbing is not.
class OptitrackTransformPublisher {
public:
  OptitrackTransformPublisher(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
    std::vector<std::string> objects;
    pnh.param<std::string>("name_base", name_base_, "");
    pnh.param<std::string>("base_frame", base_frame_, "");
    // The vrpn client publishes under its own node name. This used to be
    // hardcoded to "/vrpn_client_node", so renaming that node in the launch file
    // silently left this one subscribed to topics nobody publishes.
    pnh.param<std::string>("vrpn_namespace", vrpn_namespace_, "/vrpn_client_node");
    pnh.param<std::string>("output_suffix", output_suffix_, "_from_");
    pnh.param<bool>("require_base", require_base_, true);
    pnh.getParam("list_object", objects);

    if (name_base_.empty()) {
      throw std::runtime_error("parameter 'name_base' is empty -- set it in config/optitrack_cfg.yaml");
    }
    if (objects.empty()) {
      throw std::runtime_error("parameter 'list_object' is empty -- set it in config/optitrack_cfg.yaml");
    }
    // Empty base_frame means "reuse the base rigid-body name", matching the ROS2
    // node so both stacks stamp the same frame_id.
    if (base_frame_.empty()) { base_frame_ = name_base_; }

    // Echo what actually resolved: a params-file key that does not match the
    // node name is ignored without any warning.
    ROS_INFO("name_base      = '%s'", name_base_.c_str());
    ROS_INFO("base_frame     = '%s'", base_frame_.c_str());
    ROS_INFO("vrpn_namespace = '%s'", vrpn_namespace_.c_str());
    ROS_INFO("require_base   = %s", require_base_ ? "true" : "false");
    ROS_INFO("list_object    = %zu entries", objects.size());

    // One subscription to the base, shared by every object, rather than one per
    // object as before.
    const std::string base_topic = topicFor(name_base_);
    ROS_INFO("base topic: %s", base_topic.c_str());
    base_sub_ = nh.subscribe(base_topic, 1, &OptitrackTransformPublisher::onBase, this);

    objects_.reserve(objects.size());
    for (const auto& name : objects) {
      auto entry = std::make_unique<Entry>();
      entry->name = name;
      const std::string in_topic = topicFor(name);
      const std::string out_topic = in_topic + output_suffix_ + name_base_;
      entry->pub = nh.advertise<geometry_msgs::PoseStamped>(out_topic, 1);

      Entry* raw = entry.get();
      entry->sub = nh.subscribe<geometry_msgs::PoseStamped>(
          in_topic, 1,
          [this, raw](const geometry_msgs::PoseStamped::ConstPtr& msg) { onObject(*raw, msg); });

      ROS_INFO("%s -> %s", in_topic.c_str(), out_topic.c_str());
      objects_.push_back(std::move(entry));
    }

    ROS_WARN("WARNING! Verify that Y-up frame is applied on motive");
  }

private:
  struct Entry {
    std::string name;
    ros::Subscriber sub;
    ros::Publisher pub;
  };

  std::string topicFor(const std::string& rigid_body) const {
    return vrpn_namespace_ + "/" + rigid_body + "/pose";
  }

  void onBase(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    base_pose_ = optitrack::makeIsometry(
        msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
        msg->pose.orientation.x, msg->pose.orientation.y,
        msg->pose.orientation.z, msg->pose.orientation.w);
    base_received_ = true;
  }

  void onObject(const Entry& entry, const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // See the ROS2 node: a default-constructed Eigen pose is uninitialised
    // memory, not identity, so transforming against it before the first base
    // sample publishes plausible-looking garbage.
    if (require_base_ && !base_received_) {
      ROS_WARN_THROTTLE(2.0,
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

    geometry_msgs::PoseStamped result;
    result.header.stamp = msg->header.stamp;
    result.header.frame_id = base_frame_;
    result.pose.position.x = out.translation().x();
    result.pose.position.y = out.translation().y();
    result.pose.position.z = out.translation().z();
    result.pose.orientation.x = q.x();
    result.pose.orientation.y = q.y();
    result.pose.orientation.z = q.z();
    result.pose.orientation.w = q.w();
    entry.pub.publish(result);  // event driven -- replaces the 1 kHz poll loop
  }

  std::string name_base_;
  std::string base_frame_;
  std::string vrpn_namespace_;
  std::string output_suffix_;
  bool require_base_{true};

  Eigen::Isometry3d base_pose_{Eigen::Isometry3d::Identity()};
  bool base_received_{false};

  ros::Subscriber base_sub_;
  std::vector<std::unique_ptr<Entry>> objects_;
};

}  // namespace optitrack_ros_interface

int main(int argc, char** argv) {
  ros::init(argc, argv, "optitrack_ros_interface");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    optitrack_ros_interface::OptitrackTransformPublisher node(nh, pnh);
    ros::spin();
  } catch (const std::exception& e) {
    ROS_FATAL("%s", e.what());
    return 1;
  }
  return 0;
}

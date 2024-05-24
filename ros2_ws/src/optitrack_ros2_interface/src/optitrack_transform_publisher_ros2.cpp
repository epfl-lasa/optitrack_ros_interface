#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

using namespace Eigen;
using namespace std;

class Vrpn {
public:
  Vector3d pos, pos_obj;
  Vector4d quat, quat_obj;
  geometry_msgs::msg::PoseStamped msgP;
  rclcpp::Time time;
  bool to_publish = false;

  void CC_vrpn_base(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    pos = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    quat = {msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w};
  }

  void CC_vrpn_obj(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    pos_obj = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
    quat_obj = {msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w};
    time = msg->header.stamp;
    to_publish = true;
  }

  void transform_new_base(const std::string& inputString) {
    Quaterniond q(quat[3], quat[0], quat[1], quat[2]);
    Matrix3d R = q.toRotationMatrix();

    MatrixXd M(4, 4);
    M << R(0, 0), R(0, 1), R(0, 2), pos[0], R(1, 0), R(1, 1), R(1, 2), pos[1], R(2, 0), R(2, 1), R(2, 2), pos[2], 0, 0,
        0, 1;

    Quaterniond q_obj(quat_obj[3], quat_obj[0], quat_obj[1], quat_obj[2]);
    Matrix3d R_obj = q_obj.toRotationMatrix();
    MatrixXd M_obj(4, 4), M_out(4, 4);
    M_obj << R_obj(0, 0), R_obj(0, 1), R_obj(0, 2), pos_obj[0], R_obj(1, 0), R_obj(1, 1), R_obj(1, 2), pos_obj[1],
        R_obj(2, 0), R_obj(2, 1), R_obj(2, 2), pos_obj[2], 0, 0, 0, 1;

    M_out = M.inverse() * M_obj;

    Matrix3d R_out = M_out.block<3, 3>(0, 0);
    Quaterniond q_out(R_out);
    q_out.normalize();

    msgP.header.frame_id = inputString;
    msgP.pose.position.x = M_out(0, 3);
    msgP.pose.position.y = M_out(1, 3);
    msgP.pose.position.z = M_out(2, 3);
    msgP.pose.orientation.x = q_out.x();
    msgP.pose.orientation.y = q_out.y();
    msgP.pose.orientation.z = q_out.z();
    msgP.pose.orientation.w = q_out.w();
  }
};

std::vector<Vrpn> list_objects;
std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> list_pub;
std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> list_sub_base;
std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> list_sub;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("optitrack_transform_publisher_ros2");

  // Declare the parameter with a default value (optional but recommended)
  node->declare_parameter<std::string>("name_base", "default_value");
  node->declare_parameter<std::vector<std::string>>("list_object", {});

  std::string name_base;
  if (node->get_parameter("name_base", name_base)) {
    RCLCPP_INFO(node->get_logger(), "name_base: %s", name_base.c_str());
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to get parameter 'name_base'");
  }

  std::vector<std::string> subscribedObjects;
  if (node->get_parameter("list_object", subscribedObjects)) {
    RCLCPP_INFO(node->get_logger(), "List of topics contains:");
    for (const auto& object : subscribedObjects) { RCLCPP_INFO(node->get_logger(), "%s", object.c_str()); }
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to get parameter 'list_object'");
  }

  RCLCPP_INFO(node->get_logger(), "Base topic is:");
  std::string topic_base = name_base + "/pose";
  RCLCPP_INFO(node->get_logger(), "%s", topic_base.c_str());

  std::vector<std::string> subscribedTopics;
  for (const auto& topic : subscribedObjects) {
    subscribedTopics.push_back(topic + "/pose");
    RCLCPP_INFO(node->get_logger(), "%s", subscribedTopics.back().c_str());

    Vrpn object;
    list_objects.emplace_back(object);
  }

  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
  qos.keep_last(10);

  size_t list_n = list_objects.size();
  for (size_t i = 0; i < list_n; ++i) {
    list_sub_base.emplace_back(node->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic_base,
        qos,
        [i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { list_objects[i].CC_vrpn_base(msg); }));
    list_sub.emplace_back(node->create_subscription<geometry_msgs::msg::PoseStamped>(
        subscribedTopics[i],
        qos,
        [i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { list_objects[i].CC_vrpn_obj(msg); }));
    list_pub.emplace_back(
        node->create_publisher<geometry_msgs::msg::PoseStamped>(subscribedTopics[i] + "_from_" + name_base, qos));
  }

  rclcpp::Rate loop_rate(1000);

  RCLCPP_WARN(node->get_logger(), "WARNING! Verify that Y-up frame is applied on motive");
  while (rclcpp::ok()) {
    for (size_t i = 0; i < list_n; ++i) {
      if (list_objects[i].to_publish) {
        list_objects[i].transform_new_base(name_base);

        list_objects[i].msgP.header.stamp = list_objects[i].time;
        list_pub[i]->publish(list_objects[i].msgP);
        list_objects[i].to_publish = false;
      }
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

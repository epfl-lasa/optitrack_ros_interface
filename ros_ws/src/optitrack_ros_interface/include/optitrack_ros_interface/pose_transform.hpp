#pragma once

// Pure geometry helpers, free of any ROS types, so the transform maths can be
// unit-tested without a running graph. Kept deliberately duplicated in the ROS2
// package rather than shared across workspaces: the two workspaces must stay
// independently buildable, and a cross-workspace include path breaks as soon as
// either package is copied elsewhere.

#include <Eigen/Geometry>

namespace optitrack {

/// Build a rigid transform from a position and an (x, y, z, w) quaternion.
///
/// The quaternion is normalised on the way in. VRPN quaternions are nominally
/// unit, but a non-unit input would otherwise yield a non-orthonormal rotation
/// matrix and silently skew every pose derived from it.
inline Eigen::Isometry3d makeIsometry(double px, double py, double pz,
                                      double qx, double qy, double qz, double qw) {
  Eigen::Quaterniond q(qw, qx, qy, qz);
  q.normalize();
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = q.toRotationMatrix();
  t.translation() << px, py, pz;
  return t;
}

/// Pose of `object` expressed in `base`:  T_base_obj = (T_world_base)^-1 * T_world_obj
///
/// Isometry3d::inverse() transposes the rotation block and negates the rotated
/// translation. The previous implementation built a dynamic-size MatrixXd(4, 4)
/// and called .inverse() on it, which heap-allocates and runs a PartialPivLU
/// factorisation -- both wasted on a matrix that is known to be rigid.
inline Eigen::Isometry3d relativePose(const Eigen::Isometry3d& base,
                                      const Eigen::Isometry3d& object) {
  return base.inverse() * object;
}

}  // namespace optitrack

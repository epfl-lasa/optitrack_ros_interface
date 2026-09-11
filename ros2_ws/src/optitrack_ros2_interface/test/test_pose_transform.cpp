#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include "optitrack_ros2_interface/pose_transform.hpp"

namespace {

constexpr double kEps = 1e-9;

TEST(PoseTransform, IdentityQuaternionGivesPureTranslation) {
  const auto t = optitrack::makeIsometry(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0);
  EXPECT_TRUE(t.linear().isApprox(Eigen::Matrix3d::Identity(), kEps));
  EXPECT_NEAR(t.translation().x(), 1.0, kEps);
  EXPECT_NEAR(t.translation().y(), 2.0, kEps);
  EXPECT_NEAR(t.translation().z(), 3.0, kEps);
}

TEST(PoseTransform, NonUnitQuaternionIsNormalised) {
  // A caller passing an unnormalised quaternion must still get an orthonormal
  // rotation, otherwise every derived pose is silently skewed.
  const auto t = optitrack::makeIsometry(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0);
  EXPECT_TRUE(t.linear().isApprox(Eigen::Matrix3d::Identity(), kEps));
  EXPECT_NEAR(t.linear().determinant(), 1.0, kEps);
}

TEST(PoseTransform, SamePoseGivesIdentity) {
  // Object coincident with the base must transform to the origin. This is the
  // invariant the synthetic end-to-end check exercises.
  const auto base = optitrack::makeIsometry(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0);
  const auto out = optitrack::relativePose(base, base);
  EXPECT_TRUE(out.isApprox(Eigen::Isometry3d::Identity(), kEps));
}

TEST(PoseTransform, PureTranslationOffset) {
  const auto base = optitrack::makeIsometry(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  const auto object = optitrack::makeIsometry(2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  const auto out = optitrack::relativePose(base, object);
  EXPECT_NEAR(out.translation().x(), 1.0, kEps);
  EXPECT_NEAR(out.translation().y(), 0.0, kEps);
  EXPECT_NEAR(out.translation().z(), 0.0, kEps);
}

TEST(PoseTransform, RotatedBaseRotatesTheOffset) {
  // Base yawed +90 deg about Z at the origin; object one metre along world +X.
  // Expressed in the base frame the object sits one metre along the base's -Y.
  const Eigen::Quaterniond yaw(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()));
  const auto base = optitrack::makeIsometry(0.0, 0.0, 0.0, yaw.x(), yaw.y(), yaw.z(), yaw.w());
  const auto object = optitrack::makeIsometry(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  const auto out = optitrack::relativePose(base, object);
  EXPECT_NEAR(out.translation().x(), 0.0, 1e-9);
  EXPECT_NEAR(out.translation().y(), -1.0, 1e-9);
  EXPECT_NEAR(out.translation().z(), 0.0, 1e-9);
}

TEST(PoseTransform, MatchesDenseMatrixImplementation) {
  // Guards the MatrixXd(4,4).inverse() -> Isometry3d::inverse() change: the new
  // implementation must agree with the old one on an arbitrary rigid pair.
  const Eigen::Quaterniond qb =
      Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d(1, 2, 3).normalized()));
  const Eigen::Quaterniond qo =
      Eigen::Quaterniond(Eigen::AngleAxisd(-1.1, Eigen::Vector3d(-2, 1, 0.5).normalized()));
  const Eigen::Vector3d pb(0.4, -1.2, 3.3);
  const Eigen::Vector3d po(-2.1, 0.7, 1.9);

  Eigen::Matrix4d mb = Eigen::Matrix4d::Identity();
  mb.block<3, 3>(0, 0) = qb.toRotationMatrix();
  mb.block<3, 1>(0, 3) = pb;
  Eigen::Matrix4d mo = Eigen::Matrix4d::Identity();
  mo.block<3, 3>(0, 0) = qo.toRotationMatrix();
  mo.block<3, 1>(0, 3) = po;
  const Eigen::Matrix4d expected = mb.inverse() * mo;

  const auto base = optitrack::makeIsometry(pb.x(), pb.y(), pb.z(), qb.x(), qb.y(), qb.z(), qb.w());
  const auto object = optitrack::makeIsometry(po.x(), po.y(), po.z(), qo.x(), qo.y(), qo.z(), qo.w());
  const auto out = optitrack::relativePose(base, object);

  EXPECT_TRUE(out.matrix().isApprox(expected, 1e-9));
}

}  // namespace

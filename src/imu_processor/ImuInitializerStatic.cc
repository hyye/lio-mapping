//
// Created by hyye on 2/13/20.
//

#include "imu_processor/ImuInitializerStatic.h"

namespace lio {

bool ImuInitializerStatic::Initialization(
    CircularBuffer<PairTimeLaserTransform> &all_laser_transforms,
    CircularBuffer<Vector3d> &Vs, CircularBuffer<Vector3d> &Bas,
    CircularBuffer<Vector3d> &Bgs, Vector3d &g, Transform &transform_lb,
    Matrix3d &R_WI) {
  size_t window_size = all_laser_transforms.size() - 1;

  // TODO: to be corrected for static reading?
  for (int i = 0; i <= window_size; ++i) {
    Vs[i].setZero();
    Bas[i].setZero();
    Bgs[i].setZero();
  }

  double g_norm = all_laser_transforms.first().second.pre_integration->config_.g_norm;
  Eigen::Vector3d acc0 = all_laser_transforms.first().second.pre_integration->acc0_;
  Eigen::Vector3d unit_z(0, 0, 1);
  // IMU frame to inertial frame
  Eigen::Quaterniond q_ib = Eigen::Quaterniond::FromTwoVectors(acc0, unit_z);
  Eigen::Quaterniond q_il = q_ib * transform_lb.rot.inverse().cast<double>();

  g = q_il.inverse() * Eigen::Vector3d(0, 0, -1) * g_norm;
  // inertial frame to Lidar's world frame
  R_WI = q_il.inverse().toRotationMatrix();

  return true;
}

} // namespace lio
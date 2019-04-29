/**
* This file is part of LIO-mapping.
* 
* Copyright (C) 2019 Haoyang Ye <hy.ye at connect dot ust dot hk>,
* Robotics and Multiperception Lab (RAM-LAB <https://ram-lab.com>),
* The Hong Kong University of Science and Technology
* 
* For more information please see <https://ram-lab.com/file/hyye/lio-mapping>
* or <https://sites.google.com/view/lio-mapping>.
* If you use this code, please cite the respective publications as
* listed on the above websites.
* 
* LIO-mapping is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* LIO-mapping is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with LIO-mapping.  If not, see <http://www.gnu.org/licenses/>.
*/

//
// Created by hyye on 3/26/18.
//

#include "imu_processor/ImuInitializer.h"

namespace lio {

void EstimateGyroBias(CircularBuffer<PairTimeLaserTransform> &all_laser_transforms,
                      CircularBuffer<Vector3d> &Bgs) {
  Matrix3d A;
  Vector3d b;
  Vector3d delta_bg;
  A.setZero();
  b.setZero();

//  for (int i = 0; i < pre_integrations.size(); ++i) {
//    DLOG(INFO) << pre_integrations[i]->acc0_.transpose();
//  }

  size_t window_size = all_laser_transforms.size() - 1;

  for (size_t i = 0; i < window_size; ++i) {
    PairTimeLaserTransform &laser_trans_i = all_laser_transforms[i];
    PairTimeLaserTransform &laser_trans_j = all_laser_transforms[i + 1];

    MatrixXd tmp_A(3, 3);
    tmp_A.setZero();
    VectorXd tmp_b(3);
    tmp_b.setZero();
    Eigen::Quaterniond q_ij(laser_trans_i.second.transform.rot.conjugate() * laser_trans_j.second.transform.rot);
    tmp_A = laser_trans_j.second.pre_integration->jacobian_.template block<3, 3>(O_R, O_BG); /// Jacobian of dr12_bg
    tmp_b = 2 * (laser_trans_j.second.pre_integration->delta_q_.conjugate() * q_ij).vec(); /// 2*vec(IMU_ij^T * q_ij)
    A += tmp_A.transpose() * tmp_A;
    b += tmp_A.transpose() * tmp_b;

  }
  delta_bg = A.ldlt().solve(b);
  DLOG(WARNING) << "gyroscope bias initial calibration: " << delta_bg.transpose();

  for (int i = 0; i <= window_size; ++i) {
    Bgs[i] += delta_bg;
  }

  for (size_t i = 0; i < window_size; ++i) {
    PairTimeLaserTransform &laser_trans_j = all_laser_transforms[i + 1];
    laser_trans_j.second.pre_integration->Repropagate(Vector3d::Zero(), Bgs[0]);
  }

}

bool ApproximateGravity(CircularBuffer<PairTimeLaserTransform> &all_laser_transforms, Vector3d &g,
                        Transform &transform_lb) {

  size_t num_states = 3;
  size_t window_size = all_laser_transforms.size() - 1;

  if (window_size < 5) {
    DLOG(WARNING) << ">>>>>>> window size not enough <<<<<<<";
    return false;
  }

  Matrix3d I3x3;
  I3x3.setIdentity();

  MatrixXd A{num_states, num_states};
  A.setZero();
  VectorXd b{num_states};
  b.setZero();

  Vector3d &g_approx = g;

  for (size_t i = 0; i < window_size - 1; ++i) {
    PairTimeLaserTransform &laser_trans_i = all_laser_transforms[i];
    PairTimeLaserTransform &laser_trans_j = all_laser_transforms[i + 1];
    PairTimeLaserTransform &laser_trans_k = all_laser_transforms[i + 2];

    double dt12 = laser_trans_j.second.pre_integration->sum_dt_;
    double dt23 = laser_trans_k.second.pre_integration->sum_dt_;

    Vector3d dp12 = laser_trans_j.second.pre_integration->delta_p_.template cast<double>();
    Vector3d dp23 = laser_trans_k.second.pre_integration->delta_p_.template cast<double>();
    Vector3d dv12 = laser_trans_j.second.pre_integration->delta_v_.template cast<double>();

    Vector3d pl1 = laser_trans_i.second.transform.pos.template cast<double>();
    Vector3d pl2 = laser_trans_j.second.transform.pos.template cast<double>();
    Vector3d pl3 = laser_trans_k.second.transform.pos.template cast<double>();
    Vector3d plb = transform_lb.pos.template cast<double>();

    Matrix3d rl1 = laser_trans_i.second.transform.rot.template cast<double>().toRotationMatrix();
    Matrix3d rl2 = laser_trans_j.second.transform.rot.template cast<double>().toRotationMatrix();
    Matrix3d rl3 = laser_trans_k.second.transform.rot.template cast<double>().toRotationMatrix();
    Matrix3d rlb = transform_lb.rot.template cast<double>().toRotationMatrix();

    MatrixXd tmp_A(3, 3);
    tmp_A.setZero();
    VectorXd tmp_b(3);
    tmp_b.setZero();

    tmp_A = 0.5 * I3x3 * (dt12 * dt12 * dt23 + dt23 * dt23 * dt12);
    tmp_b = (pl2 - pl1) * dt23 - (pl3 - pl2) * dt12
        + (rl2 - rl1) * plb * dt23 - (rl3 - rl2) * plb * dt12
        + rl2 * rlb * dp23 * dt12 + rl1 * rlb * dv12 * dt12 * dt23
        - rl1 * rlb * dp12 * dt23;

    A += tmp_A.transpose() * tmp_A;
    b -= tmp_A.transpose() * tmp_b;

//    A += tmp_A;
//    b -= tmp_b;

  }

  A = A * 10000.0;
  b = b * 10000.0;

//  DLOG(INFO) << "A" << endl << A;
//  DLOG(INFO) << "b" << endl << b;

  g_approx = A.ldlt().solve(b);

//  g_approx = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

  DLOG(INFO) << "g_approx: " << g_approx.transpose();

  // TODO: verify g

  double g_norm = all_laser_transforms.first().second.pre_integration->config_.g_norm;

  return fabs(g_approx.norm() - g_norm) <= 1.0;

}

void RefineGravityAccBias(CircularBuffer<PairTimeLaserTransform> &all_laser_transforms,
                          CircularBuffer<Vector3d> &Bas,
                          CircularBuffer<Vector3d> &Bgs,
                          Vector3d &g_approx,
                          Transform &transform_lb,
                          Matrix3d &R_WI) {

  typedef Sophus::SO3d SO3;

  Vector3d &g_refined = g_approx;

  size_t window_size = all_laser_transforms.size() - 1;
  size_t num_states = 5;

  if (window_size < 5) {
    LOG(WARNING)
        << ">>>>>>> window size not enough <<<<<<<";
    return;
  }

  Matrix3d I3x3;
  I3x3.setIdentity();

  MatrixXd A{3, num_states};
  A.setZero();
  VectorXd b{3};
  b.setZero();
  VectorXd x{num_states};
  x.setZero();

  double g_norm = all_laser_transforms.first().second.pre_integration->config_.g_norm;

  Vector3d gI_n{0.0, 0.0, -1.0};
  Vector3d gW_n = g_approx.normalized(); // NOTE: the Lidar's world frame
  Vector3d gIxgW = gI_n.cross(gW_n);
  Vector3d v_WI = gIxgW / gIxgW.norm();
  double ang_WI = atan2(gIxgW.norm(), gI_n.dot(gW_n));

  Eigen::Matrix3d r_WI(SO3::exp(ang_WI * v_WI).unit_quaternion());

//  DLOG(INFO) << (r_WI * gI_n * g_approx.norm()).transpose();
//  DLOG(INFO) << g_approx.transpose();

  Vector3d delta_ba;

  for (int k = 0; k < 5; ++k) {

    for (size_t i = 0; i < window_size - 1; ++i) {
      PairTimeLaserTransform &laser_trans_i = all_laser_transforms[i];
      PairTimeLaserTransform &laser_trans_j = all_laser_transforms[i + 1];
      PairTimeLaserTransform &laser_trans_k = all_laser_transforms[i + 2];

      double dt12 = laser_trans_j.second.pre_integration->sum_dt_;
      double dt23 = laser_trans_k.second.pre_integration->sum_dt_;

      Vector3d dp12 = laser_trans_j.second.pre_integration->delta_p_;
      Vector3d dp23 = laser_trans_k.second.pre_integration->delta_p_;
      Vector3d dv12 = laser_trans_j.second.pre_integration->delta_v_;

      Vector3d pl1 = laser_trans_i.second.transform.pos.template cast<double>();
      Vector3d pl2 = laser_trans_j.second.transform.pos.template cast<double>();
      Vector3d pl3 = laser_trans_k.second.transform.pos.template cast<double>();
      Vector3d plb = transform_lb.pos.template cast<double>();

      Matrix3d rl1 = laser_trans_i.second.transform.rot.normalized().template cast<double>().toRotationMatrix();
      Matrix3d rl2 = laser_trans_j.second.transform.rot.normalized().template cast<double>().toRotationMatrix();
      Matrix3d rl3 = laser_trans_k.second.transform.rot.normalized().template cast<double>().toRotationMatrix();
      Matrix3d rlb = transform_lb.rot.normalized().template cast<double>().toRotationMatrix();

      Matrix3d J_dp12_ba = laser_trans_j.second.pre_integration->jacobian_.template block<3, 3>(O_P, O_BA);
      Matrix3d J_dp23_ba = laser_trans_k.second.pre_integration->jacobian_.template block<3, 3>(O_P, O_BA);
      Matrix3d J_dv12_ba = laser_trans_j.second.pre_integration->jacobian_.template block<3, 3>(O_V, O_BA);

      MatrixXd tmp_A(3, 5);
      tmp_A.setZero();
      VectorXd tmp_b(3);
      tmp_b.setZero();

      MatrixXd tmp_phi(3, 3); // NOTE: only 0 and 1 columns will be used
      tmp_phi.setZero();
      MatrixXd tmp_zeta(3, 3);
      tmp_zeta.setZero();

      tmp_phi = -0.5 * r_WI * SO3::hat(gI_n) * g_norm * (dt12 * dt12 * dt23 + dt23 * dt23 * dt12);
      tmp_zeta = rl2 * rlb * J_dp23_ba * dt12 + rl1 * rlb * J_dv12_ba * dt12 * dt23
          - rl1 * rlb * J_dp12_ba * dt23;
      tmp_b = (pl2 - pl1) * dt23 - (pl3 - pl2) * dt12
          + (rl2 - rl1) * plb * dt23 - (rl3 - rl2) * plb * dt12
          + rl2 * rlb * dp23 * dt12 + rl1 * rlb * dv12 * dt12 * dt23
          - rl1 * rlb * dp12 * dt23 + 0.5 * r_WI * gI_n * g_norm * (dt12 * dt12 * dt23 + dt23 * dt23 * dt12);

      tmp_A.topLeftCorner<3, 2>() = tmp_phi.topLeftCorner<3, 2>();
      tmp_A.bottomRightCorner<3, 3>() = tmp_zeta;

// NOTE: is ti necessary to add IMU cov?
//    Matrix<double, 3, 3> cov_inv = Matrix<double, 3, 3>::Zero();
//    cov_inv.setIdentity();
//
//    MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
//    VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

//    cout << tmp_phi << endl;
//    cout << tmp_zeta << endl;
//    cout << tmp_A << endl;
//    cout << tmp_b << endl;
//    cout << J_dp12_ba << endl;
//    cout << J_dp23_ba << endl;
//    cout << J_dv12_ba << endl;

      A += tmp_A;
      b -= tmp_b;

    }

//    MatrixXd AtA = A.transpose() * A;
//    MatrixXd Atb = A.transpose() * b;
//
//    AtA = AtA * 1000.0;
//    Atb = Atb * 1000.0;
//    x = AtA.ldlt().solve(Atb);

    A *= 1000.0;
    b *= 1000.0;

    x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

    DLOG(INFO) << "k: " << k << ", x: " << x.transpose();

    // TODO: verify if the gravity is right
    R_WI = r_WI * SO3::exp(Vector3d(x(0), x(1), 0.0)).unit_quaternion();
    // WARNING
    r_WI = R_WI;
    g_refined = R_WI * gI_n * g_norm;
    delta_ba = x.segment<3>(2);

    for (int i = 0; i <= window_size; ++i) {
//      Bas[i] += delta_ba;
      Bas[i] = delta_ba;
    }

    for (size_t i = 0; i < window_size; ++i) {
      PairTimeLaserTransform &laser_trans_j = all_laser_transforms[i + 1];
      laser_trans_j.second.pre_integration->Repropagate(Bas[0], Bgs[0]);
    }

  }

//  cout << delta_ba << endl;

//  DLOG(WARNING) << "refined gravity:  " << g_refined.transpose();
//  DLOG(WARNING) << "refined gravity in the inertial frame:  " << (R_WI.transpose() * g_refined).transpose();
  DLOG(WARNING) << "acceleration bias initial calibration:  " << Bas[0].transpose();

//  for (int i = 0; i <= window_size; ++i) {
//    Bas[i] += delta_ba;
//  }
//
//  for (size_t i = 0; i < window_size; ++i) {
//    laser_trans_i = all_laser_transforms[i];
//    laser_trans_j = all_laser_transforms[i + 1];
//
//    laser_trans_j.second.pre_integration->Repropagate(Bas[0], Bgs[0]);
//  }

}

void UpdateVelocities(CircularBuffer<PairTimeLaserTransform> &all_laser_transforms,
                      CircularBuffer<Vector3d> &Vs,
                      CircularBuffer<Vector3d> &Bas,
                      CircularBuffer<Vector3d> &Bgs,
                      Vector3d &g,
                      Transform &transform_lb) {

  size_t window_size = all_laser_transforms.size() - 1;

  double dt12;
  Vector3d dp12;
  Vector3d pl1;
  Vector3d pl2;
  Vector3d plb;
  Matrix3d rl1;
  Matrix3d rl2;
  Matrix3d rlb;

  for (size_t i = 0; i < window_size; ++i) {
    PairTimeLaserTransform &laser_trans_i = all_laser_transforms[i];
    PairTimeLaserTransform &laser_trans_j = all_laser_transforms[i + 1];

    dt12 = laser_trans_j.second.pre_integration->sum_dt_;
    dp12 = laser_trans_j.second.pre_integration->delta_p_;

    pl1 = laser_trans_i.second.transform.pos.template cast<double>();
    pl2 = laser_trans_j.second.transform.pos.template cast<double>();
    plb = transform_lb.pos.template cast<double>();

    rl1 = laser_trans_i.second.transform.rot.normalized().template cast<double>().toRotationMatrix();
    rl2 = laser_trans_j.second.transform.rot.normalized().template cast<double>().toRotationMatrix();
    rlb = transform_lb.rot.normalized().template cast<double>().toRotationMatrix();

    Vs[i] = (pl2 - pl1 - 0.5 * g * dt12 * dt12 - rl1 * rlb * dp12 - (rl1 - rl2) * plb) / dt12;
  }

  dt12 = all_laser_transforms.last().second.pre_integration->sum_dt_;
  Vector3d dv12 = all_laser_transforms.last().second.pre_integration->delta_v_;

  Vs[window_size] = Vs[window_size - 1] + g * dt12 + rl1 * rlb * (dv12);

}

bool ImuInitializer::EstimateExtrinsicRotation(CircularBuffer<PairTimeLaserTransform> &all_laser_transforms,
                                               Transform &transform_lb) {

  Transform transform_bl = transform_lb.inverse();
  Eigen::Quaterniond rot_bl = transform_bl.rot.template cast<double>();

  size_t window_size = all_laser_transforms.size() - 1;

  Eigen::MatrixXd A(window_size * 4, 4);

  for (size_t i = 0; i < window_size; ++i) {
    PairTimeLaserTransform &laser_trans_i = all_laser_transforms[i];
    PairTimeLaserTransform &laser_trans_j = all_laser_transforms[i + 1];

    Eigen::Quaterniond delta_qij_imu = laser_trans_j.second.pre_integration->delta_q_;

    Eigen::Quaterniond delta_qij_laser
        = (laser_trans_i.second.transform.rot.conjugate() * laser_trans_j.second.transform.rot).template cast<double>();

    Eigen::Quaterniond delta_qij_laser_from_imu = rot_bl.conjugate() * delta_qij_imu * rot_bl;

    double angular_distance = 180 / M_PI * delta_qij_laser.angularDistance(delta_qij_laser_from_imu);

//    DLOG(INFO) << i << ", " << angular_distance;

    double huber = angular_distance > 5.0 ? 5.0 / angular_distance : 1.0;

    Eigen::Matrix4d lq_mat = LeftQuatMatrix(delta_qij_laser);
    Eigen::Matrix4d rq_mat = RightQuatMatrix(delta_qij_imu);

    A.block<4, 4>(i * 4, 0) = huber * (lq_mat - rq_mat);

//    cout << (lq_mat * transform_lb.rot.coeffs().template cast<double>()).transpose() << endl;
//
//    cout << (delta_qij_laser * transform_lb.rot.template cast<double>()).coeffs().transpose() << endl;
//
//    cout << (rq_mat * transform_lb.rot.coeffs().template cast<double>()).transpose() << endl;
//
//    cout << (transform_lb.rot.template cast<double>() * delta_qij_imu).coeffs().transpose() << endl;


  }

//  DLOG(INFO) << ">>>>>>> A <<<<<<<" << endl << A;

  Eigen::JacobiSVD<MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix<double, 4, 1> x = svd.matrixV().col(3);
  Quaterniond estimated_qlb(x);

  transform_lb.rot = estimated_qlb.cast<float>().toRotationMatrix();

  Vector3d cov = svd.singularValues().tail<3>();

//  DLOG(INFO) << "x: " << x.transpose();
  DLOG(INFO) << "extrinsic rotation: " << transform_lb.rot.coeffs().transpose();
//  cout << x.transpose << endl;
//  DLOG(INFO) << "singular values: " << svd.singularValues().transpose();
//  cout << cov << endl;

  // NOTE: covariance 0.25
  if (cov(1) > 0.25) {
    return true;
  } else {
    return false;
  }

}

bool ImuInitializer::Initialization(CircularBuffer<PairTimeLaserTransform> &all_laser_transforms,
                                    CircularBuffer<Vector3d> &Vs,
                                    CircularBuffer<Vector3d> &Bas,
                                    CircularBuffer<Vector3d> &Bgs,
                                    Vector3d &g,
                                    Transform &transform_lb,
                                    Matrix3d &R_WI) {
//  TicToc tic_toc;
//  tic_toc.Tic();

  EstimateGyroBias(all_laser_transforms, Bgs);

//  DLOG(INFO) << "EstimateGyroBias time: " << tic_toc.Toc() << " ms";
//  tic_toc.Tic();

  if (!ApproximateGravity(all_laser_transforms, g, transform_lb)) {
    return false;
  };

//  DLOG(INFO) << "ApproximateGravity time: " << tic_toc.Toc() << " ms";
//  tic_toc.Toc();

  RefineGravityAccBias(all_laser_transforms, Bas, Bgs, g, transform_lb, R_WI);

//  DLOG(INFO) << "RefineGravityAccBias time: " << tic_toc.Toc() << " ms";
//  tic_toc.Tic();

  UpdateVelocities(all_laser_transforms, Vs, Bas, Bgs, g, transform_lb);

//  DLOG(INFO) << "UpdateVecocities time: " << tic_toc.Toc() << " ms";

  return true;

}

}
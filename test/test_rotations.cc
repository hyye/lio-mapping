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
// Created by hyye on 3/15/18.
//

#include <gtest/gtest.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_datatypes.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <Eigen/Eigen>
#include <geometry_msgs/Quaternion.h>
#include "3rdparty/sophus/so3.hpp"
#include "3rdparty/sophus/se3.hpp"

#include "utils/TicToc.h"
#include "utils/Twist.h"
#include "utils/math_utils.h"
#include "utils/geometry_utils.h"

using namespace std;

using geometryutils::RightJacobianInverse;

tf::Matrix3x3 RotX(tfScalar th) {
  tf::Matrix3x3 m;
  m.setValue(1, 0, 0,
             0, cos(th), -sin(th),
             0, sin(th), cos(th));
  return m;
}

tf::Matrix3x3 RotY(tfScalar th) {
  tf::Matrix3x3 m;
  m.setValue(cos(th), 0, sin(th),
             0, 1, 0,
             -sin(th), 0, cos(th));
  return m;
}

tf::Matrix3x3 RotZ(tfScalar th) {
  tf::Matrix3x3 m;
  m.setValue(cos(th), -sin(th), 0,
             sin(th), cos(th), 0,
             0, 0, 1);
  return m;
}

namespace Eigen {

using std::sin;
using std::cos;

Eigen::Matrix3d RotX(double th) {
  Eigen::Matrix3d m;
  m << 1, 0, 0,
      0, cos(th), -sin(th),
      0, sin(th), cos(th);
  return m;
}

Eigen::Matrix3d RotY(double th) {
  Eigen::Matrix3d m;
  m << cos(th), 0, sin(th),
      0, 1, 0,
      -sin(th), 0, cos(th);
  return m;
}

Eigen::Matrix3d RotZ(double th) {
  Eigen::Matrix3d m;
  m << cos(th), -sin(th), 0,
      sin(th), cos(th), 0,
      0, 0, 1;
  return m;
}

Eigen::Matrix3d RotZXY(double x, double y, double z) {
  Eigen::Matrix3d m;
  m << cos(y) * cos(z) - sin(x) * sin(y) * sin(z), -cos(x) * sin(z), cos(z) * sin(y) + cos(y) * sin(x) * sin(z),
      cos(y) * sin(z) + cos(z) * sin(x) * sin(y), cos(x) * cos(z), sin(y) * sin(z) - cos(y) * cos(z) * sin(x),
      -cos(x) * sin(y), sin(x), cos(x) * cos(y);
  return m;
}

}

Eigen::Matrix3d RightJacobian(Eigen::Vector3d v) {

  double v_norm = v.norm();
  double v_norm2 = v_norm * v_norm;
  double v_norm3 = v_norm2 * v_norm;
  auto v_skew = Sophus::SO3d::hat(v);

  Eigen::Matrix3d I3x3 = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Jr =
      I3x3
          - ((1 - cos(v_norm)) / v_norm2) * v_skew
          + (v_norm - sin(v_norm)) / v_norm3 * v_skew * v_skew;

  return Jr;
}

Eigen::Matrix3d RightJacobian(Sophus::SO3d R) {
  Eigen::Matrix3d Jr = RightJacobian(R.log());
  return Jr;
}

Eigen::Matrix3d RotationVectorJacobian(Sophus::SO3d R, Eigen::Vector3d a) {

  Eigen::Matrix3d Jr = -R.matrix() * Sophus::SO3d::hat(a) * RightJacobian(R);

  return Jr;
}

Eigen::Matrix3d RotationTransposeVectorJacobian(const Sophus::SO3d &R, const Eigen::Vector3d &a) {

  Sophus::SO3d::Tangent v = R.log();

  Eigen::Matrix3d Jr = Sophus::SO3d::hat(R.unit_quaternion().conjugate() * a) * RightJacobian(v);

  return Jr;
}

Eigen::Matrix3d RotationTransposeVectorJacobianPrev(const Sophus::SO3d &R, const Eigen::Vector3d &a) {

  Sophus::SO3d::Tangent v = R.log();

  Eigen::Matrix3d Jr = Sophus::SO3d::exp(-v).matrix() * Sophus::SO3d::hat(a) * RightJacobian(-v);

  return Jr;
}

template<typename T>
Eigen::Matrix<T, 3, 3> RightJacobianInverseApproximate(const Eigen::Matrix<T, 3, 1> &v) {

  T v_norm = v.norm();
  T v_norm2 = v_norm * v_norm;
  T v_norm3 = v_norm2 * v_norm;
  Eigen::Matrix<T, 3, 3> v_skew = Sophus::SO3<T>::hat(v);
  Eigen::Matrix<T, 3, 3> I3x3 = Eigen::Matrix<T, 3, 3>::Identity();

  if (v_norm < Sophus::Constants<T>::epsilon()) {
    cout << "C: " << 1 << endl;
    return I3x3;
  }

  double C = (1 / v_norm2 - (1 + cos(v_norm)) / (2 * v_norm * sin(v_norm)));

  cout << "C: " << C << endl;
//  cout << "0.5 * v_skew:" << endl << 0.5 * v_skew << endl;
//  cout << "C * v_skew * v_skew:" << endl << C * v_skew * v_skew << endl;

  Eigen::Matrix<T, 3, 3> Jr_inv = I3x3 + 0.5 * v_skew;
  return Jr_inv;
}

template<typename T>
Eigen::Matrix<T, 3, 3> RightJacobianInverseDebug(const Eigen::Matrix<T, 3, 1> &v) {

  T v_norm = v.norm();
  T v_norm2 = v_norm * v_norm;
  T v_norm3 = v_norm2 * v_norm;
  Eigen::Matrix<T, 3, 3> v_skew = Sophus::SO3<T>::hat(v);
  Eigen::Matrix<T, 3, 3> I3x3 = Eigen::Matrix<T, 3, 3>::Identity();

  if (v_norm < Sophus::Constants<T>::epsilon()) {
    cout << "C_t: " << 1 << endl;
    return I3x3;
  }

  double C_t = (1 / v_norm2 - (1 + cos(v_norm)) / (2 * v_norm * sin(v_norm)));

  cout << "C_t: " << C_t << endl;
//  cout << "0.5 * v_skew:" << endl << 0.5 * v_skew << endl;
//  cout << "C_t * v_skew * v_skew:" << endl << C_t * v_skew * v_skew << endl;

  Eigen::Matrix<T, 3, 3> Jr_inv
      = I3x3 + 0.5 * v_skew + (1 / v_norm2 - (1 + cos(v_norm)) / (2 * v_norm * sin(v_norm))) * v_skew * v_skew;
  return Jr_inv;
}

TEST(RotationTest, Rotation2JacobianTest) {

  typedef Sophus::SO3d SO3;

  Eigen::Matrix3d J_dr_r1;
  Eigen::Quaterniond q1(1, 0.2, 0.7, 0.8), q2(-0.5, 0.4, 0, -0.5), dq12(0.3, 0.5, 0.3, -0.9);
  q1.normalize();
  q2.normalize();
  dq12.normalize();
  Eigen::Matrix3d R1(q1), R2(q2), Rd12(dq12); /// Rd12 is updated by Jacobian

  SO3 SO3_(Rd12.transpose() * R1.transpose() * R2);
  Eigen::Vector3d th_k = SO3_.log();

  cout << RightJacobianInverse(th_k) * RightJacobian(th_k) << endl;

  Eigen::Vector3d turb(-0.05, -0.02, 0.03);

  Eigen::Vector3d r2_v = SO3(R2).log();

  Eigen::Matrix3d m_J = RightJacobianInverse(th_k) * RightJacobian(r2_v);

  Eigen::Vector3d gt_out
      = SO3(Rd12.transpose() * R1.transpose() * SO3::exp(r2_v + turb).unit_quaternion().toRotationMatrix()).log();

  Eigen::Vector3d out = th_k + m_J * turb;

  cout << "ground truth turb: " << gt_out.transpose() << endl;
  cout << "th_k + J * turb: " << (th_k + m_J * turb).transpose() << endl;
  cout << "th_k + J * turb: " << (-out / out.norm() * (2 * M_PI - out.norm())).transpose() << endl;
  cout << "diff: " << SO3::exp(gt_out).unit_quaternion().angularDistance(SO3::exp(th_k + m_J * turb).unit_quaternion())
       << endl;
  cout << "th_k: " << (-th_k / th_k.norm() * (2 * M_PI - th_k.norm())).transpose() << endl;

}

TEST(RotationTest, Rotation1JacobianTest) {

  typedef Sophus::SO3d SO3;

  Eigen::Matrix3d J_dr_r1;
  Eigen::Quaterniond q1(1, 0.2, 0.7, 0.8), q2(-0.5, 0.4, 0, -0.5), dq12(0.3, 0.5, 0.3, -0.9);
  q1.normalize();
  q2.normalize();
  dq12.normalize();
  Eigen::Matrix3d R1(q1), R2(q2), Rd12(dq12); /// Rd12 is updated by Jacobian

  SO3 SO3_(Rd12.transpose() * R1.transpose() * R2);
  Eigen::Vector3d th_k = SO3_.log();

  cout << RightJacobianInverse(th_k) * RightJacobian(th_k) << endl;

  Eigen::Vector3d turb(-0.1, -0.2, 0.3);

  Eigen::Vector3d r1_v = SO3(R1).log();

  Eigen::Matrix3d m_J = -RightJacobianInverse(th_k) * R2.transpose() * R1 * RightJacobian(r1_v);

  Eigen::Vector3d gt_out
      = SO3((Rd12.transpose() * (SO3::exp(r1_v + turb).unit_quaternion()).toRotationMatrix().transpose() * R2)).log();

  cout << "ground truth turb: " << gt_out.transpose() << endl;
  cout << "th_k + J * turb: " << (th_k + m_J * turb).transpose() << endl;
  cout << "diff: " << SO3::exp(gt_out).unit_quaternion().angularDistance(SO3::exp(th_k + m_J * turb).unit_quaternion())
       << endl;
  cout << "th_k: " << (th_k).transpose() << endl;

}

TEST(RotationTest, BiasJacobianTest) {

  typedef Sophus::SO3d SO3;

  Eigen::Matrix3d J_dr_db;
  J_dr_db << 0.02, -0.001, 0.001,
      -0.001, 0.02, 0.001,
      0.001, 0.001, 0.02;
  Eigen::Quaterniond q1(1, 0.2, 0.7, 0.8), q2(-0.5, 0.4, 1, -0.5), dq12(0.3, 0.2, 0.3, -0.9);
  q1.normalize();
  q2.normalize();
  dq12.normalize();
  Eigen::Matrix3d R1(q1), R2(q2), Rd12(dq12);

  SO3 SO3_(R2.transpose() * R1 * Rd12);
  Eigen::Vector3d th = SO3_.log();

  Eigen::Matrix3d m_J = -RightJacobianInverse(th) * J_dr_db;

  Eigen::Vector3d turb(0.1, -0.2, 0.3);

  Eigen::Vector3d gt_out
      = SO3(((Rd12 * SO3::exp(J_dr_db * turb).unit_quaternion()).transpose() * R1.transpose() * R2)).log();

  cout << "ground truth turb: " << gt_out.transpose() << endl;
  cout << "-th + J * turb: " << (-th + m_J * turb).transpose() << endl;
  cout << "-th: " << (-th).transpose() << endl;

}

TEST(RotationTest, JacobianInverseTest) {

  using namespace Sophus;

  Eigen::Vector3d w(1, -2, 2);
  double dt = 0.5;

  SO3d SO3_;
  Eigen::Quaterniond qt(1, -2, 2, 4);
  qt.normalize();

  SO3_.setQuaternion(qt);

  Eigen::Vector3d th = SO3_.log();

  Eigen::Vector3d dth_dt = RightJacobianInverse(SO3_) * w;

  cout << dth_dt.transpose() << endl;

  Eigen::Quaterniond q_truth = (SO3d::exp(th) * SO3d::exp(w * dt)).unit_quaternion();
  Eigen::Quaterniond q_from_j = SO3d::exp(th + dth_dt * dt).unit_quaternion();

  cout << "truth: " << q_truth.coeffs().transpose() << endl;
  cout << "with Jacobian: " << q_from_j.coeffs().transpose() << endl;
  cout << "diff Jacobian: " << q_truth.angularDistance(q_from_j) << endl;

  Eigen::Matrix3d w_skew = Sophus::SO3d::hat(w);
  Eigen::Matrix4d Omega;
  Omega.block<3, 3>(0, 0) = -w_skew;
  Omega.block<1, 3>(3, 0) = -w.transpose();
  Omega.block<3, 1>(0, 3) = w;
  Omega(3, 3) = 0;

  Eigen::Matrix4d Fd = 0.5 * Omega * dt;
  Eigen::Matrix4d I4x4;
  Eigen::Vector4d q_vec = qt.coeffs();
  q_vec = (I4x4 + Fd + 0.5 * Fd * Fd) * q_vec;
  Eigen::Quaterniond q_from_q(q_vec.w(), q_vec.x(), q_vec.y(), q_vec.z());

  q_from_q.normalize();

  cout << "with Quaternion: " << q_from_q.coeffs().transpose() << endl;
  cout << "diff Quaternion: " << q_truth.angularDistance(q_from_q) << endl;

  Eigen::Vector3d r_vec = Eigen::Vector3d::Random();

  cout << "SO3d::exp(r_vec): " << SO3d::exp(r_vec).unit_quaternion().coeffs().transpose() << endl;

  auto r_vec_2pi = (r_vec / r_vec.norm()) * (r_vec.norm() + M_PI * 2);

  cout << "SO3d::exp(r_vec + 2pi): "
       << SO3d::exp(r_vec_2pi).unit_quaternion().coeffs().transpose() << endl;

  for (int i = 0; i < 10; ++i) {
    Eigen::Vector3d tangent = Eigen::Vector3d::Random();
    Eigen::Vector3d d_tangent(0.1, 0.1, 0.1);
    tangent *= (i + 1) * 0.1;

    tangent = (tangent / tangent.norm()) * (mathutils::NormalizeRad(tangent.norm()));

    cout << "tangent: " << tangent.transpose() << endl;
    auto a = RightJacobianInverseApproximate(tangent);
    tangent += d_tangent;
    cout << "tangent: " << tangent.transpose() << endl;
    auto b = RightJacobianInverseDebug(tangent);
    cout << "RightJacobianInverseApproximate:" << endl << a << endl;
    cout << "RightJacobianInverseDebug:" << endl << b << endl;
    cout << "norm: " << (a - b).norm() << endl;

    cout << "=======" << endl;

  }

}

TEST(RotationTest, IntegrationTest) {
//  Eigen::Quaterniond q(0.4, -0.4, 1, 2);
  Eigen::Quaterniond q(10, -0.4, 1, 2); // NOTE: dq should be small to make the tangent close?
  q.normalize();

  Eigen::Quaterniond q_ig = q.conjugate();

  Eigen::Vector4d q_vec, q_ig_vec;

  q_vec = q.coeffs();
  q_ig_vec = q_ig.coeffs();

  Eigen::Vector3d omega(0.5, -0.3, 0.3), tangent;
  Eigen::Matrix3d omega_skew = Sophus::SO3d::hat(omega), minus_ogema_skew_dt;
  Eigen::Matrix4d Omega, Omega_ig;
  Omega.block<3, 3>(0, 0) = -omega_skew;
  Omega.block<1, 3>(3, 0) = -omega.transpose();
  Omega.block<3, 1>(0, 3) = omega;
  Omega(3, 3) = 0;

  Eigen::Vector4d omega_ig;
  omega_ig.head<3>() = -omega;
  omega_ig.w() = 0;
  Omega_ig = mathutils::LeftQuatMatrix(omega_ig);

  cout << Omega << endl;
  cout << Omega_ig << endl;

  double dt = 0.01;

  Sophus::SO3d R_SO3(q);

  tangent = R_SO3.log();

  minus_ogema_skew_dt = -omega_skew * dt;

  Eigen::Matrix3d I3x3;
  Eigen::Matrix4d I4x4;
  I3x3.setIdentity();
  I4x4.setIdentity();

  Eigen::Matrix4d Fd = 0.5 * Omega * dt;
  Eigen::Matrix4d Fd_ig = 0.5 * Omega_ig * dt;

  for (int i = 0; i < 20; ++i) {

    q_vec = (I4x4 + Fd + 0.5 * Fd * Fd) * q_vec;

    q_ig_vec = (I4x4 + Fd_ig + 0.5 * Fd_ig * Fd_ig) * q_ig_vec;

    R_SO3 = R_SO3 * Sophus::SO3d::exp(omega * dt);

    tangent = (I3x3 + minus_ogema_skew_dt + 0.5 * minus_ogema_skew_dt * minus_ogema_skew_dt) * tangent;
//    tangent = (I3x3 - 1.0 / omega.norm() * sin(omega.norm() * dt) * omega_skew
//        + 1.0 / (omega.norm() * omega.norm()) * (1 - cos(omega.norm() * dt)) * omega_skew * omega_skew) * tangent;

  }

  Sophus::SO3d deltaq_r = Sophus::SO3d::exp(omega * dt);
  cout << deltaq_r.unit_quaternion().coeffs() << endl;
  cout << mathutils::RightQuatMatrix(deltaq_r.unit_quaternion()) << endl;
  cout << (I4x4 + Fd + 0.5 * Fd * Fd) << endl;


  cout << q_vec.transpose() << endl;
  cout << q_ig_vec.transpose() << endl;
  cout << R_SO3.unit_quaternion().coeffs().transpose() << endl;
  cout << Sophus::SO3d::exp(tangent).unit_quaternion().coeffs().transpose() << endl;

//  cout << Omega << endl;

}

TEST(RotationTest, TwistTest) {
  lio::Twist<float> t1, t2;
  t1.rot = Eigen::Quaternionf(0.4, -0.4, 1, 2).normalized();
  t1.pos = Eigen::Vector3f(-20, 10, 10);
  t2.rot = Eigen::Quaternionf(0.3, 0.5, -0.8, 2).normalized();
  t2.pos = Eigen::Vector3f(20, 20, -10);

  cout << t1.transform().matrix() << endl;
  cout << t2.transform().matrix() << endl;
  cout << (t1 * t2).transform().matrix() << endl;
  cout << t1.transform().matrix() * t2.transform().matrix() << endl;

  t2 = t1 * t2;

  cout << t2.transform().matrix() << endl;

  lio::Twist<float> t3(t2);

  cout << t2 << endl;
  cout << t3 << endl;

}

TEST(RotationTest, DiffTest) {

  lio::Twist<float> t1, t2;

  double x = 0.5, y = -0.1, z = 0.5;
  double delta = 0.2;
  Eigen::Matrix3d rotm = Eigen::RotZXY(x, y, z);
  Eigen::Quaterniond q(rotm);

  Sophus::SO3d R_SO3(rotm);

  Sophus::SO3d::Tangent tangent = R_SO3.log();

  tangent = tangent + Eigen::Vector3d(1, 1, 1) * delta;
  Eigen::Quaterniond q_new(Sophus::SO3d::exp(tangent).unit_quaternion());

  cout << "euler: " << Eigen::Quaterniond(Eigen::RotZXY(x + delta, y + delta, z + delta)).angularDistance(q) << endl;
  cout << "manifold: " << q_new.angularDistance(q) << endl;

  auto q_diff = q_new.conjugate() * q;
  cout << "q->q_new: " << atan2(q_diff.vec().norm(), q_diff.w()) * 2 << endl;
  cout << "q->q_new: " << (q_diff.vec() / q_diff.vec().norm()).transpose() << endl;

  cout << "manifold: " << q.angularDistance(q_new) << endl;
  q_diff = q.conjugate() * q_new;
  cout << "q_new->q: " << atan2(q_diff.vec().norm(), q_diff.w()) * 2 << endl;
  cout << "q->q_new: " << (q_diff.vec() / q_diff.vec().norm()).transpose() << endl;

}

TEST(RotationTest, QuaternionTest) {
  lio::Twist<float> t1, t2;
  t1.rot = Eigen::Quaternionf(0.3827, 0, 0, 0.9239).normalized();
  t1.pos = Eigen::Vector3f(-20, 10, 10);
  t2.rot = Eigen::Quaternionf(0.3, 0.5, -0.8, 2).normalized();
  t2.pos = Eigen::Vector3f(20, 20, -10);

  Eigen::Quaternionf qI;
  qI.setIdentity();

  float s = 0.25;
  cout << t2.rot.slerp(s, qI).coeffs().transpose() << endl;
  cout << qI.slerp(1 - s, t2.rot).coeffs().transpose() << endl;

  s = 0.5;
  cout << t2.rot.slerp(s, qI).coeffs().transpose() << endl;
  cout << qI.slerp(1 - s, t2.rot).coeffs().transpose() << endl;

  s = 0.75;
  cout << t2.rot.slerp(s, qI).coeffs().transpose() << endl;
  cout << qI.slerp(1 - s, t2.rot).coeffs().transpose() << endl;

}

TEST(RotationTest, SophusTest) {
  Sophus::SO3d SO3_;
  Eigen::Quaterniond qt(1, -2, 2, 4);
  qt.normalize();

  SO3_.setQuaternion(qt);
  Eigen::MatrixXd out = SO3_.Dx_exp_x_at_0();
  cout << SO3_.matrix() << endl;
  cout << SO3_.matrix() * SO3_.matrix().transpose() << endl;

}

TEST(RotationTest, JacobianTest) {
  using namespace Sophus;

  SO3d SO3_;
  Eigen::Quaterniond qt(1, -2, 2, 4);
  qt.normalize();

  SO3_.setQuaternion(qt);

  Eigen::Vector3d u = qt.vec() / qt.vec().norm();
  double s = 2 * atan2(qt.vec().norm(), qt.w());
  Eigen::Vector3d th = s * u;

  cout << "th " << th.transpose() << endl;
  SO3d tmp = SO3d::exp(th);

  auto tt = SO3_.log();
  double tt_norm = tt.norm();
  double tt_norm2 = tt_norm * tt_norm;
  double tt_norm3 = tt_norm2 * tt_norm;
  auto hat_tt = SO3d::hat(tt);

  Eigen::Matrix3d I3x3 = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d Jr = RightJacobian(tt);

  auto JrS = SO3_.Dx_exp_x(tt); // NOTE: quaternion w.r.t. x
//  auto JrS = SO3_.Dx_this_mul_exp_x_at_0();

  cout << "Jr" << endl << Jr << endl;
  cout << "JrS" << endl << JrS << endl;

  SO3d::Tangent dtt(0.1, -0.5, 0.7);
  auto tt_true = th + dtt;
  auto out_with_exp_Jr = SO3_ * SO3d::exp(Jr * dtt);

  Eigen::Vector4d dqtt = dtt.transpose() * JrS; /// 0123 vec+real
  Eigen::Quaterniond out_with_Jq(qt.w() + dqtt(3), qt.x() + dqtt(0), qt.y() + dqtt(1), qt.z() + dqtt(2));
  // qout3.normalize();

//  cout << "test" << endl << SO3d::exp(Jr * dtt).unit_quaternion().coeffs().transpose() << endl;
//  cout << "out_with_exp_Jr" << endl << out_with_exp_Jr.matrix() << endl;
  cout << "out_with_exp_Jr " << endl << out_with_exp_Jr.unit_quaternion().coeffs().transpose() << endl;
  cout << "out_with_Jq " << endl << out_with_Jq.coeffs().transpose() << endl;
  cout << "out_ground_truth " << endl << SO3d::exp(tt_true).unit_quaternion().coeffs().transpose() << endl;

  cout << "diff with Jr " << SO3d::exp(tt_true).unit_quaternion().angularDistance(out_with_exp_Jr.unit_quaternion())
       << endl;

  cout << "diff with q Jr " << SO3d::exp(tt_true).unit_quaternion().angularDistance(out_with_Jq) << endl;

//  cout << "SO3_.log " << SO3_.log().transpose() << endl;
//  cout << "tmp matrix" << endl << tmp.matrix() << endl;
//  cout << "tmp.log " << tmp.log().transpose() << endl;
//  cout << "qt " << qt.coeffs().transpose() << endl;
//  cout << "tmp.q " << Eigen::Quaterniond(tmp.matrix()).coeffs().transpose() << endl;
//  d_Ra_d_th = ;
}

TEST(RotationTest, RotationVectorTest) {
  using namespace Sophus;

  SO3d SO3_;
  Eigen::Quaterniond qt(1, -2, 2, 4);
  qt.normalize();

  SO3_.setQuaternion(qt);

  Eigen::Vector3d a_(50, 20, -10);
  Eigen::Vector3d Ra = SO3_ * a_;
  Eigen::Matrix3d JRa = RotationVectorJacobian(SO3_, a_);

  SO3d::Tangent dtt(0.1, -0.08, 0.07);

  auto outtt = SO3_.log() + dtt;
  auto R_true = SO3d::exp(outtt);
  cout << "R_true * a_ " << (R_true * a_).transpose() << endl;
  cout << "rotation Jacobian " << (Ra + JRa * dtt).transpose() << endl;
  cout << "diff " << (Ra + JRa * dtt - (R_true * a_)).norm() << endl;

  Eigen::Matrix3d JRTa_prev = RotationTransposeVectorJacobianPrev(SO3_, a_);
  Eigen::Matrix3d JRTa = RotationTransposeVectorJacobian(SO3_, a_);
  Eigen::Vector3d RTa = SO3_.matrix().transpose() * a_;

  auto RT_true = R_true.matrix().transpose();
  cout << "RT_true * a_ " << (RT_true * a_).transpose() << endl;
  cout << "rotation T Jacobian_prev " << (RTa + JRTa_prev * dtt).transpose() << endl;
  cout << "diff " << (RTa + JRTa_prev * dtt - (RT_true * a_)).norm() << endl;

  cout << "rotation T Jacobian " << (RTa + JRTa * dtt).transpose() << endl;
  cout << "diff " << (RTa + JRTa * dtt - (RT_true * a_)).norm() << endl;

  cout << SO3d(SO3_.matrix()).log().transpose() << endl;
  cout << SO3d(SO3_.matrix().transpose()).log().transpose() << endl;
}

TEST(RotationTest, ROSEulerTest) {

  tf::Quaternion q(1, 2, 3, 4);
  q.normalize();
  tf::Matrix3x3 m(q);
  tfScalar r, p, y;
//  m.getEulerYPR(y, p, r);
  m.getRPY(r, p, y);
  cout << "yaw: " << y << " pitch: " << p << " roll: " << r << endl;

  tf::Matrix3x3 m_o = RotZ(y) * RotY(p) * RotX(r);
  // m_o = RotX(r) * RotY(p) * RotZ(y);

  m_o.getRPY(r, p, y);

  cout << "yaw: " << y << " pitch: " << p << " roll: " << r << endl;

  tf::Quaternion q2;
  q2.setRPY(r, p, y);

  m_o.setRotation(q2);

  cout << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << endl;
  cout << q2.w() << ", " << q2.x() << ", " << q2.y() << ", " << q2.z() << endl;

  tfScalar roll, pitch, yaw, rot_x, rot_y, rot_z, rot_xo, rot_yo, rot_zo;
  rot_xo = 1.2;
  rot_yo = -0.9;
  rot_zo = 0.5;
  geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(rot_zo,
                                                                              -rot_xo,
                                                                              -rot_yo);
  geometry_msgs::Quaternion geoQuat_recv;

  geometry_msgs::Pose pose1;

  pose1.orientation.x = -geoQuat.y;
  pose1.orientation.y = -geoQuat.z;
  pose1.orientation.z = geoQuat.x;
  pose1.orientation.w = geoQuat.w;

  geoQuat_recv = pose1.orientation;

  pose1.position.x = 0.5;
  pose1.position.y = -0.5;
  pose1.position.z = 1.5;

  tf::Quaternion q_tmp;
  q_tmp = tf::Quaternion(geoQuat_recv.z, -geoQuat_recv.x, -geoQuat_recv.y, geoQuat_recv.w);
  tf::Matrix3x3(tf::Quaternion(geoQuat_recv.z, -geoQuat_recv.x, -geoQuat_recv.y, geoQuat_recv.w)).getRPY(roll,
                                                                                                         pitch,
                                                                                                         yaw);

  rot_x = -pitch;
  rot_y = -yaw;
  rot_z = roll;

  cout << rot_x << ", " << rot_y << ", " << rot_z << endl;
  cout << rot_xo << ", " << rot_yo << ", " << rot_zo << endl;

  EXPECT_DOUBLE_EQ(rot_x, rot_xo);
  EXPECT_DOUBLE_EQ(rot_y, rot_yo);
  EXPECT_DOUBLE_EQ(rot_z, rot_zo);

}

TEST(RotationTest, EigenTest) {
  Eigen::Matrix<double, 3, 6> rand = Eigen::MatrixXd::Random(3, 6);
  Eigen::MatrixXd A(rand.rows() * 4, 6);
  A << rand,
      rand,
      rand,
      rand;

  Eigen::Matrix<double, 1, 6> matE;
  Eigen::Matrix<double, 6, 6> matV, matV2;
  Eigen::MatrixXd AtA = A.transpose() * A;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6> > esolver(AtA);
  matE = esolver.eigenvalues().real();
  matV = esolver.eigenvectors().real();

  cout << AtA << endl;

  cout << matE << endl;
  cout << matV << endl;

  cout << "=======" << endl;

  cout << matE(5) << endl;

  cout << matV.block(0, 5, 6, 1) << endl;

  cout << "=======" << endl;

  cv::Mat cvmatAtA(6, 6, CV_32F, cv::Scalar::all(0));
  cv::Mat cvmatE(1, 6, CV_32F, cv::Scalar::all(0));
  cv::Mat cvmatV(6, 6, CV_32F, cv::Scalar::all(0));
  cv::eigen2cv(AtA, cvmatAtA);

  cv::eigen(cvmatAtA, cvmatE, cvmatV);

  cout << cvmatAtA.at<double>(0, 3) << endl;
  cout << AtA(0, 3) << endl;

  cout << cvmatE << endl;
  cout << cvmatV << endl;

  matV2 = matV;

  float eignThre[6] = {10, 10, 10, 10, 10, 10};
  for (int i = 5; i >= 0; i--) {
    if (matE(0, i) < eignThre[i]) {
      for (int j = 0; j < 6; j++) {
        matV2(i, j) = 0;
      }
      std::cout << matE << std::endl;
    } else {
      break;
    }
  }

  cout << matV2 << endl;

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_alsologtostderr = true;

  return RUN_ALL_TESTS();
}
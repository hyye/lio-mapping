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
// Created by hyye on 3/27/18.
//

#ifndef LIO_INTEGRATIONBASE_H_
#define LIO_INTEGRATIONBASE_H_

#include <Eigen/Eigen>

#include "utils/math_utils.h"

namespace lio {

using namespace mathutils;
using Eigen::Vector3d;
using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Quaterniond;


/// Adapted from VINS-mono

enum SizeParameterization {
  SIZE_QUAT = 4,
  SIZE_POSE = 7,
  SIZE_SPEED_BIAS = 9,
};

enum StateOrder {
  O_P = 0,
  O_R = 3,
  O_V = 6,
  O_BA = 9,
  O_BG = 12,
};

struct IntegrationBaseConfig {
  double acc_n = 0.1;
  double gyr_n = 0.01;
  double acc_w = 0.0002;
  double gyr_w = 2.0e-5;
  double g_norm = 9.805;
};

class IntegrationBase {

 public:
  IntegrationBase() = delete;

  IntegrationBase(const Vector3d &acc0, const Vector3d &gyr0,
                  const Vector3d &linearized_ba, const Vector3d &linearized_bg,
                  const IntegrationBaseConfig config = IntegrationBaseConfig())
      : acc0_{acc0},
        gyr0_{gyr0},
        linearized_acc_{acc0},
        linearized_gyr_{gyr0},
        linearized_ba_{linearized_ba},
        linearized_bg_{linearized_bg},
        jacobian_{Matrix<double, 15, 15>::Identity()},
        covariance_{Matrix<double, 15, 15>::Zero()},
        sum_dt_{0.0},
        delta_p_{Vector3d::Zero()},
        delta_q_{Quaterniond::Identity()},
        delta_v_{Vector3d::Zero()} {
    config_ = config;
    g_vec_ = Vector3d(0, 0, -config_.g_norm);
    noise_ = Matrix<double, 18, 18>::Zero();
    noise_.block<3, 3>(0, 0) = (config_.acc_n * config_.acc_n) * Matrix3d::Identity();
    noise_.block<3, 3>(3, 3) = (config_.gyr_n * config_.gyr_n) * Matrix3d::Identity();
    noise_.block<3, 3>(6, 6) = (config_.acc_n * config_.acc_n) * Matrix3d::Identity();
    noise_.block<3, 3>(9, 9) = (config_.gyr_n * config_.gyr_n) * Matrix3d::Identity();
    noise_.block<3, 3>(12, 12) = (config_.acc_w * config_.acc_w) * Matrix3d::Identity();
    noise_.block<3, 3>(15, 15) = (config_.gyr_w * config_.gyr_w) * Matrix3d::Identity();
  }

  void push_back(double dt, const Vector3d &acc, const Vector3d &gyr) {
    dt_buf_.push_back(dt);
    acc_buf_.push_back(acc);
    gyr_buf_.push_back(gyr);
    Propagate(dt, acc, gyr);
  }

  void Repropagate(const Vector3d &linearized_ba, const Vector3d &linearized_bg) {
    // NOTE: Repropagate with measurements, not efficient
    sum_dt_ = 0.0;
    acc0_ = linearized_acc_;
    gyr0_ = linearized_gyr_;
    delta_p_.setZero();
    delta_q_.setIdentity();
    delta_v_.setZero();
    linearized_ba_ = linearized_ba;
    linearized_bg_ = linearized_bg;
    jacobian_.setIdentity();
    covariance_.setZero();
    for (size_t i = 0; i < dt_buf_.size(); ++i) {
      Propagate(dt_buf_[i], acc_buf_[i], gyr_buf_[i]);
    }
  }

  void MidPointIntegration(double dt,
                           const Vector3d &acc0, const Vector3d &gyr0,
                           const Vector3d &acc1, const Vector3d &gyr1,
                           const Vector3d &delta_p, const Quaterniond &delta_q,
                           const Vector3d &delta_v, const Vector3d &linearized_ba,
                           const Vector3d &linearized_bg, Vector3d &result_delta_p,
                           Quaterniond &result_delta_q, Vector3d &result_delta_v,
                           Vector3d &result_linearized_ba, Vector3d &result_linearized_bg,
                           bool update_jacobian) {
    //ROS_DEBUG("midpoint integration");

    // NOTE: the un_acc here is different from the un_acc in the Estimator
    Vector3d un_acc_0 = delta_q * (acc0 - linearized_ba);
    Vector3d un_gyr = 0.5 * (gyr0 + gyr1) - linearized_bg;
    result_delta_q = delta_q * Quaterniond(1, un_gyr(0) * dt / 2, un_gyr(1) * dt / 2, un_gyr(2) * dt / 2);
    Vector3d un_acc_1 = result_delta_q * (acc1 - linearized_ba);
    Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    result_delta_p = delta_p + delta_v * dt + 0.5 * un_acc * dt * dt;
    result_delta_v = delta_v + un_acc * dt;
    result_linearized_ba = linearized_ba;
    result_linearized_bg = linearized_bg;

    if (update_jacobian) {
      Vector3d w_x = 0.5 * (gyr0 + gyr1) - linearized_bg;
      Vector3d a_0_x = acc0 - linearized_ba;
      Vector3d a_1_x = acc1 - linearized_ba;
      Matrix3d R_w_x, R_a_0_x, R_a_1_x;

      R_w_x << 0, -w_x(2), w_x(1),
          w_x(2), 0, -w_x(0),
          -w_x(1), w_x(0), 0;
      R_a_0_x << 0, -a_0_x(2), a_0_x(1),
          a_0_x(2), 0, -a_0_x(0),
          -a_0_x(1), a_0_x(0), 0;
      R_a_1_x << 0, -a_1_x(2), a_1_x(1),
          a_1_x(2), 0, -a_1_x(0),
          -a_1_x(1), a_1_x(0), 0;

      // NOTE: F = Fd = \Phi = I + dF*dt
      MatrixXd F = MatrixXd::Zero(15, 15);
      F.block<3, 3>(0, 0) = Matrix3d::Identity();
      F.block<3, 3>(0, 3) = -0.25 * delta_q.toRotationMatrix() * R_a_0_x * dt * dt +
          -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * dt) * dt * dt;
      F.block<3, 3>(0, 6) = MatrixXd::Identity(3, 3) * dt;
      F.block<3, 3>(0, 9) = -0.25 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * dt * dt;
//      F.block<3, 3>(0, 12) = -0.25 * result_delta_q.toRotationMatrix() * R_a_1_x * dt * dt * -dt;
      F.block<3, 3>(0, 12) = -0.1667 * result_delta_q.toRotationMatrix() * R_a_1_x * dt * dt * -dt;
      F.block<3, 3>(3, 3) = Matrix3d::Identity() - R_w_x * dt;
      F.block<3, 3>(3, 12) = -1.0 * MatrixXd::Identity(3, 3) * dt;
      F.block<3, 3>(6, 3) = -0.5 * delta_q.toRotationMatrix() * R_a_0_x * dt +
          -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * (Matrix3d::Identity() - R_w_x * dt) * dt;
      F.block<3, 3>(6, 6) = Matrix3d::Identity();
      F.block<3, 3>(6, 9) = -0.5 * (delta_q.toRotationMatrix() + result_delta_q.toRotationMatrix()) * dt;
      F.block<3, 3>(6, 12) = -0.5 * result_delta_q.toRotationMatrix() * R_a_1_x * dt * -dt;
      F.block<3, 3>(9, 9) = Matrix3d::Identity();
      F.block<3, 3>(12, 12) = Matrix3d::Identity();
      //cout<<"A"<<endl<<A<<endl;

      // NOTE: V = Fd * G_c
      // FIXME: verify if it is right, the 0.25 part
      MatrixXd V = MatrixXd::Zero(15, 18);
//      V.block<3, 3>(0, 0) = 0.25 * delta_q.toRotationMatrix() * dt * dt;
      V.block<3, 3>(0, 0) = 0.5 * delta_q.toRotationMatrix() * dt * dt;
      V.block<3, 3>(0, 3) = 0.25 * -result_delta_q.toRotationMatrix() * R_a_1_x * dt * dt * 0.5 * dt;
//      V.block<3, 3>(0, 6) = 0.25 * result_delta_q.toRotationMatrix() * dt * dt;
      V.block<3, 3>(0, 6) = 0.5 * result_delta_q.toRotationMatrix() * dt * dt;
      V.block<3, 3>(0, 9) = V.block<3, 3>(0, 3);
      V.block<3, 3>(3, 3) = 0.5 * MatrixXd::Identity(3, 3) * dt;
      V.block<3, 3>(3, 9) = 0.5 * MatrixXd::Identity(3, 3) * dt;
      V.block<3, 3>(6, 0) = 0.5 * delta_q.toRotationMatrix() * dt;
      V.block<3, 3>(6, 3) = 0.5 * -result_delta_q.toRotationMatrix() * R_a_1_x * dt * 0.5 * dt;
      V.block<3, 3>(6, 6) = 0.5 * result_delta_q.toRotationMatrix() * dt;
      V.block<3, 3>(6, 9) = V.block<3, 3>(6, 3);
      V.block<3, 3>(9, 12) = MatrixXd::Identity(3, 3) * dt;
      V.block<3, 3>(12, 15) = MatrixXd::Identity(3, 3) * dt;

      //step_jacobian = F;
      //step_V = V;
      jacobian_ = F * jacobian_;
      covariance_ = F * covariance_ * F.transpose() + V * noise_ * V.transpose();
    }

  }

  void EulerIntegration(double dt,
                        const Eigen::Vector3d &acc_0, const Eigen::Vector3d &gyr_0,
                        const Eigen::Vector3d &acc_1, const Eigen::Vector3d &gyr_1,
                        const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q,
                        const Eigen::Vector3d &delta_v, const Eigen::Vector3d &linearized_ba,
                        const Eigen::Vector3d &linearized_bg, Eigen::Vector3d &result_delta_p,
                        Eigen::Quaterniond &result_delta_q, Eigen::Vector3d &result_delta_v,
                        Eigen::Vector3d &result_linearized_ba, Eigen::Vector3d &result_linearized_bg,
                        bool update_jacobian) {
    result_delta_p = delta_p + delta_v * dt + 0.5 * (delta_q * (acc_1 - linearized_ba)) * dt * dt;
    result_delta_v = delta_v + delta_q * (acc_1 - linearized_ba) * dt;
    Vector3d omg = gyr_1 - linearized_bg;
    omg = omg * dt / 2;
    Quaterniond dR(1, omg(0), omg(1), omg(2));
    result_delta_q = (delta_q * dR);
    result_linearized_ba = linearized_ba;
    result_linearized_bg = linearized_bg;

    if (update_jacobian) {
      Vector3d w_x = gyr_1 - linearized_bg;
      Vector3d a_x = acc_1 - linearized_ba;
      Matrix3d R_w_x, R_a_x;

      R_w_x << 0, -w_x(2), w_x(1),
          w_x(2), 0, -w_x(0),
          -w_x(1), w_x(0), 0;
      R_a_x << 0, -a_x(2), a_x(1),
          a_x(2), 0, -a_x(0),
          -a_x(1), a_x(0), 0;

      MatrixXd A = MatrixXd::Zero(15, 15);
      // one step euler 0.5
      A.block<3, 3>(0, 3) = 0.5 * (-1 * delta_q.toRotationMatrix()) * R_a_x * dt;
      A.block<3, 3>(0, 6) = MatrixXd::Identity(3, 3);
      A.block<3, 3>(0, 9) = 0.5 * (-1 * delta_q.toRotationMatrix()) * dt;
      A.block<3, 3>(3, 3) = -R_w_x;
      A.block<3, 3>(3, 12) = -1 * MatrixXd::Identity(3, 3);
      A.block<3, 3>(6, 3) = (-1 * delta_q.toRotationMatrix()) * R_a_x;
      A.block<3, 3>(6, 9) = (-1 * delta_q.toRotationMatrix());
      //cout<<"A"<<endl<<A<<endl;

      MatrixXd U = MatrixXd::Zero(15, 12);
      U.block<3, 3>(0, 0) = 0.5 * delta_q.toRotationMatrix() * dt;
      U.block<3, 3>(3, 3) = MatrixXd::Identity(3, 3);
      U.block<3, 3>(6, 0) = delta_q.toRotationMatrix();
      U.block<3, 3>(9, 6) = MatrixXd::Identity(3, 3);
      U.block<3, 3>(12, 9) = MatrixXd::Identity(3, 3);

      // put outside
      Eigen::Matrix<double, 12, 12> noise = Eigen::Matrix<double, 12, 12>::Zero();
      noise.block<3, 3>(0, 0) = (config_.acc_n * config_.acc_n) * Eigen::Matrix3d::Identity();
      noise.block<3, 3>(3, 3) = (config_.gyr_n * config_.gyr_n) * Eigen::Matrix3d::Identity();
      noise.block<3, 3>(6, 6) = (config_.acc_w * config_.acc_w) * Eigen::Matrix3d::Identity();
      noise.block<3, 3>(9, 9) = (config_.gyr_w * config_.gyr_w) * Eigen::Matrix3d::Identity();

      //write F directly
      MatrixXd F, V;
      F = (MatrixXd::Identity(15, 15) + dt * A);
      V = dt * U;
//      step_jacobian = F;
//      step_V = V;
      jacobian_ = F * jacobian_;
      covariance_ = F * covariance_ * F.transpose() + V * noise * V.transpose();
    }

  }

  void Propagate(double dt, const Vector3d &acc1, const Vector3d &gyr1) {
    dt_ = dt;
    acc1_ = acc1;
    gyr1_ = gyr1;
    Vector3d result_delta_p;
    Quaterniond result_delta_q;
    Vector3d result_delta_v;
    Vector3d result_linearized_ba;
    Vector3d result_linearized_bg;

    MidPointIntegration(dt, acc0_, gyr0_, acc1, gyr1, delta_p_, delta_q_, delta_v_,
                        linearized_ba_, linearized_bg_,
                        result_delta_p, result_delta_q, result_delta_v,
                        result_linearized_ba, result_linearized_bg, true);
//    EulerIntegration(dt, acc0_, gyr0_, acc1, gyr1, delta_p_, delta_q_, delta_v_,
//                     linearized_ba_, linearized_bg_,
//                     result_delta_p, result_delta_q, result_delta_v,
//                     result_linearized_ba, result_linearized_bg, true);

    delta_p_ = result_delta_p;
    delta_q_ = result_delta_q;
    delta_v_ = result_delta_v;
    linearized_ba_ = result_linearized_ba;
    linearized_bg_ = result_linearized_bg;
    delta_q_.normalize();
    sum_dt_ += dt_;
    acc0_ = acc1_;
    gyr0_ = gyr1_;

  }

  Matrix<double, 15, 1> Evaluate(const Vector3d &Pi,
                                 const Quaterniond &Qi,
                                 const Vector3d &Vi,
                                 const Vector3d &Bai,
                                 const Vector3d &Bgi,
                                 const Vector3d &Pj,
                                 const Quaterniond &Qj,
                                 const Vector3d &Vj,
                                 const Vector3d &Baj,
                                 const Vector3d &Bgj) {
    // NOTE: low cost update jacobian here

//    DLOG(INFO) << "g_vec_: " << g_vec_.transpose();

    Matrix<double, 15, 1> residuals;

    Matrix3d dp_dba = jacobian_.block<3, 3>(O_P, O_BA);
    Matrix3d dp_dbg = jacobian_.block<3, 3>(O_P, O_BG);

    Matrix3d dq_dbg = jacobian_.block<3, 3>(O_R, O_BG);

    Matrix3d dv_dba = jacobian_.block<3, 3>(O_V, O_BA);
    Matrix3d dv_dbg = jacobian_.block<3, 3>(O_V, O_BG);

    Vector3d dba = Bai - linearized_ba_;
    Vector3d dbg = Bgi - linearized_bg_; // NOTE: optimized one minus the linearized one

    Quaterniond corrected_delta_q = delta_q_ * DeltaQ(dq_dbg * dbg);
    Vector3d corrected_delta_v = delta_v_ + dv_dba * dba + dv_dbg * dbg;
    Vector3d corrected_delta_p = delta_p_ + dp_dba * dba + dp_dbg * dbg;

    residuals.block<3, 1>(O_P, 0) =
        Qi.inverse() * (-0.5 * g_vec_ * sum_dt_ * sum_dt_ + Pj - Pi - Vi * sum_dt_) - corrected_delta_p;
    residuals.block<3, 1>(O_R, 0) = 2 * (corrected_delta_q.inverse() * (Qi.inverse() * Qj)).vec();
    residuals.block<3, 1>(O_V, 0) = Qi.inverse() * (-g_vec_ * sum_dt_ + Vj - Vi) - corrected_delta_v;
    residuals.block<3, 1>(O_BA, 0) = Baj - Bai;
    residuals.block<3, 1>(O_BG, 0) = Bgj - Bgi;

//    DLOG(INFO) << Qi.inverse().coeffs().transpose();
//    DLOG(INFO) << g_vec_.transpose();
//    DLOG(INFO) << sum_dt_;
//    DLOG(INFO) << Pi.transpose();
//    DLOG(INFO) << Pj.transpose();
//    DLOG(INFO) << Vi.transpose();
//    DLOG(INFO) << "pred: " << (Qi.inverse() * (-0.5 * g_vec_ * sum_dt_ * sum_dt_ + Pj - Pi - Vi * sum_dt_)).transpose();
//    DLOG(INFO) << "meas: " << corrected_delta_p.transpose();

    return residuals;
  }

// protected:

  double dt_;
  Vector3d acc0_, gyr0_;
  Vector3d acc1_, gyr1_;

  const Vector3d linearized_acc_, linearized_gyr_;
  Vector3d linearized_ba_, linearized_bg_;

  Matrix<double, 15, 15> jacobian_, covariance_;
//  Matrix<double, 15, 15> step_jacobian;
//  Matrix<double, 15, 18> step_V;
  Matrix<double, 18, 18> noise_;

  double sum_dt_;
  Vector3d delta_p_;
  Quaterniond delta_q_;
  Vector3d delta_v_;

  std::vector<double> dt_buf_;
  std::vector<Vector3d> acc_buf_;
  std::vector<Vector3d> gyr_buf_;

  IntegrationBaseConfig config_;

  Eigen::Vector3d g_vec_;

};

}

#endif //LIO_INTEGRATIONBASE_H_

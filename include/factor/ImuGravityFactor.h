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
// Created by hyye on 4/11/18.
//

#ifndef LIO_IMUGRAVITYFACTOR_H_
#define LIO_IMUGRAVITYFACTOR_H_

/// adapted from VINS-mono

#include <ceres/ceres.h>
#include "imu_processor/IntegrationBase.h"
#include "utils/math_utils.h"

namespace lio {

using namespace mathutils;

class ImuGravityFactor : public ceres::SizedCostFunction<15, 7, 9, 7, 9, 4> {

 public:
  ImuGravityFactor() = delete;
  ImuGravityFactor(std::shared_ptr<IntegrationBase> pre_integration) : pre_integration_{
      pre_integration}, g_norm_{pre_integration->config_.g_norm} {
    GI_ = g_norm_ * GI_dir_;
  }
  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {

    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]);
    Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]);

    Eigen::Vector3d Pj(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond Qj(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

    Eigen::Vector3d Vj(parameters[3][0], parameters[3][1], parameters[3][2]);
    Eigen::Vector3d Baj(parameters[3][3], parameters[3][4], parameters[3][5]);
    Eigen::Vector3d Bgj(parameters[3][6], parameters[3][7], parameters[3][8]);

    Eigen::Quaterniond Qwi(parameters[4][3], parameters[4][0], parameters[4][1], parameters[4][2]);

    Eigen::Vector3d GW = Qwi * GI_;

//Eigen::Matrix<double, 15, 15> Fd;
//Eigen::Matrix<double, 15, 12> Gd;

//Eigen::Vector3d pPj = Pi + Vi * sum_t - 0.5 * g * sum_t * sum_t + corrected_delta_p;
//Eigen::Quaterniond pQj = Qi * delta_q;
//Eigen::Vector3d pVj = Vi - g * sum_t + corrected_delta_v;
//Eigen::Vector3d pBaj = Bai;
//Eigen::Vector3d pBgj = Bgi;

//Vi + Qi * delta_v - g * sum_dt = Vj;
//Qi * delta_q = Qj;

//delta_p = Qi.inverse() * (0.5 * g * sum_dt * sum_dt + Pj - Pi);
//delta_v = Qi.inverse() * (g * sum_dt + Vj - Vi);
//delta_q = Qi.inverse() * Qj;

#if 0
    if ((Bai - pre_integration_->linearized_ba_).norm() > 0.10 ||
            (Bgi - pre_integration_->linearized_bg_).norm() > 0.01)
        {
            pre_integration_->Repropagate(Bai, Bgi);
        }
#endif

    Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
    residual = pre_integration_->Evaluate(Pi, Qi, Vi, Bai, Bgi,
                                          Pj, Qj, Vj, Baj, Bgj,
                                          GW);

//    DLOG(INFO) << "GW: " << GW.transpose();

    Eigen::Matrix<double, 15, 15> sqrt_info =
        Eigen::LLT<Eigen::Matrix<double, 15, 15>>(pre_integration_->covariance_.inverse()).matrixL().transpose();
    //sqrt_info.setIdentity();
    residual = sqrt_info * residual;

    if (jacobians) {
      double sum_dt = pre_integration_->sum_dt_;
      Eigen::Matrix3d dp_dba = pre_integration_->jacobian_.template block<3, 3>(O_P, O_BA);
      Eigen::Matrix3d dp_dbg = pre_integration_->jacobian_.template block<3, 3>(O_P, O_BG);

      Eigen::Matrix3d dq_dbg = pre_integration_->jacobian_.template block<3, 3>(O_R, O_BG);

      Eigen::Matrix3d dv_dba = pre_integration_->jacobian_.template block<3, 3>(O_V, O_BA);
      Eigen::Matrix3d dv_dbg = pre_integration_->jacobian_.template block<3, 3>(O_V, O_BG);

      if (pre_integration_->jacobian_.maxCoeff() > 1e8 || pre_integration_->jacobian_.minCoeff() < -1e8) {
        ROS_DEBUG("numerical unstable in preintegration");
        //std::cout << pre_integration_->jacobian_ << std::endl;
///                ROS_BREAK();
      }

      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
        jacobian_pose_i.setZero();

        jacobian_pose_i.block<3, 3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();
        jacobian_pose_i.block<3, 3>(O_P, O_R) =
            SkewSymmetric(Qi.inverse() * (-0.5 * GW * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

#if 0
        jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Qj.inverse() * Qi).toRotationMatrix();
#else
        Eigen::Quaterniond corrected_delta_q =
            pre_integration_->delta_q_ * DeltaQ(dq_dbg * (Bgi - pre_integration_->linearized_bg_));
        jacobian_pose_i.block<3, 3>(O_R, O_R) =
            -(LeftQuatMatrix(Qj.inverse() * Qi) * RightQuatMatrix(corrected_delta_q)).topLeftCorner<3, 3>();
#endif

        jacobian_pose_i.block<3, 3>(O_V, O_R) =
            SkewSymmetric(Qi.inverse() * (-GW * sum_dt + Vj - Vi));

        jacobian_pose_i = sqrt_info * jacobian_pose_i;

        if (jacobian_pose_i.maxCoeff() > 1e8 || jacobian_pose_i.minCoeff() < -1e8) {
          ROS_DEBUG("numerical unstable in preintegration");
          //std::cout << sqrt_info << std::endl;
          //ROS_BREAK();
        }
      }

      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_i(jacobians[1]);
        jacobian_speedbias_i.setZero();
        jacobian_speedbias_i.block<3, 3>(O_P, O_V - O_V) = -Qi.inverse().toRotationMatrix() * sum_dt;
        jacobian_speedbias_i.block<3, 3>(O_P, O_BA - O_V) = -dp_dba;
        jacobian_speedbias_i.block<3, 3>(O_P, O_BG - O_V) = -dp_dbg;

#if 0
        jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -dq_dbg;
#else
        Eigen::Quaterniond corrected_delta_q =
            pre_integration_->delta_q_ * DeltaQ(dq_dbg * (Bgi - pre_integration_->linearized_bg_));
        jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) =
            -LeftQuatMatrix(Qj.inverse() * Qi * corrected_delta_q).topLeftCorner<3, 3>() * dq_dbg;
#endif

        jacobian_speedbias_i.block<3, 3>(O_V, O_V - O_V) = -Qi.inverse().toRotationMatrix();
        jacobian_speedbias_i.block<3, 3>(O_V, O_BA - O_V) = -dv_dba;
        jacobian_speedbias_i.block<3, 3>(O_V, O_BG - O_V) = -dv_dbg;

        jacobian_speedbias_i.block<3, 3>(O_BA, O_BA - O_V) = -Eigen::Matrix3d::Identity();

        jacobian_speedbias_i.block<3, 3>(O_BG, O_BG - O_V) = -Eigen::Matrix3d::Identity();

        jacobian_speedbias_i = sqrt_info * jacobian_speedbias_i;

        //ROS_ASSERT(fabs(jacobian_speedbias_i.maxCoeff()) < 1e8);
        //ROS_ASSERT(fabs(jacobian_speedbias_i.minCoeff()) < 1e8);
      }

      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[2]);
        jacobian_pose_j.setZero();

        jacobian_pose_j.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();

#if 0
        jacobian_pose_j.block<3, 3>(O_R, O_R) = Eigen::Matrix3d::Identity();
#else
        Eigen::Quaterniond corrected_delta_q =
            pre_integration_->delta_q_ * DeltaQ(dq_dbg * (Bgi - pre_integration_->linearized_bg_));
        jacobian_pose_j.block<3, 3>(O_R, O_R) =
            LeftQuatMatrix(corrected_delta_q.inverse() * Qi.inverse() * Qj).topLeftCorner<3, 3>();
#endif

        jacobian_pose_j = sqrt_info * jacobian_pose_j;

        //ROS_ASSERT(fabs(jacobian_pose_j.maxCoeff()) < 1e8);
        //ROS_ASSERT(fabs(jacobian_pose_j.minCoeff()) < 1e8);
      }
      if (jacobians[3]) {
        Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_j(jacobians[3]);
        jacobian_speedbias_j.setZero();

        jacobian_speedbias_j.block<3, 3>(O_V, O_V - O_V) = Qi.inverse().toRotationMatrix();

        jacobian_speedbias_j.block<3, 3>(O_BA, O_BA - O_V) = Eigen::Matrix3d::Identity();

        jacobian_speedbias_j.block<3, 3>(O_BG, O_BG - O_V) = Eigen::Matrix3d::Identity();

        jacobian_speedbias_j = sqrt_info * jacobian_speedbias_j;

        //ROS_ASSERT(fabs(jacobian_speedbias_j.maxCoeff()) < 1e8);
        //ROS_ASSERT(fabs(jacobian_speedbias_j.minCoeff()) < 1e8);
      }

      if (jacobians[4]) {
        Eigen::Map<Eigen::Matrix<double, 15, 4, Eigen::RowMajor>> jacobian_gravity(jacobians[4]);
        jacobian_gravity.setZero();

        Eigen::Matrix<double, 3, 3> dP_dg
            = 0.5 * Qi.inverse().toRotationMatrix() * Qwi.toRotationMatrix() * SkewSymmetric(GI_) * sum_dt * sum_dt;
        jacobian_gravity.block<3, 2>(O_P, 0) = dP_dg.leftCols<2>();

        Eigen::Matrix<double, 3, 3> dV_dg
            = Qi.inverse().toRotationMatrix() * Qwi.toRotationMatrix() * SkewSymmetric(GI_) * sum_dt;
        jacobian_gravity.block<3, 2>(O_V, 0) = dV_dg.leftCols<2>();

        // FIXME: set the sqrt_info as the value from initialization
        jacobian_gravity = sqrt_info * jacobian_gravity;
      }
    }

    return true;
  }

  void Check(double const *const *parameters) {
    double *res = new double[15];
    double **jaco = new double *[5];
    jaco[0] = new double[15 * 7];
    jaco[1] = new double[15 * 9];
    jaco[2] = new double[15 * 7];
    jaco[3] = new double[15 * 9];
    jaco[4] = new double[15 * 4];
    Evaluate(parameters, res, jaco);
    DLOG(INFO) << "check begins";
    DLOG(INFO) << "analytical";

    DLOG(INFO) << Eigen::Map<Eigen::Matrix<double, 15, 1>>(res).transpose();
    DLOG(INFO) << std::endl << Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>>(jaco[0]);
    DLOG(INFO) << std::endl << Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>>(jaco[1]);
    DLOG(INFO) << std::endl << Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>>(jaco[2]);
    DLOG(INFO) << std::endl << Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>>(jaco[3]);
    DLOG(INFO) << std::endl << Eigen::Map<Eigen::Matrix<double, 15, 4, Eigen::RowMajor>>(jaco[4]);

    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]);
    Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]);

    Eigen::Vector3d Pj(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond Qj(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

    Eigen::Vector3d Vj(parameters[3][0], parameters[3][1], parameters[3][2]);
    Eigen::Vector3d Baj(parameters[3][3], parameters[3][4], parameters[3][5]);
    Eigen::Vector3d Bgj(parameters[3][6], parameters[3][7], parameters[3][8]);

    Eigen::Quaterniond Qwi(parameters[4][3], parameters[4][0], parameters[4][1], parameters[4][2]);

    Eigen::Vector3d GW = Qwi * GI_;

    Eigen::Matrix<double, 15, 1> residual;
    residual = pre_integration_->Evaluate(Pi, Qi, Vi, Bai, Bgi,
                                          Pj, Qj, Vj, Baj, Bgj,
                                          GW);

    Eigen::Matrix<double, 15, 15> sqrt_info =
        Eigen::LLT<Eigen::Matrix<double, 15, 15>>(pre_integration_->covariance_.inverse()).matrixL().transpose();
    //sqrt_info.setIdentity();
    residual = sqrt_info * residual;

    DLOG(INFO) << "num";
    DLOG(INFO) << residual.transpose();

    const double eps = 1e-6;
    Eigen::Matrix<double, 15, 33> num_jacobian;
    for (int k = 0; k < 33; k++) {
      Pi = Eigen::Vector3d{parameters[0][0], parameters[0][1], parameters[0][2]};
      Qi = Eigen::Quaterniond(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

      Vi = Eigen::Vector3d(parameters[1][0], parameters[1][1], parameters[1][2]);
      Bai = Eigen::Vector3d(parameters[1][3], parameters[1][4], parameters[1][5]);
      Bgi = Eigen::Vector3d(parameters[1][6], parameters[1][7], parameters[1][8]);

      Pj = Eigen::Vector3d{parameters[2][0], parameters[2][1], parameters[2][2]};
      Qj = Eigen::Quaterniond(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

      Vj = Eigen::Vector3d(parameters[3][0], parameters[3][1], parameters[3][2]);
      Baj = Eigen::Vector3d(parameters[3][3], parameters[3][4], parameters[3][5]);
      Bgj = Eigen::Vector3d(parameters[3][6], parameters[3][7], parameters[3][8]);

      Qwi = Eigen::Quaterniond(parameters[4][3], parameters[4][0], parameters[4][1], parameters[4][2]);

      int a = k / 3, b = k % 3;
      Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

      if (a == 0)
        Pi += delta;
      else if (a == 1)
        Qi = Qi * DeltaQ(delta);
      else if (a == 2)
        Vi += delta;
      else if (a == 3)
        Bai += delta;
      else if (a == 4)
        Bgi += delta;
      else if (a == 5)
        Pj += delta;
      else if (a == 6)
        Qj = Qj * DeltaQ(delta);
      else if (a == 7)
        Vj += delta;
      else if (a == 8)
        Baj += delta;
      else if (a == 9)
        Bgj += delta;
      else if (a == 10)
        Qwi = Qwi * DeltaQ(delta);

      GW = Qwi * GI_;

      Eigen::Matrix<double, 15, 1> tmp_residual;
      tmp_residual = pre_integration_->Evaluate(Pi, Qi, Vi, Bai, Bgi,
                                                Pj, Qj, Vj, Baj, Bgj,
                                                GW);

      tmp_residual = sqrt_info * tmp_residual;

      num_jacobian.col(k) = (tmp_residual - residual) / eps;
    }
    DLOG(INFO) << std::endl << num_jacobian.block<15, 6>(0, 0);
    DLOG(INFO) << std::endl << num_jacobian.block<15, 9>(0, 6);
    DLOG(INFO) << std::endl << num_jacobian.block<15, 6>(0, 15);
    DLOG(INFO) << std::endl << num_jacobian.block<15, 9>(0, 21);
    DLOG(INFO) << std::endl << num_jacobian.block<15, 3>(0, 30);
  }

  //bool Evaluate_Direct(double const *const *parameters, Eigen::Matrix<double, 15, 1> &residuals, Eigen::Matrix<double, 15, 30> &jacobians);

  //void checkCorrection();
  //void checkTransition();
  //void checkJacobian(double **parameters);
//  IntegrationBase *pre_integration_;
  std::shared_ptr<IntegrationBase> pre_integration_;
  double g_norm_;

 private:
  Eigen::Vector3d GI_dir_{0.0, 0.0, -1.0}; ///< direction of gravity in an inertial reference
  Eigen::Vector3d GI_; ///< direction of gravity in an inertial reference

};

} // namespace lio

#endif //LIO_IMUGRAVITYFACTOR_H_

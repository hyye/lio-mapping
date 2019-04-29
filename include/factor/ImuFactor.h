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
// Created by hyye on 4/4/18.
//

#ifndef LIO_IMUFACTOR_H_
#define LIO_IMUFACTOR_H_

/// adapted from VINS-mono

#include <ceres/ceres.h>
#include "imu_processor/IntegrationBase.h"
#include "utils/math_utils.h"

namespace lio {

using namespace mathutils;

class ImuFactor : public ceres::SizedCostFunction<15, 7, 9, 7, 9> {

 public:
  ImuFactor() = delete;
  ImuFactor(std::shared_ptr<IntegrationBase> pre_integration) : pre_integration_{
      pre_integration} {
    // NOTE: g_vec_ is the gravity in laser's original frame
    g_vec_ = pre_integration_->g_vec_;
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

    Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
    residual = pre_integration_->Evaluate(Pi, Qi, Vi, Bai, Bgi,
                                          Pj, Qj, Vj, Baj, Bgj);


    Eigen::Matrix<double, 15, 15> sqrt_info =
        Eigen::LLT<Eigen::Matrix<double, 15, 15>>(pre_integration_->covariance_.inverse()).matrixL().transpose();

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
      }

      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
        jacobian_pose_i.setZero();

        jacobian_pose_i.block<3, 3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();
        jacobian_pose_i.block<3, 3>(O_P, O_R) =
            SkewSymmetric(Qi.inverse() * (-0.5 * g_vec_ * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

        Eigen::Quaterniond corrected_delta_q =
            pre_integration_->delta_q_ * DeltaQ(dq_dbg * (Bgi - pre_integration_->linearized_bg_));
        jacobian_pose_i.block<3, 3>(O_R, O_R) =
            -(LeftQuatMatrix(Qj.inverse() * Qi) * RightQuatMatrix(corrected_delta_q)).topLeftCorner<3, 3>();

        jacobian_pose_i.block<3, 3>(O_V, O_R) =
            SkewSymmetric(Qi.inverse() * (-g_vec_ * sum_dt + Vj - Vi));

        jacobian_pose_i = sqrt_info * jacobian_pose_i;

        if (jacobian_pose_i.maxCoeff() > 1e8 || jacobian_pose_i.minCoeff() < -1e8) {
          ROS_DEBUG("numerical unstable in preintegration");
        }
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_i(jacobians[1]);
        jacobian_speedbias_i.setZero();
        jacobian_speedbias_i.block<3, 3>(O_P, O_V - O_V) = -Qi.inverse().toRotationMatrix() * sum_dt;
        jacobian_speedbias_i.block<3, 3>(O_P, O_BA - O_V) = -dp_dba;
        jacobian_speedbias_i.block<3, 3>(O_P, O_BG - O_V) = -dp_dbg;

        Eigen::Quaterniond corrected_delta_q =
            pre_integration_->delta_q_ * DeltaQ(dq_dbg * (Bgi - pre_integration_->linearized_bg_));
        jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) =
            -LeftQuatMatrix(Qj.inverse() * Qi * corrected_delta_q).topLeftCorner<3, 3>() * dq_dbg;

        jacobian_speedbias_i.block<3, 3>(O_V, O_V - O_V) = -Qi.inverse().toRotationMatrix();
        jacobian_speedbias_i.block<3, 3>(O_V, O_BA - O_V) = -dv_dba;
        jacobian_speedbias_i.block<3, 3>(O_V, O_BG - O_V) = -dv_dbg;

        jacobian_speedbias_i.block<3, 3>(O_BA, O_BA - O_V) = -Eigen::Matrix3d::Identity();

        jacobian_speedbias_i.block<3, 3>(O_BG, O_BG - O_V) = -Eigen::Matrix3d::Identity();

        jacobian_speedbias_i = sqrt_info * jacobian_speedbias_i;

      }
      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[2]);
        jacobian_pose_j.setZero();

        jacobian_pose_j.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();

        Eigen::Quaterniond corrected_delta_q =
            pre_integration_->delta_q_ * DeltaQ(dq_dbg * (Bgi - pre_integration_->linearized_bg_));
        jacobian_pose_j.block<3, 3>(O_R, O_R) =
            LeftQuatMatrix(corrected_delta_q.inverse() * Qi.inverse() * Qj).topLeftCorner<3, 3>();

        jacobian_pose_j = sqrt_info * jacobian_pose_j;

      }
      if (jacobians[3]) {
        Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_j(jacobians[3]);
        jacobian_speedbias_j.setZero();

        jacobian_speedbias_j.block<3, 3>(O_V, O_V - O_V) = Qi.inverse().toRotationMatrix();

        jacobian_speedbias_j.block<3, 3>(O_BA, O_BA - O_V) = Eigen::Matrix3d::Identity();

        jacobian_speedbias_j.block<3, 3>(O_BG, O_BG - O_V) = Eigen::Matrix3d::Identity();

        jacobian_speedbias_j = sqrt_info * jacobian_speedbias_j;

      }
    }

    return true;
  }

  std::shared_ptr<IntegrationBase> pre_integration_;
  Eigen::Vector3d g_vec_;

  const double eps_ = 10e-8;

};

} // namespace lio

#endif //LIO_IMUFACTOR_H_

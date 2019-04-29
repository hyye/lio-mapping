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
// Created by hyye on 4/12/18.
//

#include "factor/PlaneProjectionFactor.h"

namespace lio {

PlaneProjectionFactor::PlaneProjectionFactor(const Eigen::Vector4d &local_coeffi,
                                             const Eigen::Vector4d &local_coeffj,
                                             const double &score)
    : local_coeffi_{local_coeffi}, local_coeffj_{local_coeffj}, score_{score} {}

bool PlaneProjectionFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
  Eigen::Vector3d Pi{parameters[0][0], parameters[0][1], parameters[0][2]};
  Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

  Eigen::Vector3d Pj{parameters[1][0], parameters[1][1], parameters[1][2]};
  Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

  Eigen::Vector3d tlb{parameters[2][0], parameters[2][1], parameters[2][2]};
  Eigen::Quaterniond qlb(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

  Eigen::Quaterniond Qli = Qi * qlb.conjugate();
  Eigen::Vector3d Pli = Pi - Qli * tlb;
  Eigen::Quaterniond Qlj = Qj * qlb.conjugate();
  Eigen::Vector3d Plj = Pj - Qlj * tlb;

  Twist<double> transform_li{Qli, Pli};
  Twist<double> transform_lj{Qlj, Plj};
  Twist<double> transform_lb{qlb, tlb};

  Eigen::Vector3d wi = local_coeffi_.head<3>();
  double bi = local_coeffi_.w();

//  DLOG(INFO) << "wi: " << wi.transpose();

  double disi = local_coeffi_.w();
  double disj = local_coeffj_.w();
  LOG_IF(INFO, disi < 0) << "disi less than zero: " << disi;
  LOG_IF(INFO, disj < 0) << "disj less than zero: " << disj;
//  Eigen::Vector3d plane_cen_i{-disi * local_coeffi_.x(), -disi * local_coeffi_.y(), -disi * local_coeffi_.z()};
//  Eigen::Vector3d plane_cen_j{-disj * local_coeffj_.x(), -disj * local_coeffj_.y(), -disj * local_coeffj_.z()};

  Eigen::Vector4d
      coeffi_in_j = (transform_li.inverse() * transform_lj).transform().matrix().transpose() * local_coeffi_;

  if (coeffi_in_j.w() < 0) {
//    LOG_IF(INFO, coeffi_in_j.w() < 0) << "disi_in_j less than zero: " << coeffi_in_j.w();

    coeffi_in_j = (-coeffi_in_j).eval();

    LOG_IF(INFO, coeffi_in_j.w() < 0) << "disi_in_j less than zero: " << coeffi_in_j.w();
  }

  double info = score_;
  // FIXME: 100 magic number, info_mat
  Eigen::Map<Eigen::Matrix<double, 4, 1>> residual(residuals);
  residual = info * (coeffi_in_j - local_coeffj_);

  if (jacobians) {
    Eigen::Matrix3d Ri = Qi.toRotationMatrix();
    Eigen::Matrix3d Rj = Qj.toRotationMatrix();
    Eigen::Matrix3d rlb = qlb.toRotationMatrix();

    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 4, 7, Eigen::RowMajor> > jacobian_pose_i(jacobians[0]);
      Eigen::Matrix<double, 4, 6> jaco_i;

      jaco_i.setZero();
      jaco_i.bottomLeftCorner<1, 3>() = -wi.transpose() * rlb * Ri.transpose();
      jaco_i.topRightCorner<3, 3>() =
          -rlb * Rj.transpose() * Ri * SkewSymmetric(rlb.transpose() * wi);
      jaco_i.bottomRightCorner<1, 3>() =
          wi.transpose() * rlb * SkewSymmetric(Ri.transpose() * (Pj - Pi - Rj * rlb.transpose() * tlb));

      // FIXME: 100 magic number, info_mat
      jacobian_pose_i.setZero();
      jacobian_pose_i.leftCols<6>() = info * jaco_i;
      jacobian_pose_i.rightCols<1>().setZero();
    }

    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 4, 7, Eigen::RowMajor> > jacobian_pose_j(jacobians[1]);
      Eigen::Matrix<double, 4, 6> jaco_j;

      jaco_j.setZero();
      jaco_j.bottomLeftCorner<1, 3>() = wi.transpose() * rlb * Rj.transpose();
      jaco_j.topRightCorner<3, 3>() =
          rlb * SkewSymmetric(Rj.transpose() * Ri * rlb.transpose() * wi);
      jaco_j.bottomRightCorner<1, 3>() =
          wi.transpose() * rlb * Ri.transpose() * Rj * SkewSymmetric(rlb.transpose() * tlb);

      // FIXME: 100 magic number, info_mat
      jacobian_pose_j.setZero();
      jacobian_pose_j.leftCols<6>() = info * jaco_j;
      jacobian_pose_j.rightCols<1>().setZero();
    }

    if (jacobians[2]) {
      Eigen::Map<Eigen::Matrix<double, 4, 7, Eigen::RowMajor> > jacobian_pose_ex(jacobians[2]);
      Eigen::Matrix<double, 4, 6> jaco_ex;

      jaco_ex.setZero();
      jaco_ex.bottomLeftCorner<1, 3>() =
          -wi.transpose() * rlb * Ri.transpose() * (Rj - Ri) * rlb.transpose();
      jaco_ex.topRightCorner<3, 3>() = rlb * Rj.transpose() * Ri * SkewSymmetric(rlb.transpose() * wi)
          - rlb * SkewSymmetric(Rj.transpose() * Ri * rlb.transpose() * wi);
      jaco_ex.bottomRightCorner<1, 3>() =
          wi.transpose() * (-rlb * Ri.transpose() * (Rj - Ri) * SkewSymmetric(rlb.transpose() * tlb)
              - rlb * SkewSymmetric(Ri.transpose() * (Pj - Pi - (Rj - Ri) * rlb.transpose() * tlb)));

      // FIXME: 100 magic number, info_mat
      jacobian_pose_ex.setZero();
      jacobian_pose_ex.leftCols<6>() = info * jaco_ex;
      jacobian_pose_ex.rightCols<1>().setZero();
    }
  }

  return true;

} // Evaluate

void PlaneProjectionFactor::Check(double **parameters) {

  double *res = new double[4];
  double **jaco = new double *[3];
  jaco[0] = new double[4 * 7];
  jaco[1] = new double[4 * 7];
  jaco[2] = new double[4 * 7];
  Evaluate(parameters, res, jaco);
  DLOG(INFO) << "check begins";
  DLOG(INFO) << "analytical";

  DLOG(INFO) << Eigen::Map<Eigen::Matrix<double, 4, 1>>(res).transpose();
  DLOG(INFO) << std::endl << Eigen::Map<Eigen::Matrix<double, 4, 7, Eigen::RowMajor>>(jaco[0]);
  DLOG(INFO) << std::endl << Eigen::Map<Eigen::Matrix<double, 4, 7, Eigen::RowMajor>>(jaco[1]);
  DLOG(INFO) << std::endl << Eigen::Map<Eigen::Matrix<double, 4, 7, Eigen::RowMajor>>(jaco[2]);

  Eigen::Vector3d Pi{parameters[0][0], parameters[0][1], parameters[0][2]};
  Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

  Eigen::Vector3d Pj{parameters[1][0], parameters[1][1], parameters[1][2]};
  Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

  Eigen::Vector3d tlb{parameters[2][0], parameters[2][1], parameters[2][2]};
  Eigen::Quaterniond qlb(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

  Eigen::Quaterniond Qli = Qi * qlb.conjugate();
  Eigen::Vector3d Pli = Pi - Qli * tlb;
  Eigen::Quaterniond Qlj = Qj * qlb.conjugate();
  Eigen::Vector3d Plj = Pj - Qlj * tlb;

  Twist<double> transform_li{Qli, Pli};
  Twist<double> transform_lj{Qlj, Plj};
  Twist<double> transform_lb{qlb, tlb};

  Eigen::Vector3d wi = local_coeffi_.head<3>();
  double bi = local_coeffi_.w();
  double info = score_;

  Eigen::Vector4d residual;
  residual =
      info * ((transform_li.inverse() * transform_lj).transform().matrix().transpose() * local_coeffi_ - local_coeffj_);

  DLOG(INFO) << "num";
  DLOG(INFO) << residual.transpose();

  const double eps = 1e-6;
  Eigen::Matrix<double, 4, 18> num_jacobian;
  for (int k = 0; k < 18; k++) {
    Pi = Eigen::Vector3d{parameters[0][0], parameters[0][1], parameters[0][2]};
    Qi = Eigen::Quaterniond(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Pj = Eigen::Vector3d{parameters[1][0], parameters[1][1], parameters[1][2]};
    Qj = Eigen::Quaterniond(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    tlb = Eigen::Vector3d{parameters[2][0], parameters[2][1], parameters[2][2]};
    qlb = Eigen::Quaterniond(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

    int a = k / 3, b = k % 3;
    Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

    if (a == 0)
      Pi += delta;
    else if (a == 1)
      Qi = Qi * DeltaQ(delta);
    else if (a == 2)
      Pj += delta;
    else if (a == 3)
      Qj = Qj * DeltaQ(delta);
    else if (a == 4)
      tlb += delta;
    else if (a == 5)
      qlb = qlb * DeltaQ(delta);

    Qli = Qi * qlb.conjugate();
    Pli = Pi - Qli * tlb;
    Qlj = Qj * qlb.conjugate();
    Plj = Pj - Qlj * tlb;

    transform_li = Twist<double>{Qli, Pli};
    transform_lj = Twist<double>{Qlj, Plj};
    transform_lb = Twist<double>{qlb, tlb};

    Eigen::Vector4d tmp_residual;

    tmp_residual = info
        * ((transform_li.inverse() * transform_lj).transform().matrix().transpose() * local_coeffi_ - local_coeffj_);

    num_jacobian.col(k) = (tmp_residual - residual) / eps;
  }
  DLOG(INFO) << std::endl << num_jacobian.block<4, 6>(0, 0);
  DLOG(INFO) << std::endl << num_jacobian.block<4, 6>(0, 6);
  DLOG(INFO) << std::endl << num_jacobian.block<4, 6>(0, 12);

}

} // namespace lio

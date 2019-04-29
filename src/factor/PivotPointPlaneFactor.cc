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
// Created by hyye on 5/17/18.
//

#include "factor/PivotPointPlaneFactor.h"

namespace lio {

static double sqrt_info_static = 1.0;

PivotPointPlaneFactor::PivotPointPlaneFactor(const Eigen::Vector3d &point,
                                             const Eigen::Vector4d &coeff) : point_{point},
                                                                             coeff_{coeff} {
/// add new point and coeff
}

bool PivotPointPlaneFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
  TicToc tic_toc;

  Eigen::Vector3d P_pivot(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond Q_pivot(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

  Eigen::Vector3d Pi(parameters[1][0], parameters[1][1], parameters[1][2]);
  Eigen::Quaterniond Qi(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

  Eigen::Vector3d tlb(parameters[2][0], parameters[2][1], parameters[2][2]);
  Eigen::Quaterniond qlb(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

//  Eigen::Vector3d tlb = transform_lb_.pos;
//  Eigen::Quaterniond qlb = transform_lb_.rot;

  Eigen::Quaterniond Qlpivot = Q_pivot * qlb.conjugate();
  Eigen::Vector3d Plpivot = P_pivot - Qlpivot * tlb;

  Eigen::Quaterniond Qli = Qi * qlb.conjugate();
  Eigen::Vector3d Pli = Pi - Qli * tlb;

  Eigen::Quaterniond Qlpi = Qlpivot.conjugate() * Qli;
  Eigen::Vector3d Plpi = Qlpivot.conjugate() * (Pli - Plpivot);

  Eigen::Vector3d w(coeff_.x(), coeff_.y(), coeff_.z());
  double b = coeff_.w();

  double residual = (w.transpose() * (Qlpi * point_ + Plpi) + b);

  double sqrt_info = sqrt_info_static;

  residuals[0] = sqrt_info * residual;

  if (jacobians) {
    Eigen::Matrix3d Ri = Qi.toRotationMatrix();
    Eigen::Matrix3d Rp = Q_pivot.toRotationMatrix();
    Eigen::Matrix3d rlb = qlb.toRotationMatrix();

    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > jacobian_pose_pivot(jacobians[0]);
      Eigen::Matrix<double, 1, 6> jaco_pivot;

      jaco_pivot.leftCols<3>() = -w.transpose() * rlb * Rp.transpose();
      jaco_pivot.rightCols<3>() =
          w.transpose() * rlb * (SkewSymmetric(Rp.transpose() * Ri * rlb.transpose() * (point_ - tlb))
              + SkewSymmetric(Rp.transpose() * (Pi - P_pivot)));

      jacobian_pose_pivot.setZero();
      jacobian_pose_pivot.leftCols<6>() = sqrt_info * jaco_pivot;
      jacobian_pose_pivot.rightCols<1>().setZero();
    }

    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > jacobian_pose_i(jacobians[1]);
      Eigen::Matrix<double, 1, 6> jaco_i;

      jaco_i.leftCols<3>() = w.transpose() * rlb * Rp.transpose();
      jaco_i.rightCols<3>() =
          w.transpose() * rlb * Rp.transpose() * Ri * (-SkewSymmetric(rlb.transpose() * point_)
              + SkewSymmetric(rlb.transpose() * tlb));

      jacobian_pose_i.setZero();
      jacobian_pose_i.leftCols<6>() = sqrt_info * jaco_i;
      jacobian_pose_i.rightCols<1>().setZero();
    }

    if (jacobians[2]) {
      Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > jacobian_pose_ex(jacobians[2]);
      jacobian_pose_ex.setZero();

      Eigen::Matrix3d I3x3;
      I3x3.setIdentity();

      Eigen::Matrix3d right_info_mat;
      right_info_mat.setIdentity();
      right_info_mat(2, 2) = 1e-6;
      right_info_mat = Qlpivot.conjugate().normalized() * right_info_mat * Qlpivot.normalized();

      Eigen::Matrix<double, 1, 6> jaco_ex;
      //  NOTE: planar extrinsic
//       jaco_ex.leftCols<3>() = w.transpose() * (I3x3 - rlb * Rp.transpose() * Ri * rlb.transpose()) * right_info_mat;
      jaco_ex.leftCols<3>() = w.transpose() * (I3x3 - rlb * Rp.transpose() * Ri * rlb.transpose());
      jaco_ex.rightCols<3>() =
          w.transpose() * rlb * (-SkewSymmetric(Rp.transpose() * Ri * rlb.transpose() * (point_ - tlb))
              + Rp.transpose() * Ri * SkewSymmetric(rlb.transpose() * (point_ - tlb))
              - SkewSymmetric(Rp.transpose() * (Pi - P_pivot)));

      jacobian_pose_ex.setZero();
      jacobian_pose_ex.leftCols<6>() = sqrt_info * jaco_ex;
      jacobian_pose_ex.rightCols<1>().setZero();
    }
  }

  return true;
}

void PivotPointPlaneFactor::Check(double **parameters) {

  double *res = new double[1];
//  double **jaco = new double *[1];
  double **jaco = new double *[3];
  jaco[0] = new double[1 * 7];
  jaco[1] = new double[1 * 7];
  jaco[2] = new double[1 * 7];
  Evaluate(parameters, res, jaco);
  DLOG(INFO) << "check begins";
  DLOG(INFO) << "analytical";

  DLOG(INFO) << *res;
  DLOG(INFO) << std::endl << Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>>(jaco[0]);
  DLOG(INFO) << std::endl << Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>>(jaco[1]);
  DLOG(INFO) << std::endl << Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>>(jaco[2]);

  Eigen::Vector3d P_pivot(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond Q_pivot(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

  Eigen::Vector3d Pi(parameters[1][0], parameters[1][1], parameters[1][2]);
  Eigen::Quaterniond Qi(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

  Eigen::Vector3d tlb(parameters[2][0], parameters[2][1], parameters[2][2]);
  Eigen::Quaterniond qlb(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

//  Eigen::Vector3d tlb = transform_lb_.pos;
//  Eigen::Quaterniond qlb = transform_lb_.rot;

  Eigen::Quaterniond Qlpivot = Q_pivot * qlb.conjugate();
  Eigen::Vector3d Plpivot = P_pivot - Qlpivot * tlb;

  Eigen::Quaterniond Qli = Qi * qlb.conjugate();
  Eigen::Vector3d Pli = Pi - Qli * tlb;

  Eigen::Quaterniond Qlpi = Qlpivot.conjugate() * Qli;
  Eigen::Vector3d Plpi = Qlpivot.conjugate() * (Pli - Plpivot);

  Eigen::Vector3d w(coeff_.x(), coeff_.y(), coeff_.z());
  double b = coeff_.w();

  double sqrt_info = sqrt_info_static;

  double residual = (w.transpose() * (Qlpi * point_ + Plpi) + b);

  residual = sqrt_info * residual;
  DLOG(INFO) << "num";
  DLOG(INFO) << residual;

  const double eps = 1e-6;
//  Eigen::Matrix<double, 1, 6> num_jacobian;
  Eigen::Matrix<double, 1, 18> num_jacobian;
//  for (int k = 0; k < 6; k++) {
  for (int k = 0; k < 18; k++) {
    Eigen::Vector3d P_pivot(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Q_pivot(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Pi(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qi(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    Eigen::Vector3d tlb(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond qlb(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

    int a = k / 3, b = k % 3;
    Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

    if (a == 0)
      P_pivot += delta;
    else if (a == 1)
      Q_pivot = Q_pivot * DeltaQ(delta);
    else if (a == 2)
      Pi += delta;
    else if (a == 3)
      Qi = Qi * DeltaQ(delta);
    else if (a == 4)
      tlb += delta;
    else if (a == 5)
      qlb = qlb * DeltaQ(delta);

    Eigen::Quaterniond Qlpivot = Q_pivot * qlb.conjugate();
    Eigen::Vector3d Plpivot = P_pivot - Qlpivot * tlb;

    Eigen::Quaterniond Qli = Qi * qlb.conjugate();
    Eigen::Vector3d Pli = Pi - Qli * tlb;

    Eigen::Quaterniond Qlpi = Qlpivot.conjugate() * Qli;
    Eigen::Vector3d Plpi = Qlpivot.conjugate() * (Pli - Plpivot);

    Eigen::Vector3d coeffw(coeff_.x(), coeff_.y(), coeff_.z());
    double coeffb = coeff_.w();

    double tmp_residual = (coeffw.transpose() * (Qlpi * point_ + Plpi) + coeffb);

    tmp_residual = sqrt_info * tmp_residual;

    num_jacobian(k) = (tmp_residual - residual) / eps;
  }
  DLOG(INFO) << std::endl << num_jacobian.block<1, 6>(0, 0);
  DLOG(INFO) << std::endl << num_jacobian.block<1, 6>(0, 6);
  DLOG(INFO) << std::endl << num_jacobian.block<1, 6>(0, 12);
}

}
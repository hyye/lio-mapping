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
// Created by hyye on 4/3/18.
//

#include "factor/PointDistanceFactor.h"

namespace lio {

static double sqrt_info_static = 100;

PointDistanceFactor::PointDistanceFactor(const Eigen::Vector3d &point,
                                         const Eigen::Vector4d &coeff,
                                         const Eigen::Matrix<double, 6, 6> info_mat) : point_{point},
                                                                                       coeff_{coeff},
                                                                                       info_mat_{info_mat} {
/// add new point and coeff
}

bool PointDistanceFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
  TicToc tic_toc;

  Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

  Eigen::Vector3d tlb(parameters[1][0], parameters[1][1], parameters[1][2]);
  Eigen::Quaterniond qlb(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

//  Eigen::Vector3d tlb = transform_lb_.pos;
//  Eigen::Quaterniond qlb = transform_lb_.rot;


  Eigen::Quaterniond Qli = Qi * qlb.conjugate();
  Eigen::Vector3d Pli = Pi - Qli * tlb;

  Eigen::Vector3d w(coeff_.x(), coeff_.y(), coeff_.z());
  double b = coeff_.w();

  double residual = (w.transpose() * (Qli * point_ + Pli) + b);

  double sqrt_info = sqrt_info_static;
  // FIXME: 100 magic number, info_mat
  residuals[0] = sqrt_info * residual;

  if (jacobians) {
    Eigen::Matrix3d Ri = Qi.toRotationMatrix();
    Eigen::Matrix3d rlb = qlb.toRotationMatrix();

    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > jacobian_pose_i(jacobians[0]);
      Eigen::Matrix<double, 1, 6> jaco_i;

      jaco_i.leftCols<3>() = w.transpose();
      jaco_i.rightCols<3>() =
          -w.transpose() * Ri * (SkewSymmetric(rlb.transpose() * point_) - SkewSymmetric(rlb.transpose() * tlb));

      // FIXME: 100 magic number, info_mat
      jacobian_pose_i.setZero();
      jacobian_pose_i.leftCols<6>() = sqrt_info * jaco_i /* * info_mat_*/;
      jacobian_pose_i.rightCols<1>().setZero();
    }

    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor> > jacobian_pose_ex(jacobians[1]);
      jacobian_pose_ex.setZero();

      Eigen::Matrix<double, 1, 6> jaco_ex;
      jaco_ex.leftCols<3>() = -w.transpose() * Ri * rlb.transpose();
      jaco_ex.rightCols<3>() =
          w.transpose() * Ri * (SkewSymmetric(rlb.transpose() * point_) - SkewSymmetric(rlb.transpose() * tlb));

      // FIXME: 100 magic number, info_mat
      jacobian_pose_ex.setZero();
      jacobian_pose_ex.leftCols<6>() = sqrt_info * jaco_ex;
      jacobian_pose_ex.rightCols<1>().setZero();
    }
  }

  return true;
}

void PointDistanceFactor::Check(double **parameters) {
  Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

  Eigen::Vector3d tlb(parameters[1][0], parameters[1][1], parameters[1][2]);
  Eigen::Quaterniond qlb(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

//  Eigen::Vector3d tlb = transform_lb_.pos;
//  Eigen::Quaterniond qlb = transform_lb_.rot;

  Eigen::Quaterniond Qli = Qi * qlb.conjugate();
  Eigen::Vector3d Pli = Pi - Qli * tlb;

  double *res = new double[1];
//  double **jaco = new double *[1];
  double **jaco = new double *[2];
  jaco[0] = new double[1 * 7];
  jaco[1] = new double[1 * 7];
  Evaluate(parameters, res, jaco);
  DLOG(INFO) << "check begins";
  DLOG(INFO) << "analytical";

  DLOG(INFO) << *res;
  DLOG(INFO) << std::endl << Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>>(jaco[0]);
  DLOG(INFO) << std::endl << Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>>(jaco[1]);

  double sqrt_info = sqrt_info_static;

  Eigen::Vector3d coeffw(coeff_.x(), coeff_.y(), coeff_.z());
  double coeffb = coeff_.w();

  double residual = (coeffw.transpose() * (Qli * point_ + Pli) + coeffb);

  residual = sqrt_info * residual;
  DLOG(INFO) << "num";
  DLOG(INFO) << residual;

  const double eps = 1e-6;
//  Eigen::Matrix<double, 1, 6> num_jacobian;
  Eigen::Matrix<double, 1, 12> num_jacobian;
//  for (int k = 0; k < 6; k++) {
  for (int k = 0; k < 12; k++) {
    Pi = Eigen::Vector3d{parameters[0][0], parameters[0][1], parameters[0][2]};
    Qi = Eigen::Quaterniond(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    tlb = Eigen::Vector3d(parameters[1][0], parameters[1][1], parameters[1][2]);
    qlb = Eigen::Quaterniond(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    int a = k / 3, b = k % 3;
    Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

    if (a == 0)
      Pi += delta;
    else if (a == 1)
      Qi = Qi * DeltaQ(delta);
    else if (a == 2)
      tlb += delta;
    else if (a == 3)
      qlb = qlb * DeltaQ(delta);

    Qli = Qi * qlb.conjugate();
    Pli = Pi - Qli * tlb;

    double tmp_residual = (coeffw.transpose() * (Qli * point_ + Pli) + coeffb);

    tmp_residual = sqrt_info * tmp_residual;

    num_jacobian(k) = (tmp_residual - residual) / eps;
  }
  DLOG(INFO) << std::endl << num_jacobian.block<1, 6>(0, 0);
  DLOG(INFO) << std::endl << num_jacobian.block<1, 6>(0, 6);
}

} // namespace lio
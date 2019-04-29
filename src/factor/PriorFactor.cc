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
// Created by hyye on 18-6-2.
//

#include "factor/PriorFactor.h"

namespace lio {

bool PriorFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
  Eigen::Vector3d P{parameters[0][0], parameters[0][1], parameters[0][2]};
  Eigen::Quaterniond Q(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

  Matrix<double, 6, 6> sqrt_info;
  sqrt_info.setZero();
  sqrt_info.topLeftCorner<3, 3>() = Matrix3d::Identity() * 1000;
  sqrt_info.bottomRightCorner<3, 3>() = Matrix3d::Identity() * 0.1;

  Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
  residual.topRows<3>() = P - pos_;
  residual.bottomRows<3>() = 2 * (rot_.inverse() * Q).coeffs().head<3>();

  // FIXME: info
  residual = sqrt_info * residual;
  DLOG(INFO) << "residual: " << residual.transpose();

  if (jacobians) {
    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor> > jacobian_prior(jacobians[0]);
      Eigen::Matrix<double, 6, 6> jaco_prior;
      jaco_prior.setIdentity();

      jaco_prior.bottomRightCorner<3, 3>() = LeftQuatMatrix(Q.inverse() * rot_).topLeftCorner<3, 3>();

      // FIXME: info
      jacobian_prior.setZero();
      jacobian_prior.leftCols<6>() = sqrt_info * jaco_prior;
      jacobian_prior.rightCols<1>().setZero();
    }
  }
  return true;
}

void PriorFactor::Check(double const *const *parameters) {
  Eigen::Vector3d P{parameters[0][0], parameters[0][1], parameters[0][2]};
  Eigen::Quaterniond Q(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

  double *res = new double[6];
  double **jaco = new double *[1];
  jaco[0] = new double[6 * 7];
  Evaluate(parameters, res, jaco);
  DLOG(INFO) << "check begins";
  DLOG(INFO) << "analytical";

  DLOG(INFO) << Eigen::Map<Eigen::Matrix<double, 6, 1>>(res).transpose();
  DLOG(INFO) << std::endl << Eigen::Map<Eigen::Matrix<double, 6, 7, Eigen::RowMajor>>(jaco[0]);

  double info = 1;

  Eigen::Matrix<double, 6, 1> residual;
  residual.topRows<3>() = P - pos_;
  residual.bottomRows<3>() = 2 * (rot_.inverse() * Q).coeffs().head<3>();

  residual = info * residual;
  DLOG(INFO) << "num";
  DLOG(INFO) << residual.transpose();

  const double eps = 1e-6;
  Eigen::Matrix<double, 6, 7> num_jacobian;
  for (int k = 0; k < 6; k++) {
    P = Eigen::Vector3d{parameters[0][0], parameters[0][1], parameters[0][2]};
    Q = Eigen::Quaterniond(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    int a = k / 3, b = k % 3;
    Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

    if (a == 0)
      P += delta;
    else if (a == 1)
      Q = Q * DeltaQ(delta);

    Eigen::Matrix<double, 6, 1> tmp_residual;

    tmp_residual.topRows<3>() = P - pos_;
    tmp_residual.bottomRows<3>() = 2 * (rot_.inverse() * Q).coeffs().head<3>();

    // FIXME: info
    tmp_residual = info * tmp_residual;

    num_jacobian.col(k) = (tmp_residual - residual) / eps;
  }
  DLOG(INFO) << std::endl << num_jacobian.block<6, 6>(0, 0);
}

}
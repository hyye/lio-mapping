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

#include "factor/GravityLocalParameterization.h"

namespace lio {

bool GravityLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
  Eigen::Map<const Eigen::Quaterniond> q(x);

  Eigen::Map<const Eigen::Vector2d> dq_xy(delta);

  Eigen::Quaterniond dq = DeltaQ(Eigen::Vector3d(dq_xy.x(), dq_xy.y(), 0.0));

  Eigen::Map<Eigen::Quaterniond> q_plus(x_plus_delta);

  q_plus = (q * dq).normalized();

  return true;
}
bool GravityLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
  Eigen::Map<Eigen::Matrix<double, 4, 2, Eigen::RowMajor>> j(jacobian);
  j.topRows<2>().setIdentity();
  j.bottomRows<2>().setZero();

  return true;
}

}
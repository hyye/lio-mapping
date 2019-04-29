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
// Created by hyye on 5/9/18.
//

#ifndef LIO_PLANETOPLANEFACTOR_H_
#define LIO_PLANETOPLANEFACTOR_H_

#include <ceres/ceres.h>
#include <Eigen/Eigen>

#include "utils/CircularBuffer.h"
#include "utils/Twist.h"
#include "utils/common_ros.h"
#include "utils/TicToc.h"
#include "utils/math_utils.h"
#include "utils/geometry_utils.h"

#include "feature_manager/FeatureManager.h"

namespace lio {

using namespace mathutils;

class PlaneToPlaneFactor : public ceres::SizedCostFunction<3, 7, 7, 7> {
 public:
  PlaneToPlaneFactor(const Eigen::Vector3d &pi_local, const Eigen::Vector3d &ni_local,
                     const Eigen::Vector3d &pj_local, const Eigen::Vector3d &nj_local);
  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

  void Check(double **parameters);

  Eigen::Vector3d pi_local_;
  Eigen::Vector3d ni_local_;
  Eigen::Vector3d pj_local_;
  Eigen::Vector3d nj_local_;
//  Eigen::Matrix3d mahalanobis_;

  PointNormalFeature pfi_;
  PointNormalFeature pfj_;

  const double eps_ = 1e-8;

  static double sum_t_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}

#endif //LIO_PLANETOPLANEFACTOR_H_

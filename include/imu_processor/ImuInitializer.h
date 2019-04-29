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
// Created by hyye on 5/20/18.
//

#ifndef LIO_IMUINITIALIZER_H_
#define LIO_IMUINITIALIZER_H_

#include <glog/logging.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "imu_processor/IntegrationBase.h"

#include "utils/Twist.h"
#include "utils/common_ros.h"
#include "utils/TicToc.h"
#include "utils/math_utils.h"
#include "utils/geometry_utils.h"
#include "utils/CircularBuffer.h"
#include "3rdparty/sophus/se3.hpp"

namespace lio {

using namespace std;
using namespace mathutils;
using namespace geometryutils;

using Eigen::VectorXd;

typedef Twist<float> Transform;

struct LaserTransform {
  LaserTransform() {};
  LaserTransform(double laser_time, Transform laser_transform) : time{laser_time}, transform{laser_transform} {};

  double time;
  Transform transform;
  shared_ptr<IntegrationBase> pre_integration;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

typedef pair<double, LaserTransform> PairTimeLaserTransform;

class ImuInitializer {

 public:

  ///< R_WI is the rotation from the inertial frame into Lidar's world frame
  static bool Initialization(CircularBuffer<PairTimeLaserTransform> &all_laser_transforms,
                             CircularBuffer<Vector3d> &Vs,
                             CircularBuffer<Vector3d> &Bas,
                             CircularBuffer<Vector3d> &Bgs,
                             Vector3d &g,
                             Transform &transform_lb,
                             Matrix3d &R_WI);

  static bool EstimateExtrinsicRotation(CircularBuffer<PairTimeLaserTransform> &all_laser_transforms,
                                        Transform &transform_lb);

};

}

#endif //LIO_IMUINITIALIZER_H_

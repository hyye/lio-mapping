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
// Created by hyye on 5/12/18.
//

#ifndef LIO_LOADVIRTUAL_H_
#define LIO_LOADVIRTUAL_H_

#include <Eigen/Eigen>

#include "utils/CircularBuffer.h"

namespace lio_test {

struct MotionData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double timestamp;
  Eigen::Matrix3d Rwb;
  Eigen::Vector3d twb;
  Eigen::Vector3d imu_acc;
  Eigen::Vector3d imu_gyro;

  Eigen::Vector3d imu_gyro_bias;
  Eigen::Vector3d imu_acc_bias;

  Eigen::Vector3d imu_velocity;
};

void LoadPoseVel(std::string filename, std::vector<MotionData> &pose) {

  std::ifstream f;
  f.open(filename.c_str());

  if (!f.is_open()) {
    std::cerr << " can't open LoadFeatures file " << std::endl;
    return;
  }

  while (!f.eof()) {

    std::string s;
    std::getline(f, s);

    if (!s.empty()) {
      std::stringstream ss;
      ss << s;

      MotionData data;
      double time;
      Eigen::Quaterniond q;
      Eigen::Vector3d t;
      Eigen::Vector3d gyro;
      Eigen::Vector3d acc;

      Eigen::Vector3d vel;
      Eigen::Vector3d acc_bias;
      Eigen::Vector3d gyro_bias;

      ss >> time;
      ss >> q.w();
      ss >> q.x();
      ss >> q.y();
      ss >> q.z();
      ss >> t(0);
      ss >> t(1);
      ss >> t(2);
      ss >> vel(0);
      ss >> vel(1);
      ss >> vel(2);
      ss >> gyro(0);
      ss >> gyro(1);
      ss >> gyro(2);
      ss >> acc(0);
      ss >> acc(1);
      ss >> acc(2);
      ss >> acc_bias(0);
      ss >> acc_bias(1);
      ss >> acc_bias(2);
      ss >> gyro_bias(0);
      ss >> gyro_bias(1);
      ss >> gyro_bias(2);

      data.timestamp = time;
      data.imu_gyro = gyro;
      data.imu_acc = acc;
      data.twb = t;
      data.Rwb = Eigen::Matrix3d(q);

      data.imu_velocity = vel;
      data.imu_acc_bias = acc_bias;
      data.imu_gyro_bias = gyro_bias;

      pose.push_back(data);

    }
  }
}

}

#endif //LIO_LOADVIRTUAL_H_

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
// Created by hyye on 3/27/18.
//


#include <gtest/gtest.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "imu_processor/Estimator.h"
#include "utils/TicToc.h"

#include "utils/CircularBuffer.h"

using namespace lio;
using namespace std;
using namespace mathutils;

TEST(CirclurBufferTest, ForLoopTest) {

  CircularBuffer<shared_ptr<IntegrationBase> > pre_integrations{5 + 1};

  for (int i = 0; i < 10; ++i) {
    Eigen::Vector3d acc_last_(i, i, i);
    Eigen::Vector3d gyr_last_(i + 1, i + 1, i + 1);
    Eigen::Vector3d Bas_(i + 2, i + 2, i + 2);
    Eigen::Vector3d Bgs_(i + 3, i + 3, i + 3);
    pre_integrations.push(std::make_shared<IntegrationBase>(IntegrationBase(acc_last_, gyr_last_, Bas_, Bgs_)));
  }

  for (int i = 0; i < pre_integrations.size(); ++i) {
    DLOG(INFO) << pre_integrations[i]->acc0_.transpose();
  }

}

TEST(CirclurBufferTest, SharedPtrTest) {

//  CircularBuffer<shared_ptr<IntegrationBase> > pre_integrations = CircularBuffer<shared_ptr<IntegrationBase> >(5 + 1);
  CircularBuffer<shared_ptr<IntegrationBase> > pre_integrations{5 + 1};

  for (int i = 0; i < 10; ++i) {
    Eigen::Vector3d acc_last_(i, i, i);
    Eigen::Vector3d gyr_last_(i + 1, i + 1, i + 1);
    Eigen::Vector3d Bas_(i + 2, i + 2, i + 2);
    Eigen::Vector3d Bgs_(i + 3, i + 3, i + 3);
    pre_integrations.push(std::make_shared<IntegrationBase>(IntegrationBase(acc_last_, gyr_last_, Bas_, Bgs_)));
  }

  DLOG(INFO) << pre_integrations.size();
  DLOG(INFO) << pre_integrations[0]->acc0_.transpose();
  DLOG(INFO) << pre_integrations[pre_integrations.size() - 1]->acc0_.transpose();

  for (int j = 0; j < 5; ++j) {
    pre_integrations[j].reset();
  }

  for (int k = 0; k < pre_integrations.size() * 2; ++k) {
    DLOG(INFO) << (pre_integrations[k] != nullptr ? "not nullptr" : "nullptr");
  }

  CircularBuffer<vector<Eigen::Vector3d> > vec_buf{5 + 1};

  for (int l = 0; l < 6; ++l) {
    vec_buf.push(vector<Eigen::Vector3d>());
  }

  for (int l = 0; l < 6; ++l) {
    DLOG(INFO) << vec_buf[l].size();
  }

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_alsologtostderr = true;

  return RUN_ALL_TESTS();
}
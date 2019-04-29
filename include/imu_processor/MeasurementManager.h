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

#ifndef LIO_MEASUREMENTMANAGER_H_
#define LIO_MEASUREMENTMANAGER_H_

#include <cstdlib>
#include <queue>
#include <vector>
#include <map>
#include <string>
#include <mutex>
#include <condition_variable>

#include <glog/logging.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "utils/CircularBuffer.h"
#include "utils/Twist.h"
#include "utils/common_ros.h"
#include "utils/TicToc.h"
#include "utils/math_utils.h"
#include "utils/geometry_utils.h"
#include "3rdparty/sophus/se3.hpp"

namespace lio {

using namespace std;
using namespace mathutils;
using namespace geometryutils;
typedef Twist<float> Transform;
typedef Sophus::SO3f SO3f;
typedef Sophus::SO3d SO3d;
typedef nav_msgs::OdometryConstPtr OdomMsgConstPtr;
typedef sensor_msgs::PointCloud2ConstPtr CompactDataConstPtr;
typedef sensor_msgs::ImuConstPtr ImuMsgConstPtr;
//typedef vector<pair<vector<ImuMsgConstPtr>, OdomMsgConstPtr> > PairMeasurements;
typedef pair<vector<ImuMsgConstPtr>, CompactDataConstPtr> PairMeasurement;
typedef vector<PairMeasurement> PairMeasurements;

struct MeasurementManagerConfig {
  string imu_topic = "/imu/data";
  string laser_topic = "/velodyne_points";
  string laser_odom_topic = "/aft_mapped_to_init"; // NOTE: Check if the time is too long
  string compact_data_topic = "/compact_data"; // NOTE: Check if the time is too long
  double msg_time_delay = 0;
  bool enable_imu = true;
};

class MeasurementManager {

 public:
  virtual void SetupRos(ros::NodeHandle &nh);
  void ImuHandler(const sensor_msgs::ImuConstPtr &raw_imu_msg);
  void LaserOdomHandler(const nav_msgs::OdometryConstPtr &laser_odom_msg);
  void CompactDataHandler(const sensor_msgs::PointCloud2ConstPtr &compact_data_msg);

  PairMeasurements GetMeasurements();

 protected:
  ros::NodeHandle nh_;
  TicToc mm_tic_toc_;
  MeasurementManagerConfig mm_config_;

  mutex buf_mutex_;
  mutex state_mutex_;
  mutex thread_mutex_;
  std::condition_variable con_;
  queue<ImuMsgConstPtr> imu_buf_;
  queue<OdomMsgConstPtr> laser_odom_buf_;
  queue<CompactDataConstPtr> compact_data_buf_;

  double imu_last_time_ = -1;
  double curr_time_ = -1;

  ros::Subscriber sub_imu_;
  ros::Subscriber sub_laser_odom_;
  ros::Subscriber sub_compact_data_;

  bool is_ros_setup_ = false;

};

}

#endif //LIO_MEASUREMENTMANAGER_H_

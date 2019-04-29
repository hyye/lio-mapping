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
// Created by hyye on 3/15/18.
//

#include <gtest/gtest.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Quaternion.h>

#include "point_processor/PointProcessor.h"
#include "utils/TicToc.h"

using namespace lio;
using namespace std;
using namespace mathutils;


int main(int argc, char **argv) {

  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  ros::init(argc, argv, "point_processor");

  ros::NodeHandle nh("~");

  int sensor_type;
  double rad_diff;
  bool infer_start_ori;
  nh.param("sensor_type", sensor_type, 16);
  nh.param("rad_diff", rad_diff, 0.2);
  nh.param("infer_start_ori", infer_start_ori, false);

  PointProcessor processor; // Default sensor_type is 16

  if (sensor_type == 32) {
    processor = PointProcessor(-30.67f, 10.67f, 32);
  } else if (sensor_type == 64) {
    processor = PointProcessor(-24.9f, 2, 64);
  } else if (sensor_type == 320) {
    processor = PointProcessor(-25, 15, 32, true);
  }

  PointProcessorConfig config;
  config.rad_diff = rad_diff;
  config.infer_start_ori_ = infer_start_ori;
  processor.SetupConfig(config);

  LOG(INFO) << "Sensor type: " << processor.laser_scans.size();

  processor.SetupRos(nh);

  ros::Rate r(100);
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
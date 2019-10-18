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
// Created by hyye on 5/31/18.
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <pcl/io/pcd_io.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>

#include <thread>

#include <geometry_msgs/Quaternion.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "imu_processor/Estimator.h"
#include "point_processor/PointOdometry.h"
#include "utils/TicToc.h"
#include "utils/YamlLoader.h"

using namespace lio;
using namespace std;
using namespace mathutils;

static ros::NodeHandlePtr nh_ptr;

static std::string config_file = "";

void Run() {

  DLOG(INFO) << "config_file: " << config_file;

  YamlLoader yaml_loader(config_file);

  MeasurementManagerConfig mm_config = yaml_loader.mm_config;
  EstimatorConfig estimator_config = yaml_loader.estimator_config;

  Estimator estimator(estimator_config, mm_config);
  estimator.SetupRos(*nh_ptr);

  PointOdometry odometry(mm_config.scan_period, mm_config.odom_io);
  odometry.SetupRos(*nh_ptr);
  odometry.Reset();

  thread odom(&PointOdometry::Spin, &odometry);

  thread measurement_manager(&Estimator::ProcessEstimation, &estimator);

  if (estimator_config.pcl_viewer) {
    boost::thread visualizer(boost::bind(&PlaneNormalVisualizer::Spin, &(estimator.normal_vis)));
  }

//  while (!estimator.normal_vis.viewer->wasStopped()) {
//    {
//      boost::mutex::scoped_lock lk(estimator.normal_vis.m);
//      DLOG(INFO) << ">>>>>>> spin <<<<<<<";
//      estimator.normal_vis.viewer->spinOnce(100);
//    }
//    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
//  }

  ros::Rate r(1000);

  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }

}

int main(int argc, char **argv) {

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  ros::init(argc, argv, "test_measurement_manager");
  {
    ros::NodeHandle nh("~");
    nh_ptr = boost::make_shared<ros::NodeHandle>(nh);
  }

  nh_ptr->param("config_file", config_file, std::string(""));
  FLAGS_alsologtostderr = true;

  Run();

  return 0;
}

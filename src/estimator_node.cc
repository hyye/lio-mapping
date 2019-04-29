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

using namespace lio;
using namespace std;
using namespace mathutils;

static ros::NodeHandlePtr nh_ptr;

static std::string config_file = "/home/hyye/dev_ws/src/lio/config/test_config.yaml";

void Run() {

  DLOG(INFO) << "config_file: " << config_file;

  MeasurementManagerConfig mm_config;
  EstimatorConfig estimator_config;
  cv::FileStorage fs_settings(config_file, cv::FileStorage::READ);

  {
    int tmp_int;
    double tmp_double;
    estimator_config.min_match_sq_dis = fs_settings["min_match_sq_dis"];
    estimator_config.min_plane_dis = fs_settings["min_plane_dis"];

    estimator_config.corner_filter_size = fs_settings["corner_filter_size"];
    estimator_config.surf_filter_size = fs_settings["surf_filter_size"];
    estimator_config.map_filter_size = fs_settings["map_filter_size"];

    tmp_int = fs_settings["window_size"];
    estimator_config.window_size = size_t(tmp_int);

    tmp_int = fs_settings["opt_window_size"];
    estimator_config.opt_window_size = size_t(tmp_int);
    estimator_config.init_window_factor = fs_settings["init_window_factor"];

    tmp_int = fs_settings["estimate_extrinsic"];
    estimator_config.estimate_extrinsic = tmp_int;

    tmp_int = fs_settings["opt_extrinsic"];
    estimator_config.opt_extrinsic = (tmp_int > 0);

    cv::Mat cv_R, cv_T;
    fs_settings["extrinsic_rotation"] >> cv_R;
    fs_settings["extrinsic_translation"] >> cv_T;
    Eigen::Matrix3d eigen_R;
    Eigen::Vector3d eigen_T;
    cv::cv2eigen(cv_R, eigen_R);
    cv::cv2eigen(cv_T, eigen_T);

    estimator_config.transform_lb = Transform{Eigen::Quaternionf(eigen_R.cast<float>()), eigen_T.cast<float>()};

    tmp_int = fs_settings["run_optimization"];
    estimator_config.run_optimization = (tmp_int > 0);
    tmp_int = fs_settings["update_laser_imu"];
    estimator_config.update_laser_imu = (tmp_int > 0);
    tmp_int = fs_settings["gravity_fix"];
    estimator_config.gravity_fix = (tmp_int > 0);
    tmp_int = fs_settings["plane_projection_factor"];
    estimator_config.plane_projection_factor = (tmp_int > 0);
    tmp_int = fs_settings["imu_factor"];
    estimator_config.imu_factor = (tmp_int > 0);
    tmp_int = fs_settings["point_distance_factor"];
    estimator_config.point_distance_factor = (tmp_int > 0);
    tmp_int = fs_settings["prior_factor"];
    estimator_config.prior_factor = (tmp_int > 0);
    tmp_int = fs_settings["marginalization_factor"];
    estimator_config.marginalization_factor = (tmp_int > 0);

    tmp_int = fs_settings["pcl_viewer"];
    estimator_config.pcl_viewer = (tmp_int > 0);

    tmp_int = fs_settings["enable_deskew"];
    estimator_config.enable_deskew = (tmp_int > 0);
    tmp_int = fs_settings["cutoff_deskew"];
    estimator_config.cutoff_deskew = (tmp_int > 0);

    tmp_int = fs_settings["keep_features"];
    estimator_config.keep_features = (tmp_int > 0);

    tmp_double = fs_settings["acc_n"];
    estimator_config.pim_config.acc_n = tmp_double;
    tmp_double = fs_settings["gyr_n"];
    estimator_config.pim_config.gyr_n = tmp_double;
    tmp_double = fs_settings["acc_w"];
    estimator_config.pim_config.acc_w = tmp_double;
    tmp_double = fs_settings["gyr_w"];
    estimator_config.pim_config.gyr_w = tmp_double;
    tmp_double = fs_settings["g_norm"];
    estimator_config.pim_config.g_norm = tmp_double;

    tmp_double = fs_settings["msg_time_delay"];
    mm_config.msg_time_delay = tmp_double;
  }

  Estimator estimator(estimator_config);
  estimator.SetupRos(*nh_ptr);

  int odom_io = fs_settings["odom_io"];

  PointOdometry odometry(0.1, odom_io);
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

  nh_ptr->param("config_file", config_file, std::string("/home/hyye/dev_ws/src/lio/config/test_config.yaml"));
  FLAGS_alsologtostderr = true;

  Run();

  return 0;
}

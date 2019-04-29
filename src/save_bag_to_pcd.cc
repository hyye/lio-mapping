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

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>

#include <thread>
#include <vector>

#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "utils/TicToc.h"

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using namespace std;

DEFINE_string(input_filename, "", "the name of input bag file");
DEFINE_string(output_filename, "", "the name of output pcd file");
DEFINE_string(odom_name, "/aft_mapped_to_init", "the name of the odom");
DEFINE_string(laser_name, "/cloud_registered", "the name of the laser");

static bool laser_available = false;
static bool odom_available = false;
static sensor_msgs::PointCloud2 tmp_points_msg;
static nav_msgs::Odometry tmp_odom_msg;
static pcl::PointCloud<pcl::PointXYZI> map_points, tmp_points;
static int frame_count = 0;

void LaserHandler(const sensor_msgs::PointCloud2ConstPtr &laser_msg) {
  tmp_points_msg = *laser_msg;
  tmp_points.clear();
  pcl::fromROSMsg(tmp_points_msg, tmp_points);
  laser_available = true;
}

void OdomHandler(const nav_msgs::Odometry::ConstPtr &odom_msg) {
  tmp_odom_msg = *odom_msg;
  odom_available = true;
}

void ProcessData() {
  if (laser_available && odom_available && (tmp_odom_msg.header.stamp - tmp_points_msg.header.stamp).toSec() < 0.005) {
    odom_available = false;
    laser_available = false;
  } else {
    return;
  }

  Eigen::Affine3f transform_to_init;
  transform_to_init.setIdentity();

//  transform_to_init.translation() =
//      Eigen::Vector3f(tmp_odom_msg.pose.pose.position.x,
//                      tmp_odom_msg.pose.pose.position.y,
//                      tmp_odom_msg.pose.pose.position.z);
//
//  Eigen::Quaternionf tmp_quat;
//  tmp_quat.x() = tmp_odom_msg.pose.pose.orientation.x;
//  tmp_quat.y() = tmp_odom_msg.pose.pose.orientation.y;
//  tmp_quat.z() = tmp_odom_msg.pose.pose.orientation.z;
//  tmp_quat.w() = tmp_odom_msg.pose.pose.orientation.w;
//
//  transform_to_init.linear() = tmp_quat.normalized().toRotationMatrix();

  pcl::PointCloud<pcl::PointXYZI> transformed_cloud;

  pcl::transformPointCloud(tmp_points, transformed_cloud, transform_to_init);

  map_points += transformed_cloud;

  DLOG(INFO) << "transform_to_init: " << std::endl << transform_to_init.matrix();
  DLOG(INFO) << "merged: " << frame_count << "th frame";
  ++frame_count;

}

// Load bag
void LoadBag(const std::string &input_filename, const std::string &output_filename) {
  rosbag::Bag bag;
  bag.open(input_filename, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string(FLAGS_odom_name));
  topics.push_back(std::string(FLAGS_laser_name));

  rosbag::View view(bag, rosbag::TopicQuery(topics));

      foreach(rosbag::MessageInstance const m, view) {
          sensor_msgs::PointCloud2ConstPtr ptc = m.instantiate<sensor_msgs::PointCloud2>();
          nav_msgs::OdometryConstPtr odom = m.instantiate<nav_msgs::Odometry>();

          if (odom != NULL) {
            OdomHandler(odom);
            ProcessData();
          }

          if (ptc != NULL) {
            LaserHandler(ptc);
            ProcessData();
          }

          if (!ros::ok()) {
            break;
          }

        }

  bag.close();

  if (map_points.size() > 0) {
    pcl::PCDWriter pcd_writer;
    DLOG(INFO) << "saving...";
    pcd_writer.writeBinary(output_filename, map_points);
    DLOG(INFO) << "saved as " << output_filename;
  } else {
    LOG(FATAL) << "no points saved";
  }
}

int main(int argc, char **argv) {

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_alsologtostderr = true;

  ros::init(argc, argv, "map_builder");

  ros::NodeHandle nh("~");;

  std::string input_filename = FLAGS_input_filename;
  std::string output_filename = FLAGS_output_filename;

  if (input_filename != "" && output_filename != "") {
    LoadBag(input_filename, output_filename);
  } else {
    LOG(FATAL) << "empty file_name";
  }

  return 0;
}


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

static ros::Publisher test_pub;
static ros::NodeHandlePtr nh_ptr;

DEFINE_int64(sensor_type, 16, "Sensor type - number of scans (default 16)");

TEST(PointProcessorTest, AngleTest) {

  EXPECT_DOUBLE_EQ(NormalizeRad(-3.4 - 2 * M_PI), -3.4 + 2 * M_PI);
  EXPECT_DOUBLE_EQ(NormalizeRad(3.4 + 2 * M_PI), 3.4 - 2 * M_PI);

  EXPECT_DOUBLE_EQ(NormalizeDeg(-190 - 360), -190 + 360);
  EXPECT_DOUBLE_EQ(NormalizeDeg(190 + 360), 190 - 360);

}

TEST(PointProcessorTest, SubscribeTest) {

  PointProcessor processor;

  if (FLAGS_sensor_type == 32) {
    processor = PointProcessor(-30.67f, 10.67f, 32);
  } else if (FLAGS_sensor_type == 64) {
    processor = PointProcessor(-24.9f, 2, 64);
  }

  DLOG(INFO) << "Sensor type: " << processor.laser_scans.size();

  processor.SetupRos(*nh_ptr);

  ros::Rate r(100);
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }


}

TEST(PointProcessorTest, ObjectTest) {

  pcl::PointXYZI p;
//  PointProcessor pp(-24.8f, 2.0f, 64);
  PointProcessor pp;

  pcl::PCDReader pcd_reader;

  PointCloud test_pc;
//  pcd_reader.read("/mnt/HDD/Datasets/Lidar/sz_tests/OneDrive-2018-03-07/output/1520430666.350994142.pcd", test_pc);
  pcd_reader.read("/mnt/HDD/Datasets/Lidar/Lidar_IMU/output/1512528800.091783000.pcd", test_pc);
//  pcd_reader.read("/home/hyye/Desktop/output/kitti_data/1317013369.682684898.pcd", test_pc);

  PointCloudConstPtr test_pc_ptr(new PointCloud(test_pc));

  pp.SetDeskew(false);
  pp.SetInputCloud(test_pc_ptr);
  pp.PointToRing();
  pp.ExtractFeaturePoints();
  pp.SetupRos(*nh_ptr);

  PointCloud test_pc_out;
  for (const auto &ring_scan : pp.laser_scans) {
    for (const auto &p : *ring_scan) {
      // cout << p.intensity << " ";
    }
    // cout << endl;
    test_pc_out += (*ring_scan);
  }

  sensor_msgs::PointCloud2 pc_msg;
  pcl::toROSMsg(test_pc_out, pc_msg);
  pc_msg.header.frame_id = "map";
  ros::Rate r(10);
  while (ros::ok()) {
//    test_pub.publish(pc_msg);
    PublishCloudMsg(test_pub, test_pc_out, ros::Time::now(), "map");
    pp.PublishResults();
    ros::spinOnce();
    r.sleep();
  }

  return;

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  ros::init(argc, argv, "test_point_processor");
  {
    ros::NodeHandle nh("~");
    nh_ptr = boost::make_shared<ros::NodeHandle>(nh);
    test_pub = nh.advertise<sensor_msgs::PointCloud2>("test_points", 5);
  }

  FLAGS_alsologtostderr = true;

  return RUN_ALL_TESTS();
}
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
// Created by hyye on 3/25/18.
//

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#ifndef LIO_POINTMAPPING_H_
#define LIO_POINTMAPPING_H_

#include <glog/logging.h>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/features/normal_3d.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <map>

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
typedef pcl::PointXYZI PointT;
typedef typename pcl::PointCloud<PointT> PointCloud;
typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
typedef Twist<float> Transform;
typedef Sophus::SO3f SO3;

struct CubeCenter {
  int laser_cloud_cen_length;
  int laser_cloud_cen_width;
  int laser_cloud_cen_height;

  friend std::ostream &operator<<(std::ostream &os, const CubeCenter &cube_center) {
    os << cube_center.laser_cloud_cen_length << " " << cube_center.laser_cloud_cen_width << " "
       << cube_center.laser_cloud_cen_height;
    return os;
  }
};

class PointMapping {

 public:
  PointMapping(float scan_period = 0.1,
               size_t num_max_iterations = 10);

  void SetupRos(ros::NodeHandle &nh, bool enable_sub = true);
  void Reset();

  void LaserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &corner_points_sharp_msg);
  void LaserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &corner_points_less_sharp_msg);
  void LaserFullCloudHandler(const sensor_msgs::PointCloud2ConstPtr &full_cloud_msg);
  void LaserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laser_odom_msg);
  void CompactDataHandler(const sensor_msgs::PointCloud2ConstPtr &compact_data_msg);

  void SetInitFlag(bool set_init);

  bool HasNewData();
  void OptimizeTransformTobeMapped();

  void TransformAssociateToMap();
  void TransformUpdate();
  void PointAssociateToMap(const PointT &pi, PointT &po,
                           const Transform &transform_tobe_mapped);
  void PointAssociateTobeMapped(const PointT &pi, PointT &po,
                                const Transform &transform_tobe_mapped);

  void UpdateMapDatabase(PointCloudPtr laser_cloud_corner_stack_downsampled,
                         PointCloudPtr laser_cloud_surf_stack_downsampled,
                         std::vector<size_t> laser_cloud_valid_idx,
                         const Transform &transform_tobe_mapped,
                         const CubeCenter &cube_center);

  void PublishResults();

  void Process();

 protected:

  size_t ToIndex(int i, int j, int k) const {
    return i + laser_cloud_length_ * j + laser_cloud_length_ * laser_cloud_width_ * k;
  }

  void FromIndex(const size_t &index, int &i, int &j, int &k) {
    int residual = index % (laser_cloud_length_ * laser_cloud_width_);
    k = index / (laser_cloud_length_ * laser_cloud_width_);
    j = residual / laser_cloud_length_;
    i = residual % laser_cloud_length_;
  }

  TicToc tic_toc_;

  float scan_period_;
  float time_factor_;
  long frame_count_;        ///< number of processed frames
  long map_frame_count_;
  const int num_stack_frames_;
  const int num_map_frames_;

  bool system_inited_ = false;
  size_t num_max_iterations_ = 10;

  double delta_r_abort_;
  double delta_t_abort_;

  int laser_cloud_cen_length_;
  int laser_cloud_cen_width_;
  int laser_cloud_cen_height_;
  const size_t laser_cloud_length_;
  const size_t laser_cloud_width_;
  const size_t laser_cloud_height_;
  const size_t laser_cloud_num_;

  PointCloudPtr laser_cloud_corner_last_;   ///< last corner points cloud
  PointCloudPtr laser_cloud_surf_last_;     ///< last surface points cloud
  PointCloudPtr full_cloud_;      ///< last full resolution cloud

  PointCloudPtr laser_cloud_corner_stack_;
  PointCloudPtr laser_cloud_surf_stack_;
  PointCloudPtr laser_cloud_corner_stack_downsampled_;  ///< down sampled
  PointCloudPtr laser_cloud_surf_stack_downsampled_;    ///< down sampled

  PointCloudPtr laser_cloud_surround_;
  PointCloudPtr laser_cloud_surround_downsampled_;     ///< down sampled
  PointCloudPtr laser_cloud_corner_from_map_;
  PointCloudPtr laser_cloud_surf_from_map_;

  std::vector<PointCloudPtr> laser_cloud_corner_array_;
  std::vector<PointCloudPtr> laser_cloud_surf_array_;
  std::vector<PointCloudPtr> laser_cloud_corner_downsampled_array_;  ///< down sampled
  std::vector<PointCloudPtr> laser_cloud_surf_downsampled_array_;    ///< down sampled

  std::vector<size_t> laser_cloud_valid_idx_;
  std::vector<size_t> laser_cloud_surround_idx_;

  ros::Time time_laser_cloud_corner_last_;   ///< time of current last corner cloud
  ros::Time time_laser_cloud_surf_last_;     ///< time of current last surface cloud
  ros::Time time_laser_full_cloud_;      ///< time of current full resolution cloud
  ros::Time time_laser_odometry_;          ///< time of current laser odometry

  bool new_laser_cloud_corner_last_;  ///< flag if a new last corner cloud has been received
  bool new_laser_cloud_surf_last_;    ///< flag if a new last surface cloud has been received
  bool new_laser_full_cloud_;     ///< flag if a new full resolution cloud has been received
  bool new_laser_odometry_;         ///< flag if a new laser odometry has been received

  Transform transform_sum_;
  Transform transform_tobe_mapped_;
  Transform transform_bef_mapped_;
  Transform transform_aft_mapped_;

  pcl::VoxelGrid<pcl::PointXYZI> down_size_filter_corner_;   ///< voxel filter for down sizing corner clouds
  pcl::VoxelGrid<pcl::PointXYZI> down_size_filter_surf_;     ///< voxel filter for down sizing surface clouds
  pcl::VoxelGrid<pcl::PointXYZI> down_size_filter_map_;      ///< voxel filter for down sizing accumulated map

  nav_msgs::Odometry odom_aft_mapped_;      ///< mapping odometry message
  tf::StampedTransform aft_mapped_trans_;   ///< mapping odometry transformation

  ros::Publisher pub_laser_cloud_surround_;    ///< map cloud message publisher
  ros::Publisher pub_full_cloud_;     ///< current full resolution cloud message publisher
  ros::Publisher pub_odom_aft_mapped_;         ///< mapping odometry publisher
  tf::TransformBroadcaster tf_broadcaster_;  ///< mapping odometry transform broadcaster

  ros::Subscriber sub_laser_cloud_corner_last_;   ///< last corner cloud message subscriber
  ros::Subscriber sub_laser_cloud_surf_last_;     ///< last surface cloud message subscriber
  ros::Subscriber sub_laser_full_cloud_;      ///< full resolution cloud message subscriber
  ros::Subscriber sub_laser_odometry_;          ///< laser odometry message subscriber
  ros::Subscriber sub_compact_data_;          ///< laser odometry message subscriber

  bool is_ros_setup_ = false;
  bool compact_data_ = false;
  bool imu_inited_ = false;

  multimap<float, pair<PointT, PointT>, greater<float> > score_point_coeff_;

  float min_match_sq_dis_ = 1.0;
  float min_plane_dis_ = 0.2;
  PointT point_on_z_axis_;

  Eigen::Matrix<float, 6, 6> matP_;

};

}

#endif //LIO_POINTMAPPING_H_

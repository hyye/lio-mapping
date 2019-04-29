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

#include "point_processor/PointMapping.h"

namespace lio {

PointMapping::PointMapping(float scan_period,
                           size_t num_max_iterations)
    : scan_period_(scan_period),
      num_stack_frames_(1),
      num_map_frames_(5),
      frame_count_(0),
      map_frame_count_(0),
      num_max_iterations_(num_max_iterations),
      delta_r_abort_(0.05),
      delta_t_abort_(0.05),
      laser_cloud_cen_length_(10),
      laser_cloud_cen_width_(10),
      laser_cloud_cen_height_(5),
      laser_cloud_length_(21),
      laser_cloud_width_(21),
      laser_cloud_height_(11),
      laser_cloud_num_(laser_cloud_length_ * laser_cloud_width_ * laser_cloud_height_),
      laser_cloud_corner_last_(new PointCloud()),
      laser_cloud_surf_last_(new PointCloud()),
      full_cloud_(new PointCloud()),
      laser_cloud_corner_stack_(new PointCloud()),
      laser_cloud_surf_stack_(new PointCloud()),
      laser_cloud_corner_stack_downsampled_(new PointCloud()),
      laser_cloud_surf_stack_downsampled_(new PointCloud()),
      laser_cloud_surround_(new PointCloud()),
      laser_cloud_surround_downsampled_(new PointCloud()),
      laser_cloud_corner_from_map_(new PointCloud()),
      laser_cloud_surf_from_map_(new PointCloud()) {
  // initialize mapping odometry and odometry tf messages
  odom_aft_mapped_.header.frame_id = "/camera_init";
  odom_aft_mapped_.child_frame_id = "/aft_mapped";

  aft_mapped_trans_.frame_id_ = "/camera_init";
  aft_mapped_trans_.child_frame_id_ = "/aft_mapped";

  // initialize frame counter
  frame_count_ = num_stack_frames_ - 1;
  map_frame_count_ = num_map_frames_ - 1;

  // setup cloud vectors
  laser_cloud_corner_array_.resize(laser_cloud_num_);
  laser_cloud_surf_array_.resize(laser_cloud_num_);
  laser_cloud_corner_downsampled_array_.resize(laser_cloud_num_);
  laser_cloud_surf_downsampled_array_.resize(laser_cloud_num_);

  // NOTE: stored in array
  for (size_t i = 0; i < laser_cloud_num_; i++) {
    laser_cloud_corner_array_[i].reset(new PointCloud());
    laser_cloud_surf_array_[i].reset(new PointCloud());
    laser_cloud_corner_downsampled_array_[i].reset(new PointCloud());
    laser_cloud_surf_downsampled_array_[i].reset(new PointCloud());
  }

  // setup down size filters
  down_size_filter_corner_.setLeafSize(0.2, 0.2, 0.2);
  down_size_filter_surf_.setLeafSize(0.4, 0.4, 0.4);
  down_size_filter_map_.setLeafSize(0.6, 0.6, 0.6);

  // setup down size filters indoor
//  down_size_filter_corner_.setLeafSize(0.1, 0.1, 0.1);
//  down_size_filter_surf_.setLeafSize(0.2, 0.2, 0.2);
//  down_size_filter_map_.setLeafSize(0.2, 0.2, 0.2);
} // PointMapping

void PointMapping::SetupRos(ros::NodeHandle &nh, bool enable_sub) {

  is_ros_setup_ = true;

  nh.param("compact_data", compact_data_, true);

  // advertise laser odometry topics
  pub_laser_cloud_surround_ = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 2);
  pub_full_cloud_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 2);
  pub_odom_aft_mapped_ = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 5);

  /// for test
//  pub_diff_odometry_ = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_last", 5);

  if (enable_sub) {

    if (compact_data_) {
      sub_compact_data_ =
          nh.subscribe<sensor_msgs::PointCloud2>("/compact_data", 2, &PointMapping::CompactDataHandler, this);
    } else {
      // subscribe to scan registration topics
      sub_laser_cloud_corner_last_ = nh.subscribe<sensor_msgs::PointCloud2>
          ("/laser_cloud_corner_last", 2, &PointMapping::LaserCloudCornerLastHandler, this);

      sub_laser_cloud_surf_last_ = nh.subscribe<sensor_msgs::PointCloud2>
          ("/laser_cloud_surf_last", 2, &PointMapping::LaserCloudSurfLastHandler, this);

      sub_laser_full_cloud_ = nh.subscribe<sensor_msgs::PointCloud2>
          ("/full_odom_cloud", 2, &PointMapping::LaserFullCloudHandler, this);

      sub_laser_odometry_ = nh.subscribe<nav_msgs::Odometry>
          ("/laser_odom_to_init", 2, &PointMapping::LaserOdometryHandler, this);

//  sub_imu_trans_ = node.subscribe<sensor_msgs::PointCloud2>
//      ("/imu_trans", 5, &LaserOdometry::ImuTransHandler, this);
    }
  }

} // SetupRos

void PointMapping::CompactDataHandler(const sensor_msgs::PointCloud2ConstPtr &compact_data_msg) {

  TicToc tic_toc_decoder;

  PointCloud compact_points;
  pcl::fromROSMsg(*compact_data_msg, compact_points);

  size_t compact_point_size = compact_points.size();

  if (compact_point_size < 4) {
    LOG(ERROR) << "compact_points not enough: " << compact_points.size();
    return;
  }

  PointT compact_point;
  compact_point = compact_points[2];
  int corner_size = int(compact_point.x);
  int surf_size = int(compact_point.y);
  int full_size = int(compact_point.z);

  if ((3 + corner_size + surf_size + full_size) != compact_point_size) {
    LOG(ERROR) << "compact data error: 3+" << corner_size << "+" << surf_size << "+" << full_size << " != "
               << compact_point_size;
    return;
  }

  {
    compact_point = compact_points[0];
    transform_sum_.pos.x() = compact_point.x;
    transform_sum_.pos.y() = compact_point.y;
    transform_sum_.pos.z() = compact_point.z;
    compact_point = compact_points[1];
    transform_sum_.rot.x() = compact_point.x;
    transform_sum_.rot.y() = compact_point.y;
    transform_sum_.rot.z() = compact_point.z;
    transform_sum_.rot.w() = compact_point.intensity;
  }

  {
    laser_cloud_corner_last_->clear();
    laser_cloud_surf_last_->clear();
    full_cloud_->clear();

    // TODO: use vector map instead?
    for (size_t i = 3; i < compact_point_size; ++i) {
      const PointT &p = compact_points[i];
      if (i < 3 + corner_size) {
        laser_cloud_corner_last_->push_back(p);
      } else if (i >= 3 + corner_size && i < 3 + corner_size + surf_size) {
        laser_cloud_surf_last_->push_back(p);
      } else {
        full_cloud_->push_back(p);
      }
    }
  }

  time_laser_cloud_corner_last_ = compact_data_msg->header.stamp;
  time_laser_cloud_surf_last_ = compact_data_msg->header.stamp;
  time_laser_full_cloud_ = compact_data_msg->header.stamp;
  time_laser_odometry_ = compact_data_msg->header.stamp;

  new_laser_cloud_corner_last_ = true;
  new_laser_cloud_surf_last_ = true;
  new_laser_full_cloud_ = true;
  new_laser_odometry_ = true;

  DLOG(INFO) << "decode compact data time: " << tic_toc_decoder.Toc() << " ms";
}

void PointMapping::LaserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &corner_points_sharp_msg) {
  time_laser_cloud_corner_last_ = corner_points_sharp_msg->header.stamp;

  laser_cloud_corner_last_->clear();
  pcl::fromROSMsg(*corner_points_sharp_msg, *laser_cloud_corner_last_);

  new_laser_cloud_corner_last_ = true;
}

void PointMapping::LaserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &corner_points_less_sharp_msg) {
  time_laser_cloud_surf_last_ = corner_points_less_sharp_msg->header.stamp;

  laser_cloud_surf_last_->clear();
  pcl::fromROSMsg(*corner_points_less_sharp_msg, *laser_cloud_surf_last_);

  new_laser_cloud_surf_last_ = true;
}

void PointMapping::LaserFullCloudHandler(const sensor_msgs::PointCloud2ConstPtr &full_cloud_msg) {
  time_laser_full_cloud_ = full_cloud_msg->header.stamp;

  full_cloud_->clear();
  pcl::fromROSMsg(*full_cloud_msg, *full_cloud_);

  new_laser_full_cloud_ = true;
}

void PointMapping::LaserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laser_odom_msg) {
  time_laser_odometry_ = laser_odom_msg->header.stamp;

  geometry_msgs::Quaternion geo_quat = laser_odom_msg->pose.pose.orientation;

  transform_sum_.rot.w() = geo_quat.w;
  transform_sum_.rot.x() = geo_quat.x;
  transform_sum_.rot.y() = geo_quat.y;
  transform_sum_.rot.z() = geo_quat.z;

  transform_sum_.pos.x() = float(laser_odom_msg->pose.pose.position.x);
  transform_sum_.pos.y() = float(laser_odom_msg->pose.pose.position.y);
  transform_sum_.pos.z() = float(laser_odom_msg->pose.pose.position.z);

  new_laser_odometry_ = true;
}

void PointMapping::Reset() {
  new_laser_cloud_corner_last_ = false;
  new_laser_cloud_surf_last_ = false;
  new_laser_full_cloud_ = false;
  new_laser_odometry_ = false;
}

bool PointMapping::HasNewData() {
  return new_laser_cloud_corner_last_ && new_laser_cloud_surf_last_ &&
      new_laser_full_cloud_ && new_laser_odometry_ &&
      fabs((time_laser_cloud_corner_last_ - time_laser_odometry_).toSec()) < 0.005 &&
      fabs((time_laser_cloud_surf_last_ - time_laser_odometry_).toSec()) < 0.005 &&
      fabs((time_laser_full_cloud_ - time_laser_odometry_).toSec()) < 0.005;
}

void PointMapping::SetInitFlag(bool set_init) {
  imu_inited_ = set_init;
}

void PointMapping::PointAssociateToMap(const PointT &pi, PointT &po, const Transform &transform_tobe_mapped) {
  po.x = pi.x;
  po.y = pi.y;
  po.z = pi.z;
  po.intensity = pi.intensity;

  RotatePoint(transform_tobe_mapped.rot, po);

  po.x += transform_tobe_mapped.pos.x();
  po.y += transform_tobe_mapped.pos.y();
  po.z += transform_tobe_mapped.pos.z();
}

void PointMapping::PointAssociateTobeMapped(const PointT &pi, PointT &po, const Transform &transform_tobe_mapped) {
  po.x = pi.x - transform_tobe_mapped.pos.x();
  po.y = pi.y - transform_tobe_mapped.pos.y();
  po.z = pi.z - transform_tobe_mapped.pos.z();
  po.intensity = pi.intensity;

  RotatePoint(transform_tobe_mapped.rot.conjugate(), po);
}

void PointMapping::OptimizeTransformTobeMapped() {

  if (laser_cloud_corner_from_map_->points.size() <= 10 || laser_cloud_surf_from_map_->points.size() <= 100) {
    return;
  }

  PointT point_sel, point_ori, coeff, abs_coeff;

  std::vector<int> point_search_idx(5, 0);
  std::vector<float> point_search_sq_dis(5, 0);

  pcl::KdTreeFLANN<PointT>::Ptr kdtree_corner_from_map(new pcl::KdTreeFLANN<PointT>());
  pcl::KdTreeFLANN<PointT>::Ptr kdtree_surf_from_map(new pcl::KdTreeFLANN<PointT>());

  kdtree_corner_from_map->setInputCloud(laser_cloud_corner_from_map_);
  kdtree_surf_from_map->setInputCloud(laser_cloud_surf_from_map_);

  Eigen::Matrix<float, 5, 3> mat_A0;
  Eigen::Matrix<float, 5, 1> mat_B0;
  Eigen::Vector3f mat_X0;
  Eigen::Matrix3f mat_A1;
  Eigen::Matrix<float, 1, 3> mat_D1;
  Eigen::Matrix3f mat_V1;

  mat_A0.setZero();
  mat_B0.setConstant(-1);
  mat_X0.setZero();

  mat_A1.setZero();
  mat_D1.setZero();
  mat_V1.setZero();

  bool is_degenerate = false;
  matP_.setIdentity();

  size_t laser_cloud_corner_stack_size = laser_cloud_corner_stack_downsampled_->points.size();
  size_t laser_cloud_surf_stack_size = laser_cloud_surf_stack_downsampled_->points.size();

  PointCloud laser_cloud_ori;
  PointCloud coeff_sel;

  PointCloud laser_cloud_ori_spc;
  PointCloud coeff_sel_spc;
  PointCloud abs_coeff_sel_spc;
  score_point_coeff_.clear();

  tic_toc_.Tic();

  for (size_t iter_count = 0; iter_count < num_max_iterations_; ++iter_count) {
    laser_cloud_ori.clear();
    coeff_sel.clear();

    laser_cloud_ori_spc.clear();
    coeff_sel_spc.clear();
    abs_coeff_sel_spc.clear();

    for (int i = 0; i < laser_cloud_corner_stack_size; ++i) {
      point_ori = laser_cloud_corner_stack_downsampled_->points[i];
      PointAssociateToMap(point_ori, point_sel, transform_tobe_mapped_);
      kdtree_corner_from_map->nearestKSearch(point_sel, 5, point_search_idx, point_search_sq_dis);

      if (point_search_sq_dis[4] < min_match_sq_dis_) {
//        Vector3Intl vc(0, 0, 0);
        Eigen::Vector3f vc(0, 0, 0);

        for (int j = 0; j < 5; j++) {
//          vc += Vector3Intl(laser_cloud_corner_from_map_->points[point_search_idx[j]]);
          const PointT &point_sel_tmp = laser_cloud_corner_from_map_->points[point_search_idx[j]];
          vc.x() += point_sel_tmp.x;
          vc.y() += point_sel_tmp.y;
          vc.z() += point_sel_tmp.z;
        }
        vc /= 5.0;

        Eigen::Matrix3f mat_a;
        mat_a.setZero();

        for (int j = 0; j < 5; j++) {
//          Vector3Intl a = Vector3Intl(laser_cloud_corner_from_map_->points[point_search_idx[j]]) - vc;
          const PointT &point_sel_tmp = laser_cloud_corner_from_map_->points[point_search_idx[j]];
          Eigen::Vector3f a;
          a.x() = point_sel_tmp.x - vc.x();
          a.y() = point_sel_tmp.y - vc.y();
          a.z() = point_sel_tmp.z - vc.z();

          mat_a(0, 0) += a.x() * a.x();
          mat_a(1, 0) += a.x() * a.y();
          mat_a(2, 0) += a.x() * a.z();
          mat_a(1, 1) += a.y() * a.y();
          mat_a(2, 1) += a.y() * a.z();
          mat_a(2, 2) += a.z() * a.z();
        }
        mat_A1 = mat_a / 5.0;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esolver(mat_A1);
        mat_D1 = esolver.eigenvalues().real();
        mat_V1 = esolver.eigenvectors().real();

        if (mat_D1(0, 2) > 3 * mat_D1(0, 1)) {

          float x0 = point_sel.x;
          float y0 = point_sel.y;
          float z0 = point_sel.z;
          float x1 = vc.x() + 0.1 * mat_V1(0, 2);
          float y1 = vc.y() + 0.1 * mat_V1(1, 2);
          float z1 = vc.z() + 0.1 * mat_V1(2, 2);
          float x2 = vc.x() - 0.1 * mat_V1(0, 2);
          float y2 = vc.y() - 0.1 * mat_V1(1, 2);
          float z2 = vc.z() - 0.1 * mat_V1(2, 2);

          Eigen::Vector3f X0(x0, y0, z0);
          Eigen::Vector3f X1(x1, y1, z1);
          Eigen::Vector3f X2(x2, y2, z2);

          // NOTE: to make sure on a line with the same as eigen vector

          // NOTE: (P1 - P2) x ((P0 - P1)x(P0 - P2)), point to the point

//          float a012 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1))
//                                * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1))
//                                + ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))
//                                    * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))
//                                + ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))
//                                    * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

//          float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

//          float la = ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1))
//              + (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) / a012 / l12;
//
//          float lb = -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1))
//              - (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;
//
//          float lc = -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))
//              + (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

          Eigen::Vector3f a012_vec = (X0 - X1).cross(X0 - X2);

          Eigen::Vector3f normal_to_point = ((X1 - X2).cross(a012_vec)).normalized();

          float a012 = a012_vec.norm();

          float l12 = (X1 - X2).norm();

          float la = normal_to_point.x();
          float lb = normal_to_point.y();
          float lc = normal_to_point.z();

          float ld2 = a012 / l12;

          float s = 1 - 0.9f * fabs(ld2);

          coeff.x = s * la;
          coeff.y = s * lb;
          coeff.z = s * lc;
          coeff.intensity = s * ld2;

          abs_coeff.x = la;
          abs_coeff.y = lb;
          abs_coeff.z = lc;
          abs_coeff.intensity = (ld2 - normal_to_point.dot(X0));

          bool is_in_laser_fov = false;
          PointT transform_pos;
          transform_pos.x = transform_tobe_mapped_.pos.x();
          transform_pos.y = transform_tobe_mapped_.pos.y();
          transform_pos.z = transform_tobe_mapped_.pos.z();
          float squared_side1 = CalcSquaredDiff(transform_pos, point_sel);
          float squared_side2 = CalcSquaredDiff(point_on_z_axis_, point_sel);

          float check1 = 100.0f + squared_side1 - squared_side2
              - 10.0f * sqrt(3.0f) * sqrt(squared_side1);

          float check2 = 100.0f + squared_side1 - squared_side2
              + 10.0f * sqrt(3.0f) * sqrt(squared_side1);

          if (check1 < 0 && check2 > 0) { /// within +-60 degree
            is_in_laser_fov = true;
          }

          if (s > 0.1 && is_in_laser_fov) {
            laser_cloud_ori.push_back(point_ori);
            coeff_sel.push_back(coeff);
//            abs_coeff_sel_spc.push_back(abs_coeff);
          }
        }
      }
    }

    for (int i = 0; i < laser_cloud_surf_stack_size; i++) {
      point_ori = laser_cloud_surf_stack_downsampled_->points[i];
      PointAssociateToMap(point_ori, point_sel, transform_tobe_mapped_);

      int num_neighbors = 5;
      kdtree_surf_from_map->nearestKSearch(point_sel, num_neighbors, point_search_idx, point_search_sq_dis);

      if (point_search_sq_dis[num_neighbors - 1] < min_match_sq_dis_) {
        for (int j = 0; j < num_neighbors; j++) {
          mat_A0(j, 0) = laser_cloud_surf_from_map_->points[point_search_idx[j]].x;
          mat_A0(j, 1) = laser_cloud_surf_from_map_->points[point_search_idx[j]].y;
          mat_A0(j, 2) = laser_cloud_surf_from_map_->points[point_search_idx[j]].z;
        }
        mat_X0 = mat_A0.colPivHouseholderQr().solve(mat_B0);

        float pa = mat_X0(0, 0);
        float pb = mat_X0(1, 0);
        float pc = mat_X0(2, 0);
        float pd = 1;

        float ps = sqrt(pa * pa + pb * pb + pc * pc);
        pa /= ps;
        pb /= ps;
        pc /= ps;
        pd /= ps;

        // NOTE: plane as (x y z)*w+1 = 0

        bool planeValid = true;
        for (int j = 0; j < num_neighbors; j++) {
          if (fabs(pa * laser_cloud_surf_from_map_->points[point_search_idx[j]].x +
              pb * laser_cloud_surf_from_map_->points[point_search_idx[j]].y +
              pc * laser_cloud_surf_from_map_->points[point_search_idx[j]].z + pd) > min_plane_dis_) {
            planeValid = false;
            break;
          }
        }

        if (planeValid) {
          float pd2 = pa * point_sel.x + pb * point_sel.y + pc * point_sel.z + pd;

          float s = 1 - 0.9f * fabs(pd2) / sqrt(CalcPointDistance(point_sel));

          if (pd2 > 0) {
            coeff.x = s * pa;
            coeff.y = s * pb;
            coeff.z = s * pc;
            coeff.intensity = s * pd2;

            abs_coeff.x = pa;
            abs_coeff.y = pb;
            abs_coeff.z = pc;
            abs_coeff.intensity = pd;
          } else {
            coeff.x = -s * pa;
            coeff.y = -s * pb;
            coeff.z = -s * pc;
            coeff.intensity = -s * pd2;

            abs_coeff.x = -pa;
            abs_coeff.y = -pb;
            abs_coeff.z = -pc;
            abs_coeff.intensity = -pd;
          }

          bool is_in_laser_fov = false;
          PointT transform_pos;
          transform_pos.x = transform_tobe_mapped_.pos.x();
          transform_pos.y = transform_tobe_mapped_.pos.y();
          transform_pos.z = transform_tobe_mapped_.pos.z();
          float squared_side1 = CalcSquaredDiff(transform_pos, point_sel);
          float squared_side2 = CalcSquaredDiff(point_on_z_axis_, point_sel);

          float check1 = 100.0f + squared_side1 - squared_side2
              - 10.0f * sqrt(3.0f) * sqrt(squared_side1);

          float check2 = 100.0f + squared_side1 - squared_side2
              + 10.0f * sqrt(3.0f) * sqrt(squared_side1);

          if (check1 < 0 && check2 > 0) { /// within +-60 degree
            is_in_laser_fov = true;
          }

          if (s > 0.1 && is_in_laser_fov) {
            laser_cloud_ori.push_back(point_ori);
            coeff_sel.push_back(coeff);

            laser_cloud_ori_spc.push_back(point_ori);
            coeff_sel_spc.push_back(coeff);
            abs_coeff_sel_spc.push_back(abs_coeff);
          }
        }
      }
    }

    size_t laser_cloud_sel_size = laser_cloud_ori.points.size();
    if (laser_cloud_sel_size < 50) {
      continue;
    }

    Eigen::Matrix<float, Eigen::Dynamic, 6> mat_A(laser_cloud_sel_size, 6);
    Eigen::Matrix<float, 6, Eigen::Dynamic> mat_At(6, laser_cloud_sel_size);
    Eigen::Matrix<float, 6, 6> matAtA;
    Eigen::VectorXf mat_B(laser_cloud_sel_size);
    Eigen::VectorXf mat_AtB;
    Eigen::VectorXf mat_X;

    SO3 R_SO3(transform_tobe_mapped_.rot); /// SO3

    for (int i = 0; i < laser_cloud_sel_size; i++) {
      point_ori = laser_cloud_ori.points[i];
      coeff = coeff_sel.points[i];

      Eigen::Vector3f p(point_ori.x, point_ori.y, point_ori.z);
      Eigen::Vector3f w(coeff.x, coeff.y, coeff.z);

//      Eigen::Vector3f J_r = w.transpose() * RotationVectorJacobian(R_SO3, p);
      Eigen::Vector3f J_r = -w.transpose() * (transform_tobe_mapped_.rot * SkewSymmetric(p));
      Eigen::Vector3f J_t = w.transpose();

      float d2 = coeff.intensity;

      mat_A(i, 0) = J_r.x();
      mat_A(i, 1) = J_r.y();
      mat_A(i, 2) = J_r.z();
      mat_A(i, 3) = J_t.x();
      mat_A(i, 4) = J_t.y();
      mat_A(i, 5) = J_t.z();
      mat_B(i, 0) = -d2;
    }

    mat_At = mat_A.transpose();
    matAtA = mat_At * mat_A;
    mat_AtB = mat_At * mat_B;
    mat_X = matAtA.colPivHouseholderQr().solve(mat_AtB);

    if (iter_count == 0) {
      Eigen::Matrix<float, 1, 6> mat_E;
      Eigen::Matrix<float, 6, 6> mat_V;
      Eigen::Matrix<float, 6, 6> mat_V2;

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 6, 6> > esolver(matAtA);
      mat_E = esolver.eigenvalues().real();
      mat_V = esolver.eigenvectors().real();

      mat_V2 = mat_V;

      is_degenerate = false;
      float eignThre[6] = {100, 100, 100, 100, 100, 100};
      for (int i = 0; i < 6; ++i) {
        if (mat_E(0, i) < eignThre[i]) {
          for (int j = 0; j < 6; ++j) {
            mat_V2(i, j) = 0;
          }
          is_degenerate = true;
          DLOG(WARNING) << "degenerate case";
          DLOG(INFO) << mat_E;
        } else {
          break;
        }
      }
      matP_ = mat_V2 * mat_V.inverse();
    }

    if (is_degenerate) {
      Eigen::Matrix<float, 6, 1> matX2(mat_X);
      mat_X = matP_ * matX2;
    }

    Eigen::Vector3f r_so3 = R_SO3.log();

    r_so3.x() += mat_X(0, 0);
    r_so3.y() += mat_X(1, 0);
    r_so3.z() += mat_X(2, 0);
    transform_tobe_mapped_.pos.x() += mat_X(3, 0);
    transform_tobe_mapped_.pos.y() += mat_X(4, 0);
    transform_tobe_mapped_.pos.z() += mat_X(5, 0);

    if (!isfinite(r_so3.x())) r_so3.x() = 0;
    if (!isfinite(r_so3.y())) r_so3.y() = 0;
    if (!isfinite(r_so3.z())) r_so3.z() = 0;

    SO3 tobe_mapped_SO3 = SO3::exp(r_so3);
//    transform_tobe_mapped_.rot = tobe_mapped_SO3.unit_quaternion().normalized();

    transform_tobe_mapped_.rot =
        transform_tobe_mapped_.rot * DeltaQ(Eigen::Vector3f(mat_X(0, 0), mat_X(1, 0), mat_X(2, 0)));

    if (!isfinite(transform_tobe_mapped_.pos.x())) transform_tobe_mapped_.pos.x() = 0.0;
    if (!isfinite(transform_tobe_mapped_.pos.y())) transform_tobe_mapped_.pos.y() = 0.0;
    if (!isfinite(transform_tobe_mapped_.pos.z())) transform_tobe_mapped_.pos.z() = 0.0;

    float delta_r = RadToDeg(R_SO3.unit_quaternion().angularDistance(transform_tobe_mapped_.rot));
    float delta_t = sqrt(pow(mat_X(3, 0) * 100, 2) +
        pow(mat_X(4, 0) * 100, 2) +
        pow(mat_X(5, 0) * 100, 2));

    if (delta_r < delta_r_abort_ && delta_t < delta_t_abort_) {
      DLOG(INFO) << "iter_count: " << iter_count;
      break;
    }
  }

  TransformUpdate();

  size_t laser_cloud_sel_spc_size = laser_cloud_ori_spc.points.size();
  if (laser_cloud_sel_spc_size >= 50) {
    for (int i = 0; i < laser_cloud_sel_spc_size; i++) {
      pair<float, pair<PointT, PointT>> spc;
      const PointT &p_ori = laser_cloud_ori_spc.points[i];
      const PointT &abs_coeff_in_map = abs_coeff_sel_spc.points[i];
      const PointT &coeff_in_map = coeff_sel_spc.points[i];

      PointT p_in_map;
      PointAssociateToMap(p_ori, p_in_map, transform_tobe_mapped_);

      spc.first = CalcPointDistance(coeff_in_map); // score
      spc.second.first = p_ori; // p_ori

//      spc.second.second = coeff_in_map; // coeff in map
//      spc.second.second.intensity = coeff_in_map.intensity
//          - (coeff_in_map.x * p_in_map.x + coeff_in_map.y * p_in_map.y + coeff_in_map.z * p_in_map.z);

      spc.second.second = abs_coeff_in_map;
//      LOG_IF(INFO, p_in_map.x * abs_coeff_in_map.x + p_in_map.y * abs_coeff_in_map.y + p_in_map.z * abs_coeff_in_map.z
//          + abs_coeff_in_map.intensity < 0)
//      << "distance: " << p_in_map.x * abs_coeff_in_map.x + p_in_map.y * abs_coeff_in_map.y
//          + p_in_map.z * abs_coeff_in_map.z + abs_coeff_in_map.intensity << " < 0";

      score_point_coeff_.insert(spc);


//      DLOG(INFO) << "distance * scale: "
//                << coeff_world.x * p_in_map.x + coeff_world.y * p_in_map.y + coeff_world.z * p_in_map.z
//                    + spc.second.second.intensity;

    }
//    DLOG(INFO) << "^^^^^^^^: " << transform_aft_mapped_;
  }
}

void PointMapping::TransformAssociateToMap() {
  Transform transform_incre(transform_bef_mapped_.inverse() * transform_sum_.transform());
  transform_tobe_mapped_ = transform_tobe_mapped_ * transform_incre;
}

void PointMapping::TransformUpdate() {
  transform_bef_mapped_ = transform_sum_;
  transform_aft_mapped_ = transform_tobe_mapped_;
}

void PointMapping::Process() {
  if (!HasNewData()) {
    // waiting for new data to arrive...
    // DLOG(INFO) << "no data received or dropped";
    return;
  }

  Reset();

  ++frame_count_;
  if (frame_count_ < num_stack_frames_) {
    return;
  }
  frame_count_ = 0;

  PointT point_sel;

  // relate incoming data to map
  // WARNING
  if (!imu_inited_) {
    TransformAssociateToMap();
  }

  // NOTE: the stack points are the last corner or surf poitns
  size_t laser_cloud_corner_last_size = laser_cloud_corner_last_->points.size();
  for (int i = 0; i < laser_cloud_corner_last_size; i++) {
    PointAssociateToMap(laser_cloud_corner_last_->points[i], point_sel, transform_tobe_mapped_);
    laser_cloud_corner_stack_->push_back(point_sel);
  }

  size_t laser_cloud_surf_last_size = laser_cloud_surf_last_->points.size();
  for (int i = 0; i < laser_cloud_surf_last_size; i++) {
    PointAssociateToMap(laser_cloud_surf_last_->points[i], point_sel, transform_tobe_mapped_);
    laser_cloud_surf_stack_->push_back(point_sel);
  }

  // NOTE: above codes update the transform with incremental value and update them to the map coordinate

  point_on_z_axis_.x = 0.0;
  point_on_z_axis_.y = 0.0;
  point_on_z_axis_.z = 10.0;
  PointAssociateToMap(point_on_z_axis_, point_on_z_axis_, transform_tobe_mapped_);

  // NOTE: in which cube
  int center_cube_i = int((transform_tobe_mapped_.pos.x() + 25.0) / 50.0) + laser_cloud_cen_length_;
  int center_cube_j = int((transform_tobe_mapped_.pos.y() + 25.0) / 50.0) + laser_cloud_cen_width_;
  int center_cube_k = int((transform_tobe_mapped_.pos.z() + 25.0) / 50.0) + laser_cloud_cen_height_;

  // NOTE: negative index
  if (transform_tobe_mapped_.pos.x() + 25.0 < 0) --center_cube_i;
  if (transform_tobe_mapped_.pos.y() + 25.0 < 0) --center_cube_j;
  if (transform_tobe_mapped_.pos.z() + 25.0 < 0) --center_cube_k;

//  DLOG(INFO) << "center_before: " << center_cube_i << " " << center_cube_j << " " << center_cube_k;
  {
    while (center_cube_i < 3) {
      for (int j = 0; j < laser_cloud_width_; ++j) {
        for (int k = 0; k < laser_cloud_height_; ++k) {
          for (int i = laser_cloud_length_ - 1; i >= 1; --i) {
            const size_t index_a = ToIndex(i, j, k);
            const size_t index_b = ToIndex(i - 1, j, k);
            std::swap(laser_cloud_corner_array_[index_a], laser_cloud_corner_array_[index_b]);
            std::swap(laser_cloud_surf_array_[index_a], laser_cloud_surf_array_[index_b]);
          }
          laser_cloud_corner_array_[ToIndex(0, j, k)]->clear();
          laser_cloud_surf_array_[ToIndex(0, j, k)]->clear();
        }
      }
      ++center_cube_i;
      ++laser_cloud_cen_length_;
    }

    while (center_cube_i >= laser_cloud_length_ - 3) {
      for (int j = 0; j < laser_cloud_width_; ++j) {
        for (int k = 0; k < laser_cloud_height_; ++k) {
          for (int i = 0; i < laser_cloud_length_ - 1; ++i) {
            const size_t index_a = ToIndex(i, j, k);
            const size_t index_b = ToIndex(i + 1, j, k);
            std::swap(laser_cloud_corner_array_[index_a], laser_cloud_corner_array_[index_b]);
            std::swap(laser_cloud_surf_array_[index_a], laser_cloud_surf_array_[index_b]);
          }
          laser_cloud_corner_array_[ToIndex(laser_cloud_length_ - 1, j, k)]->clear();
          laser_cloud_surf_array_[ToIndex(laser_cloud_length_ - 1, j, k)]->clear();
        }
      }
      --center_cube_i;
      --laser_cloud_cen_length_;
    }

    while (center_cube_j < 3) {
      for (int i = 0; i < laser_cloud_length_; ++i) {
        for (int k = 0; k < laser_cloud_height_; ++k) {
          for (int j = laser_cloud_width_ - 1; j >= 1; --j) {
            const size_t index_a = ToIndex(i, j, k);
            const size_t index_b = ToIndex(i, j - 1, k);
            std::swap(laser_cloud_corner_array_[index_a], laser_cloud_corner_array_[index_b]);
            std::swap(laser_cloud_surf_array_[index_a], laser_cloud_surf_array_[index_b]);
          }
          laser_cloud_corner_array_[ToIndex(i, 0, k)]->clear();
          laser_cloud_surf_array_[ToIndex(i, 0, k)]->clear();
        }
      }
      ++center_cube_j;
      ++laser_cloud_cen_width_;
    }

    while (center_cube_j >= laser_cloud_width_ - 3) {
      for (int i = 0; i < laser_cloud_length_; ++i) {
        for (int k = 0; k < laser_cloud_height_; ++k) {
          for (int j = 0; j < laser_cloud_width_ - 1; ++j) {
            const size_t index_a = ToIndex(i, j, k);
            const size_t index_b = ToIndex(i, j + 1, k);
            std::swap(laser_cloud_corner_array_[index_a], laser_cloud_corner_array_[index_b]);
            std::swap(laser_cloud_surf_array_[index_a], laser_cloud_surf_array_[index_b]);
          }
          laser_cloud_corner_array_[ToIndex(i, laser_cloud_width_ - 1, k)]->clear();
          laser_cloud_surf_array_[ToIndex(i, laser_cloud_width_ - 1, k)]->clear();
        }
      }
      --center_cube_j;
      --laser_cloud_cen_width_;
    }

    while (center_cube_k < 3) {
      for (int i = 0; i < laser_cloud_length_; ++i) {
        for (int j = 0; j < laser_cloud_width_; ++j) {
          for (int k = laser_cloud_height_ - 1; k >= 1; --k) {
            const size_t index_a = ToIndex(i, j, k);
            const size_t index_b = ToIndex(i, j, k - 1);
            std::swap(laser_cloud_corner_array_[index_a], laser_cloud_corner_array_[index_b]);
            std::swap(laser_cloud_surf_array_[index_a], laser_cloud_surf_array_[index_b]);
          }
          laser_cloud_corner_array_[ToIndex(i, j, 0)]->clear();
          laser_cloud_surf_array_[ToIndex(i, j, 0)]->clear();
        }
      }
      ++center_cube_k;
      ++laser_cloud_cen_height_;
    }

    while (center_cube_k >= laser_cloud_height_ - 3) {
      for (int i = 0; i < laser_cloud_length_; ++i) {
        for (int j = 0; j < laser_cloud_width_; ++j) {
          for (int k = 0; k < laser_cloud_height_ - 1; ++k) {
            const size_t index_a = ToIndex(i, j, k);
            const size_t index_b = ToIndex(i, j, k + 1);
            std::swap(laser_cloud_corner_array_[index_a], laser_cloud_corner_array_[index_b]);
            std::swap(laser_cloud_surf_array_[index_a], laser_cloud_surf_array_[index_b]);
          }
          laser_cloud_corner_array_[ToIndex(i, j, laser_cloud_height_ - 1)]->clear();
          laser_cloud_surf_array_[ToIndex(i, j, laser_cloud_height_ - 1)]->clear();
        }
      }
      --center_cube_k;
      --laser_cloud_cen_height_;
    }
  }

  // NOTE: above slide cubes


  laser_cloud_valid_idx_.clear();
  laser_cloud_surround_idx_.clear();

//  DLOG(INFO) << "center_after: " << center_cube_i << " " << center_cube_j << " " << center_cube_k;
//  DLOG(INFO) << "laser_cloud_cen: " << laser_cloud_cen_length_ << " " << laser_cloud_cen_width_ << " "
//            << laser_cloud_cen_height_;

  for (int i = center_cube_i - 2; i <= center_cube_i + 2; ++i) {
    for (int j = center_cube_j - 2; j <= center_cube_j + 2; ++j) {
      for (int k = center_cube_k - 2; k <= center_cube_k + 2; ++k) {
        if (i >= 0 && i < laser_cloud_length_ &&
            j >= 0 && j < laser_cloud_width_ &&
            k >= 0 && k < laser_cloud_height_) { /// Should always in this condition

          float center_x = 50.0f * (i - laser_cloud_cen_length_);
          float center_y = 50.0f * (j - laser_cloud_cen_width_);
          float center_z = 50.0f * (k - laser_cloud_cen_height_); // NOTE: center of the cube

          PointT transform_pos;
          transform_pos.x = transform_tobe_mapped_.pos.x();
          transform_pos.y = transform_tobe_mapped_.pos.y();
          transform_pos.z = transform_tobe_mapped_.pos.z();

          bool is_in_laser_fov = false;
          for (int ii = -1; ii <= 1; ii += 2) {
            for (int jj = -1; jj <= 1; jj += 2) {
              for (int kk = -1; kk <= 1; kk += 2) {
                PointT corner;
                corner.x = center_x + 25.0f * ii;
                corner.y = center_y + 25.0f * jj;
                corner.z = center_z + 25.0f * kk;

                float squared_side1 = CalcSquaredDiff(transform_pos, corner);
                float squared_side2 = CalcSquaredDiff(point_on_z_axis_, corner);

                float check1 = 100.0f + squared_side1 - squared_side2
                    - 10.0f * sqrt(3.0f) * sqrt(squared_side1);

                float check2 = 100.0f + squared_side1 - squared_side2
                    + 10.0f * sqrt(3.0f) * sqrt(squared_side1);

                if (check1 < 0 && check2 > 0) { /// within +-60 degree
                  is_in_laser_fov = true;
                }
              }
            }
          }

          size_t cube_idx = ToIndex(i, j, k);

//          DLOG(INFO) << "ToIndex, i, j, k " << cube_idx << " " << i << " " << j << " " << k;
//          int tmpi, tmpj, tmpk;
//          FromIndex(cube_idx, tmpi, tmpj, tmpk);
//          DLOG(INFO) << "FromIndex, i, j, k " << cube_idx << " " << tmpi << " " << tmpj << " " << tmpk;

          if (is_in_laser_fov) {
            laser_cloud_valid_idx_.push_back(cube_idx);
          }
          laser_cloud_surround_idx_.push_back(cube_idx);
        }
      }
    }
  }

  // prepare valid map corner and surface cloud for pose optimization
  laser_cloud_corner_from_map_->clear();
  laser_cloud_surf_from_map_->clear();
  size_t laser_cloud_valid_size = laser_cloud_valid_idx_.size();
  for (int i = 0; i < laser_cloud_valid_size; ++i) {
    *laser_cloud_corner_from_map_ += *laser_cloud_corner_array_[laser_cloud_valid_idx_[i]];
    *laser_cloud_surf_from_map_ += *laser_cloud_surf_array_[laser_cloud_valid_idx_[i]];
  }

  // prepare feature stack clouds for pose optimization
  size_t laser_cloud_corner_stack_size2 = laser_cloud_corner_stack_->points.size();
  for (int i = 0; i < laser_cloud_corner_stack_size2; ++i) {
    PointAssociateTobeMapped(laser_cloud_corner_stack_->points[i],
                             laser_cloud_corner_stack_->points[i],
                             transform_tobe_mapped_);
  }

  size_t laserCloudSurfStackNum2 = laser_cloud_surf_stack_->points.size();
  for (int i = 0; i < laserCloudSurfStackNum2; ++i) {
    PointAssociateTobeMapped(laser_cloud_surf_stack_->points[i],
                             laser_cloud_surf_stack_->points[i],
                             transform_tobe_mapped_);
  }

  // down sample feature stack clouds
  laser_cloud_corner_stack_downsampled_->clear();
  down_size_filter_corner_.setInputCloud(laser_cloud_corner_stack_);
  down_size_filter_corner_.filter(*laser_cloud_corner_stack_downsampled_);
  size_t laser_cloud_corner_stack_ds_size = laser_cloud_corner_stack_downsampled_->points.size();

  laser_cloud_surf_stack_downsampled_->clear();
  down_size_filter_surf_.setInputCloud(laser_cloud_surf_stack_);
  down_size_filter_surf_.filter(*laser_cloud_surf_stack_downsampled_);
  size_t laser_cloud_surf_stack_ds_size = laser_cloud_surf_stack_downsampled_->points.size();

  laser_cloud_corner_stack_->clear();
  laser_cloud_surf_stack_->clear();

  // NOTE: keeps the downsampled points

  // NOTE: run pose optimization
  OptimizeTransformTobeMapped();

  if (!imu_inited_) {
    // store down sized corner stack points in corresponding cube clouds

    CubeCenter cube_center;
    cube_center.laser_cloud_cen_length = laser_cloud_cen_length_;
    cube_center.laser_cloud_cen_width = laser_cloud_cen_width_;
    cube_center.laser_cloud_cen_height = laser_cloud_cen_height_;

    UpdateMapDatabase(laser_cloud_corner_stack_downsampled_,
                      laser_cloud_surf_stack_downsampled_,
                      laser_cloud_valid_idx_,
                      transform_tobe_mapped_,
                      cube_center);
#if 0
    for (int i = 0; i < laser_cloud_corner_stack_ds_size; ++i) {
      PointAssociateToMap(laser_cloud_corner_stack_downsampled_->points[i], point_sel, transform_tobe_mapped_);

      int cube_i = int((point_sel.x + 25.0) / 50.0) + laser_cloud_cen_length_;
      int cube_j = int((point_sel.y + 25.0) / 50.0) + laser_cloud_cen_width_;
      int cube_k = int((point_sel.z + 25.0) / 50.0) + laser_cloud_cen_height_;

      if (point_sel.x + 25.0 < 0) --cube_i;
      if (point_sel.y + 25.0 < 0) --cube_j;
      if (point_sel.z + 25.0 < 0) --cube_k;

      if (cube_i >= 0 && cube_i < laser_cloud_length_ &&
          cube_j >= 0 && cube_j < laser_cloud_width_ &&
          cube_k >= 0 && cube_k < laser_cloud_height_) {
        size_t cube_idx = ToIndex(cube_i, cube_j, cube_k);
        laser_cloud_corner_array_[cube_idx]->push_back(point_sel);
      }
    }

    // store down sized surface stack points in corresponding cube clouds
    for (int i = 0; i < laser_cloud_surf_stack_ds_size; ++i) {
      PointAssociateToMap(laser_cloud_surf_stack_downsampled_->points[i], point_sel, transform_tobe_mapped_);

      int cube_i = int((point_sel.x + 25.0) / 50.0) + laser_cloud_cen_length_;
      int cube_j = int((point_sel.y + 25.0) / 50.0) + laser_cloud_cen_width_;
      int cube_k = int((point_sel.z + 25.0) / 50.0) + laser_cloud_cen_height_;

      if (point_sel.x + 25.0 < 0) --cube_i;
      if (point_sel.y + 25.0 < 0) --cube_j;
      if (point_sel.z + 25.0 < 0) --cube_k;

      if (cube_i >= 0 && cube_i < laser_cloud_length_ &&
          cube_j >= 0 && cube_j < laser_cloud_width_ &&
          cube_k >= 0 && cube_k < laser_cloud_height_) {
        size_t cube_idx = ToIndex(cube_i, cube_j, cube_k);
        laser_cloud_surf_array_[cube_idx]->push_back(point_sel);
      }
    }

    // down size all valid (within field of view) feature cube clouds
    for (int i = 0; i < laser_cloud_valid_size; ++i) {
      size_t index = laser_cloud_valid_idx_[i];

      laser_cloud_corner_downsampled_array_[index]->clear();
      down_size_filter_corner_.setInputCloud(laser_cloud_corner_array_[index]);
      down_size_filter_corner_.filter(*laser_cloud_corner_downsampled_array_[index]);

      laser_cloud_surf_downsampled_array_[index]->clear();
      down_size_filter_surf_.setInputCloud(laser_cloud_surf_array_[index]);
      down_size_filter_surf_.filter(*laser_cloud_surf_downsampled_array_[index]);

      // swap cube clouds for next processing
      laser_cloud_corner_array_[index].swap(laser_cloud_corner_downsampled_array_[index]);
      laser_cloud_surf_array_[index].swap(laser_cloud_surf_downsampled_array_[index]);
    }
#endif

    // publish result
    PublishResults();
  }

  DLOG(INFO) << "mapping: " << tic_toc_.Toc() << " ms";

}

void PointMapping::UpdateMapDatabase(lio::PointCloudPtr margin_corner_stack_downsampled,
                                     lio::PointCloudPtr margin_surf_stack_downsampled,
                                     std::vector<size_t> margin_valid_idx,
                                     const Transform &margin_transform_tobe_mapped,
                                     const CubeCenter &margin_cube_center) {

  PointT point_sel;
  size_t margin_corner_stack_ds_size = margin_corner_stack_downsampled->points.size();
  size_t margin_surf_stack_ds_size = margin_surf_stack_downsampled->points.size();
  size_t margin_valid_size = margin_valid_idx.size();

  for (int i = 0; i < margin_corner_stack_ds_size; ++i) {
    PointAssociateToMap(margin_corner_stack_downsampled->points[i], point_sel, margin_transform_tobe_mapped);

    int cube_i = int((point_sel.x + 25.0) / 50.0) + laser_cloud_cen_length_;
    int cube_j = int((point_sel.y + 25.0) / 50.0) + laser_cloud_cen_width_;
    int cube_k = int((point_sel.z + 25.0) / 50.0) + laser_cloud_cen_height_;

    if (point_sel.x + 25.0 < 0) --cube_i;
    if (point_sel.y + 25.0 < 0) --cube_j;
    if (point_sel.z + 25.0 < 0) --cube_k;

    if (cube_i >= 0 && cube_i < laser_cloud_length_ &&
        cube_j >= 0 && cube_j < laser_cloud_width_ &&
        cube_k >= 0 && cube_k < laser_cloud_height_) {
      size_t cube_idx = ToIndex(cube_i, cube_j, cube_k);
      laser_cloud_corner_array_[cube_idx]->push_back(point_sel);
    }
  }

  // store down sized surface stack points in corresponding cube clouds
  for (int i = 0; i < margin_surf_stack_ds_size; ++i) {
    PointAssociateToMap(margin_surf_stack_downsampled->points[i], point_sel, margin_transform_tobe_mapped);

    int cube_i = int((point_sel.x + 25.0) / 50.0) + laser_cloud_cen_length_;
    int cube_j = int((point_sel.y + 25.0) / 50.0) + laser_cloud_cen_width_;
    int cube_k = int((point_sel.z + 25.0) / 50.0) + laser_cloud_cen_height_;

    if (point_sel.x + 25.0 < 0) --cube_i;
    if (point_sel.y + 25.0 < 0) --cube_j;
    if (point_sel.z + 25.0 < 0) --cube_k;

    if (cube_i >= 0 && cube_i < laser_cloud_length_ &&
        cube_j >= 0 && cube_j < laser_cloud_width_ &&
        cube_k >= 0 && cube_k < laser_cloud_height_) {
      size_t cube_idx = ToIndex(cube_i, cube_j, cube_k);
      laser_cloud_surf_array_[cube_idx]->push_back(point_sel);
    }
  }

  // TODO: varlidate margin_valid_idx
  // down size all valid (within field of view) feature cube clouds
  for (int i = 0; i < margin_valid_size; ++i) {
    size_t index = margin_valid_idx[i];

//    DLOG(INFO) << "index before: " << index;

    int last_i, last_j, last_k;

    FromIndex(index, last_i, last_j, last_k);

    float center_x = 50.0f * (last_i - margin_cube_center.laser_cloud_cen_length);
    float center_y = 50.0f * (last_j - margin_cube_center.laser_cloud_cen_width);
    float center_z = 50.0f * (last_k - margin_cube_center.laser_cloud_cen_height); // NOTE: center of the margin cube

    int cube_i = int((center_x + 25.0) / 50.0) + laser_cloud_cen_length_;
    int cube_j = int((center_y + 25.0) / 50.0) + laser_cloud_cen_width_;
    int cube_k = int((center_z + 25.0) / 50.0) + laser_cloud_cen_height_;

    if (center_x + 25.0 < 0) --cube_i;
    if (center_y + 25.0 < 0) --cube_j;
    if (center_z + 25.0 < 0) --cube_k;

    if (cube_i >= 0 && cube_i < laser_cloud_length_ &&
        cube_j >= 0 && cube_j < laser_cloud_width_ &&
        cube_k >= 0 && cube_k < laser_cloud_height_) {

      index = ToIndex(cube_i, cube_j, cube_k); // NOTE: update to current index

//      DLOG(INFO) << "index after: " << index;

      laser_cloud_corner_downsampled_array_[index]->clear();
      down_size_filter_corner_.setInputCloud(laser_cloud_corner_array_[index]);
      down_size_filter_corner_.filter(*laser_cloud_corner_downsampled_array_[index]);

      laser_cloud_surf_downsampled_array_[index]->clear();
      down_size_filter_surf_.setInputCloud(laser_cloud_surf_array_[index]);
      down_size_filter_surf_.filter(*laser_cloud_surf_downsampled_array_[index]);

      // swap cube clouds for next processing
      laser_cloud_corner_array_[index].swap(laser_cloud_corner_downsampled_array_[index]);
      laser_cloud_surf_array_[index].swap(laser_cloud_surf_downsampled_array_[index]);
    }

  }

}

void PointMapping::PublishResults() {

  if (!is_ros_setup_) {
    DLOG(WARNING) << "ros is not set up, and no results will be published";
    return;
  }

  // publish new map cloud according to the input output ratio
  ++map_frame_count_;
  if (map_frame_count_ >= num_map_frames_) {
    map_frame_count_ = 0;

    // accumulate map cloud
    laser_cloud_surround_->clear();
    size_t laser_cloud_surround_size = laser_cloud_surround_idx_.size();
    for (int i = 0; i < laser_cloud_surround_size; ++i) {
      size_t index = laser_cloud_surround_idx_[i];
      *laser_cloud_surround_ += *laser_cloud_corner_array_[index];
      *laser_cloud_surround_ += *laser_cloud_surf_array_[index];
    }

    // down size map cloud
    laser_cloud_surround_downsampled_->clear();
    down_size_filter_map_.setInputCloud(laser_cloud_surround_);
    down_size_filter_map_.filter(*laser_cloud_surround_downsampled_);

    // publish new map cloud
    PublishCloudMsg(pub_laser_cloud_surround_,
                    *laser_cloud_surround_downsampled_,
                    time_laser_odometry_,
                    "/camera_init");
  }


  // transform full resolution input cloud to map
  size_t laser_full_cloud_size = full_cloud_->points.size();
  for (int i = 0; i < laser_full_cloud_size; i++) {
    PointAssociateToMap(full_cloud_->points[i], full_cloud_->points[i], transform_tobe_mapped_);
  }

  // publish transformed full resolution input cloud
  PublishCloudMsg(pub_full_cloud_, *full_cloud_, time_laser_odometry_, "/camera_init");


  // publish odometry after mapped transformations
  geometry_msgs::Quaternion geo_quat;
  geo_quat.w = transform_aft_mapped_.rot.w();
  geo_quat.x = transform_aft_mapped_.rot.x();
  geo_quat.y = transform_aft_mapped_.rot.y();
  geo_quat.z = transform_aft_mapped_.rot.z();

  odom_aft_mapped_.header.stamp = time_laser_odometry_;
  odom_aft_mapped_.pose.pose.orientation.x = geo_quat.x;
  odom_aft_mapped_.pose.pose.orientation.y = geo_quat.y;
  odom_aft_mapped_.pose.pose.orientation.z = geo_quat.z;
  odom_aft_mapped_.pose.pose.orientation.w = geo_quat.w;
  odom_aft_mapped_.pose.pose.position.x = transform_aft_mapped_.pos.x();
  odom_aft_mapped_.pose.pose.position.y = transform_aft_mapped_.pos.y();
  odom_aft_mapped_.pose.pose.position.z = transform_aft_mapped_.pos.z();

//  odom_aft_mapped_.twist.twist.angular.x = transform_bef_mapped_.rot.x();
//  odom_aft_mapped_.twist.twist.angular.y = transform_bef_mapped_.rot.y();
//  odom_aft_mapped_.twist.twist.angular.z = transform_bef_mapped_.rot.z();
//  odom_aft_mapped_.twist.twist.linear.x = transform_bef_mapped_.pos.x();
//  odom_aft_mapped_.twist.twist.linear.y = transform_bef_mapped_.pos.y();
//  odom_aft_mapped_.twist.twist.linear.z = transform_bef_mapped_.pos.z();
  pub_odom_aft_mapped_.publish(odom_aft_mapped_);

  aft_mapped_trans_.stamp_ = time_laser_odometry_;
  aft_mapped_trans_.setRotation(tf::Quaternion(geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w));
  aft_mapped_trans_.setOrigin(tf::Vector3(transform_aft_mapped_.pos.x(),
                                          transform_aft_mapped_.pos.y(),
                                          transform_aft_mapped_.pos.z()));
  tf_broadcaster_.sendTransform(aft_mapped_trans_);
}

} // namespace lio
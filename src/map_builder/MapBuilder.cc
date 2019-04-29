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

#include "map_builder/MapBuilder.h"

#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>

//#define DEBUG

namespace lio {

template<typename Derived>
inline Eigen::Matrix<typename Derived::Scalar, 3, 1>
ConstrainedRotAxis(const Eigen::QuaternionBase<Derived> &quat_in) {
  typedef typename Derived::Scalar Scalar;
  Eigen::Quaternion<Scalar> q_R = quat_in.normalized();
  Eigen::AngleAxis<Scalar> angle_axis_R(q_R);
  Eigen::Matrix<Scalar, 3, 1> axis_R = angle_axis_R.axis();
  Scalar angle_R = angle_axis_R.angle();
  Scalar cot_half_angle = (isinf(1 / tan(angle_R / 2)) ? 1e6 : (1 / tan(angle_R / 2)));
  Eigen::Matrix<Scalar, 3, 1> delta_axis_R{-axis_R.y(), axis_R.x(), cot_half_angle};
  Eigen::Matrix<Scalar, 3, 1> constrained_axis = axis_R.cross(delta_axis_R);
  constrained_axis.normalize();
  return constrained_axis;
}

void MapBuilder::Transform4DAssociateToMap() {
  Transform transform_incre(transform_bef_mapped_.inverse() * transform_sum_.transform());
  Transform full_transform = transform_tobe_mapped_ * transform_incre;

  Eigen::Vector3d origin_R0 = R2ypr(full_transform.rot.normalized().toRotationMatrix().cast<double>());

  Eigen::Vector3d origin_R00 = R2ypr(transform_sum_.rot.normalized().toRotationMatrix().cast<double>());

  // Z-axix R00 to R0, regard para_pose's R as rotate along the Z-axis first
  double y_diff = origin_R0.x() - origin_R00.x();

  //TODO
  Eigen::Matrix3f rot_diff = ypr2R(Eigen::Vector3f(y_diff, 0, 0));

  transform_tobe_mapped_.pos = full_transform.pos;
#ifdef DEBUG
  transform_tobe_mapped_.rot = transform_sum_.rot.normalized();
#else
  transform_tobe_mapped_.rot = rot_diff * transform_sum_.rot.normalized();
#endif
}

void MapBuilder::Transform4DUpdate() {
//  Eigen::Vector3d origin_R0 = R2ypr(transform_tobe_mapped_.rot.normalized().toRotationMatrix().cast<double>());
//  Eigen::Vector3d origin_R00 = R2ypr(transform_sum_.rot.normalized().toRotationMatrix().cast<double>());
//
//  // Z-axix R00 to R0, regard para_pose's R as rotate along the Z-axis first
//  double y_diff = origin_R0.x() - origin_R00.x();
//
//  Eigen::Matrix3f rot_diff = ypr2R(Eigen::Vector3f(y_diff, 0, 0));
//
//  transform_tobe_mapped_.rot = rot_diff * transform_sum_.rot.normalized();

  transform_bef_mapped_ = transform_sum_;
  transform_aft_mapped_ = transform_tobe_mapped_;
}

MapBuilder::MapBuilder(MapBuilderConfig config) {

  down_size_filter_corner_.setLeafSize(config.corner_filter_size, config.corner_filter_size, config.corner_filter_size);
  down_size_filter_surf_.setLeafSize(config.surf_filter_size, config.surf_filter_size, config.surf_filter_size);
  down_size_filter_map_.setLeafSize(config.map_filter_size, config.map_filter_size, config.map_filter_size);

  min_match_sq_dis_ = config.min_match_sq_dis;
  min_plane_dis_ = config.min_plane_dis;

  config_ = config;
}

void MapBuilder::SetupRos(ros::NodeHandle &nh) {
//  PointMapping::SetupRos(nh, true);

  is_ros_setup_ = true;

  nh.param("enable_4d", enable_4d_, true);
  nh.param("skip_count", skip_count_, 2);

  pub_laser_cloud_surround_ = nh.advertise<sensor_msgs::PointCloud2>("laser_cloud_surround", 2);
  pub_full_cloud_ = nh.advertise<sensor_msgs::PointCloud2>("cloud_registered", 2);
  pub_odom_aft_mapped_ = nh.advertise<nav_msgs::Odometry>("aft_mapped_to_init", 5);

  /// for test
//  pub_diff_odometry_ = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_last", 5);


  // subscribe to scan registration topics
  sub_laser_cloud_corner_last_ = nh.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_corner_last", 2, &PointMapping::LaserCloudCornerLastHandler, (PointMapping *) this);

  sub_laser_cloud_surf_last_ = nh.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_surf_last", 2, &PointMapping::LaserCloudSurfLastHandler, (PointMapping *) this);

  sub_laser_full_cloud_ = nh.subscribe<sensor_msgs::PointCloud2>
      ("/full_odom_cloud", 2, &PointMapping::LaserFullCloudHandler, (PointMapping *) this);

  sub_laser_odometry_ = nh.subscribe<nav_msgs::Odometry>
      ("/laser_odom_to_init", 2, &PointMapping::LaserOdometryHandler, (PointMapping *) this);

//  sub_imu_trans_ = node.subscribe<sensor_msgs::PointCloud2>
//      ("/imu_trans", 5, &LaserOdometry::ImuTransHandler, this);


  odom_aft_mapped_.header.frame_id = "/world";
  odom_aft_mapped_.child_frame_id = "/aft_4d_mapped";

  aft_mapped_trans_.frame_id_ = "/world";
  aft_mapped_trans_.child_frame_id_ = "/aft_4d_mapped";
}

void MapBuilder::PublishMapBuilderResults() {

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
                    "/world");
  }


  // transform full resolution input cloud to map
  size_t laser_full_cloud_size = full_cloud_->points.size();
  for (int i = 0; i < laser_full_cloud_size; i++) {
    PointAssociateToMap(full_cloud_->points[i], full_cloud_->points[i], transform_tobe_mapped_);
  }

  // publish transformed full resolution input cloud
  PublishCloudMsg(pub_full_cloud_, *full_cloud_, time_laser_odometry_, "/world");


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

void MapBuilder::ProcessMap() {
  if (!HasNewData()) {
    // waiting for new data to arrive...
    // DLOG(INFO) << "no data received or dropped";
    return;
  }

  if (!system_init_) {
    system_init_ = true;
    transform_bef_mapped_ = transform_sum_;
    transform_tobe_mapped_ = transform_sum_;
    transform_aft_mapped_ = transform_tobe_mapped_;
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
  if (enable_4d_) {
    Transform4DAssociateToMap();
  } else {
    TransformAssociateToMap();
    DLOG(INFO) << "DISABLE 4D";
  }

//  // NOTE: for debug
//  {
//    Eigen::Affine3f transform_to_world;
//    tf::Transform tf_transform;
//    Eigen::Matrix3f R_inv;
//    R_inv << -4.91913910e-01, -5.01145813e-01, -7.11950546e-01,
//              7.13989130e-01, -7.00156621e-01, -4.78439170e-04,
//             -4.98237120e-01, -5.08560301e-01, 7.02229444e-01;
//    transform_to_world.setIdentity();
//    transform_to_world.linear() = R_inv.transpose();
//
//
//    Eigen::Quaternionf q_eigen(R_inv.transpose());
//    tf::Quaternion q(tf::Quaternion{q_eigen.x(), q_eigen.y(), q_eigen.z(), q_eigen.w()});
//    tf_transform.setRotation(q);
//
//
//    // fetch new input cloud
//    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_ptr(new pcl::PointCloud<pcl::PointXYZI>());
//    pcl::PointCloud<pcl::PointXYZI> tmp_cloud;
//
//    pcl::transformPointCloud(*laser_cloud_surf_last_, *transformed_ptr, transform_to_world);
//
//    pcl::CropBox<pcl::PointXYZI> box_filter;
//    box_filter.setMin(Eigen::Vector4f(-12, -5, -3, 1.0));
//    box_filter.setMax(Eigen::Vector4f(5, 10, 0.6, 1.0));
//    box_filter.setNegative(true);
//    box_filter.setInputCloud(transformed_ptr);
//    box_filter.filter(tmp_cloud);
//
//    pcl::transformPointCloud(tmp_cloud, *laser_cloud_surf_last_, transform_to_world.inverse());
//  }

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
  if (odom_count_ % skip_count_ == 0) {
    if (enable_4d_) {
      OptimizeMap();
    } else {
      OptimizeTransformTobeMapped();
      DLOG(INFO) << "DISABLE 4D";
    }
  } else {
    if (enable_4d_) {
      Transform4DUpdate();
    } else {
      TransformUpdate();
      DLOG(INFO) << "DISABLE 4D";
    }
  }
  ++odom_count_;

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
  // PublishResults();
  PublishMapBuilderResults();

  DLOG(INFO) << "mapping: " << tic_toc_.Toc() << " ms";

}

void MapBuilder::OptimizeMap() {
  if (laser_cloud_corner_from_map_->points.size() <= 10 || laser_cloud_surf_from_map_->points.size() <= 100) {
    LOG(ERROR) << "skip due to insufficient points";
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
  score_point_coeff_.clear();

  tic_toc_.Tic();

  Eigen::Quaternionf q_R = transform_sum_.rot.normalized();
  Eigen::Vector3f constrained_axis = ConstrainedRotAxis(q_R);
  constrained_axis.normalize();
  Eigen::Matrix3f constrained_R =
      Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f{1.0, 0.0, 0.0}, constrained_axis).toRotationMatrix();
  Eigen::Matrix3f info_mat;
  info_mat.setZero();
  info_mat(0, 0) = 1;
  info_mat(1, 1) = 1;
  info_mat(2, 2) = 1;
  info_mat = (constrained_R.transpose() * info_mat * constrained_R).eval();
  Eigen::Matrix3f projection_mat =
      constrained_axis * (constrained_axis.transpose() * constrained_axis).inverse() * constrained_axis.transpose();

  for (size_t iter_count = 0; iter_count < num_max_iterations_; ++iter_count) {
    laser_cloud_ori.clear();
    coeff_sel.clear();

    laser_cloud_ori_spc.clear();

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

          coeff.x = s * pa;
          coeff.y = s * pb;
          coeff.z = s * pc;
          coeff.intensity = s * pd2;

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

      Eigen::Matrix3f right_info_mat;
      right_info_mat.setZero();
      right_info_mat(0, 0) = 5e-3;
      right_info_mat(1, 1) = 5e-3;
      right_info_mat(2, 2) = 1;
//      Eigen::Vector3f J_r = w.transpose() * RotationVectorJacobian(R_SO3, p);
#ifdef DEBUG
      Eigen::Vector3f J_r = -w.transpose() * (transform_tobe_mapped_.rot * SkewSymmetric(p));
#else
      Eigen::Vector3f J_r = -w.transpose() * (transform_tobe_mapped_.rot * SkewSymmetric(p))
          * transform_tobe_mapped_.rot.inverse().toRotationMatrix() * right_info_mat;
#endif
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

//    DLOG(INFO) << "mat_X.head<3>() bef: " << mat_X.head<3>().transpose();
//    Eigen::Vector3f X_r = projection_mat * mat_X.head<3>();
//    mat_X.head<3>() = mat_X.head<3>() - X_r;
//    DLOG(INFO) << "X_r:" << X_r.transpose();
//    DLOG(INFO) << "mat_X.head<3>() aft: " << mat_X.head<3>().transpose();

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

#ifdef DEBUG
    transform_tobe_mapped_.rot =
        (transform_tobe_mapped_.rot * DeltaQ(Eigen::Vector3f(mat_X(0, 0), mat_X(1, 0), mat_X(2, 0))));
#else
    transform_tobe_mapped_.rot =
        (DeltaQ(Eigen::Vector3f(mat_X(0, 0), mat_X(1, 0), mat_X(2, 0))) * transform_tobe_mapped_.rot);
#endif

//    Eigen::AngleAxisf angle_axis_out(transform_tobe_mapped_.rot);
//    DLOG(INFO) << "1. angle_axis_out.axis() bef: " << angle_axis_out.axis().transpose();
//    Eigen::Vector3f X_r = projection_mat * angle_axis_out.axis();
//    angle_axis_out.axis() = angle_axis_out.axis() - X_r;
//    angle_axis_out.axis().normalize();
//    transform_tobe_mapped_.rot = Eigen::Quaternionf(angle_axis_out);
//
//    DLOG(INFO) << "2. X_r: " << X_r.transpose();
//    DLOG(INFO) << "3. angle_axis_out.axis() aft: " << angle_axis_out.axis().transpose();

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

  Transform4DUpdate();
}

}
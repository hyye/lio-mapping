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
// Created by hyye on 5/9/18.
//

#ifndef LIO_FEATURE_MANAGER_H_
#define LIO_FEATURE_MANAGER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <glog/logging.h>

namespace lio {

using std::unique_ptr;

struct Feature {
  std::string feature_name;
  virtual void GetFeature(Feature *feature) {
    DLOG(WARNING) << ">>>>>>> GetFeature not implemented <<<<<<<";
  }
};

struct PointNormalFeature : public Feature {
 public:
  PointNormalFeature() {
    feature_name = "PointNormalFeature";
  }
  PointNormalFeature(const Eigen::Vector3d &point3d_in, const Eigen::Vector3d &normal3d_in) {
    feature_name = "PointNormalFeature";
    point3d = point3d_in;
    normal3d = normal3d_in;
    diag_covariance = Eigen::Vector3d{gicp_epsilon, 1.0, 1.0}.asDiagonal();
    UpdateCovariance(normal3d);
  }

  void UpdateCovariance(const Eigen::Vector3d &normal3d_in);

  PointNormalFeature &GetFeatureInner() {
    return *this;
  }

  void GetFeature(Feature *feature) {
    PointNormalFeature *derived_feature = static_cast<PointNormalFeature *>(feature);
    *derived_feature = this->GetFeatureInner();
  }

  double gicp_epsilon = 0.001;
  Eigen::Vector3d e1{1.0, 0.0, 0.0};
  Eigen::Vector3d point3d; /// the point in the current frame
  Eigen::Vector3d normal3d; /// the normal vector in the current frame
  Eigen::Matrix3d diag_covariance;
  Eigen::Matrix3d covariance; /// the covariance in the current frame from the normal vector

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

struct PointPlaneFeature : public Feature {
  PointPlaneFeature() {
    feature_name = "PointPlaneFeature";
  }

  PointPlaneFeature(const Eigen::Vector3d &point_in, const Eigen::Vector4d &coeffs_in) {
    feature_name = "PointPlaneFeature";
    point = point_in;
    coeffs = coeffs_in;
  }

  PointPlaneFeature &GetFeatureInner() {
    return *this;
  }

  void GetFeature(Feature *feature) {
    PointPlaneFeature *derived_feature = static_cast<PointPlaneFeature *>(feature);
    *derived_feature = this->GetFeatureInner();
  }

  double score;
  Eigen::Vector3d point;
  Eigen::Vector4d coeffs;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct FeaturePerFrame {
  int id;
  std::vector<unique_ptr<Feature>> features;
};

class FeatureManager {
 public:
  FeatureManager();
//  static void CalculateFeatures(const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_surf_from_map,
//                                const PointCloudPtr &local_surf_points_filtered_ptr,
//                                const PointCloudPtr &surf_stack,
//                                const Transform &local_transform,
//                                vector<PointPlaneFeature> &features);
//
//  static void CalculateLaserOdom(const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_surf_from_map,
//                                const PointCloudPtr &local_surf_points_filtered_ptr,
//                                const PointCloudPtr &surf_stack,
//                                const Transform &local_transform,
//                                vector<PointPlaneFeature> &features);
//
//  static void CalculateFeatures(const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_surf_from_map,
//                                const PointCloudPtr &local_surf_points_filtered_ptr,
//                                const PointCloudPtr &surf_stack,
//                                const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_corner_from_map,
//                                const PointCloudPtr &local_corner_points_filtered_ptr,
//                                const PointCloudPtr &corner_stack,
//                                const Transform &local_transform,
//                                vector<PointPlaneFeature> &features);
//
//  static void CalculateLaserOdom(const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_surf_from_map,
//                                 const PointCloudPtr &local_surf_points_filtered_ptr,
//                                 const PointCloudPtr &surf_stack,
//                                 const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_corner_from_map,
//                                 const PointCloudPtr &local_corner_points_filtered_ptr,
//                                 const PointCloudPtr &corner_stack,
//                                 const Transform &local_transform,
//                                 vector<PointPlaneFeature> &features);
};

}

#endif //LIO_FEATURE_MANAGER_H_

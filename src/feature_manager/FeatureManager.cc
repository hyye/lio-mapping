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

#include "feature_manager/FeatureManager.h"

namespace lio {

void PointNormalFeature::UpdateCovariance(const Eigen::Vector3d &normal3d_in) {
  ;
  Eigen::Matrix3d R = Eigen::Quaterniond::FromTwoVectors(e1, normal3d_in).toRotationMatrix();
  covariance = R * diag_covariance * R.transpose();
}

FeatureManager::FeatureManager() {

}

//void FeatureManager::CalculateFeatures(const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_surf_from_map,
//                                       const PointCloudPtr &local_surf_points_filtered_ptr,
//                                       const PointCloudPtr &surf_stack,
//                                       const Transform &local_transform,
//                                       vector<PointPlaneFeature> &features) {
//
//  PointT point_sel, point_ori, point_proj, coeff1, coeff2;
//
//  std::vector<int> point_search_idx(5, 0);
//  std::vector<float> point_search_sq_dis(5, 0);
//  Eigen::Matrix<float, 5, 3> mat_A0;
//  Eigen::Matrix<float, 5, 1> mat_B0;
//  Eigen::Vector3f mat_X0;
//  Eigen::Matrix3f mat_A1;
//  Eigen::Matrix<float, 1, 3> mat_D1;
//  Eigen::Matrix3f mat_V1;
//
//  mat_A0.setZero();
//  mat_B0.setConstant(-1);
//  mat_X0.setZero();
//
//  mat_A1.setZero();
//  mat_D1.setZero();
//  mat_V1.setZero();
//
//  PointCloud laser_cloud_ori;
//  PointCloud coeff_sel;
//  vector<float> scores;
//
//  const PointCloudPtr &origin_surf_points = surf_stack;
//  const Transform &transform_to_local = local_transform;
//  size_t surf_points_size = origin_surf_points->points.size();
//
////    DLOG(INFO) << "transform_to_local: " << transform_to_local;
//
//  for (int i = 0; i < surf_points_size; i++) {
//    point_ori = origin_surf_points->points[i];
//    PointMapping::PointAssociateToMap(point_ori, point_sel, transform_to_local);
//
//    int num_neighbors = 5;
//    kdtree_surf_from_map->nearestKSearch(point_sel, num_neighbors, point_search_idx, point_search_sq_dis);
//
//    if (point_search_sq_dis[num_neighbors - 1] < min_match_sq_dis_) {
//      for (int j = 0; j < num_neighbors; j++) {
//        mat_A0(j, 0) = local_surf_points_filtered_ptr->points[point_search_idx[j]].x;
//        mat_A0(j, 1) = local_surf_points_filtered_ptr->points[point_search_idx[j]].y;
//        mat_A0(j, 2) = local_surf_points_filtered_ptr->points[point_search_idx[j]].z;
//      }
//      mat_X0 = mat_A0.colPivHouseholderQr().solve(mat_B0);
//
//      float pa = mat_X0(0, 0);
//      float pb = mat_X0(1, 0);
//      float pc = mat_X0(2, 0);
//      float pd = 1;
//
//      float ps = sqrt(pa * pa + pb * pb + pc * pc);
//      pa /= ps;
//      pb /= ps;
//      pc /= ps;
//      pd /= ps;
//
//      // NOTE: plane as (x y z)*w+1 = 0
//
//      bool planeValid = true;
//      for (int j = 0; j < num_neighbors; j++) {
//        if (fabs(pa * local_surf_points_filtered_ptr->points[point_search_idx[j]].x +
//            pb * local_surf_points_filtered_ptr->points[point_search_idx[j]].y +
//            pc * local_surf_points_filtered_ptr->points[point_search_idx[j]].z + pd) > min_plane_dis_) {
//          planeValid = false;
//          break;
//        }
//      }
//
//      if (planeValid) {
//
//        float pd2 = pa * point_sel.x + pb * point_sel.y + pc * point_sel.z + pd;
//
//        float s = 1 - 0.9f * fabs(pd2) / sqrt(CalcPointDistance(point_sel));
//
//        coeff1.x = s * pa;
//        coeff1.y = s * pb;
//        coeff1.z = s * pc;
//        coeff1.intensity = s * pd;
//
//        bool is_in_laser_fov = false;
//        PointT transform_pos;
//        PointT point_on_z_axis;
//
//        point_on_z_axis.x = 0.0;
//        point_on_z_axis.y = 0.0;
//        point_on_z_axis.z = 10.0;
//        PointMapping::PointAssociateToMap(point_on_z_axis, point_on_z_axis, transform_to_local);
//
//        transform_pos.x = transform_to_local.pos.x();
//        transform_pos.y = transform_to_local.pos.y();
//        transform_pos.z = transform_to_local.pos.z();
//        float squared_side1 = CalcSquaredDiff(transform_pos, point_sel);
//        float squared_side2 = CalcSquaredDiff(point_on_z_axis, point_sel);
//
//        float check1 = 100.0f + squared_side1 - squared_side2
//            - 10.0f * sqrt(3.0f) * sqrt(squared_side1);
//
//        float check2 = 100.0f + squared_side1 - squared_side2
//            + 10.0f * sqrt(3.0f) * sqrt(squared_side1);
//
//        if (check1 < 0 && check2 > 0) { /// within +-60 degree
//          is_in_laser_fov = true;
//        }
//
//        if (s > 0.1 && is_in_laser_fov) {
//          PointPlaneFeature feature;
//          feature.score = s;
//          feature.point = Eigen::Vector3d{point_ori.x, point_ori.y, point_ori.z};
//          feature.coeffs = Eigen::Vector4d{coeff1.x, coeff1.y, coeff1.z, coeff1.intensity};
//          features.push_back(feature);
//        }
//      }
//    }
//  }
//
//}
//
//void FeatureManager::CalculateLaserOdom(const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_surf_from_map,
//                                        const PointCloudPtr &local_surf_points_filtered_ptr,
//                                        const PointCloudPtr &surf_stack,
//                                        const Transform &local_transform,
//                                        vector<PointPlaneFeature> &features) {
//// TODO: max_iter, delta_r, delta_t
//  bool is_degenerate = false;
//  for (size_t iter_count = 0; iter_count < 10; ++iter_count) {
//
//    CalculateFeatures(kdtree_surf_from_map, local_surf_points_filtered_ptr, surf_stack,
//                      local_transform, features);
//
//    size_t laser_cloud_sel_size = features.size();
//    Eigen::Matrix<float, Eigen::Dynamic, 6> mat_A(laser_cloud_sel_size, 6);
//    Eigen::Matrix<float, 6, Eigen::Dynamic> mat_At(6, laser_cloud_sel_size);
//    Eigen::Matrix<float, 6, 6> matAtA;
//    Eigen::VectorXf mat_B(laser_cloud_sel_size);
//    Eigen::VectorXf mat_AtB;
//    Eigen::VectorXf mat_X;
//    Eigen::Matrix<float, 6, 6> matP;
//
//    PointT point_sel, point_ori, coeff;
//
//    SO3 R_SO3(local_transform.rot); /// SO3
//
//    for (int i = 0; i < laser_cloud_sel_size; i++) {
//      point_ori.x = features[i].point.x();
//      point_ori.y = features[i].point.y();
//      point_ori.z = features[i].point.z();
//      coeff.x = features[i].coeffs.x();
//      coeff.y = features[i].coeffs.y();
//      coeff.z = features[i].coeffs.z();
//      coeff.intensity = features[i].coeffs.w();
//
//      Eigen::Vector3f p(point_ori.x, point_ori.y, point_ori.z);
//      Eigen::Vector3f w(coeff.x, coeff.y, coeff.z);
//
////      Eigen::Vector3f J_r = w.transpose() * RotationVectorJacobian(R_SO3, p);
//      Eigen::Vector3f J_r = -w.transpose() * (local_transform.rot * SkewSymmetric(p));
//      Eigen::Vector3f J_t = w.transpose();
//
//      float d2 = w.transpose() * (local_transform.rot * p + local_transform.pos) + coeff.intensity;
//
//      mat_A(i, 0) = J_r.x();
//      mat_A(i, 1) = J_r.y();
//      mat_A(i, 2) = J_r.z();
//      mat_A(i, 3) = J_t.x();
//      mat_A(i, 4) = J_t.y();
//      mat_A(i, 5) = J_t.z();
//      mat_B(i, 0) = -d2;
//    }
//
//    mat_At = mat_A.transpose();
//    matAtA = mat_At * mat_A;
//    mat_AtB = mat_At * mat_B;
//    mat_X = matAtA.colPivHouseholderQr().solve(mat_AtB);
//
//    if (iter_count == 0) {
//      Eigen::Matrix<float, 1, 6> mat_E;
//      Eigen::Matrix<float, 6, 6> mat_V;
//      Eigen::Matrix<float, 6, 6> mat_V2;
//
//      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 6, 6>> esolver(matAtA);
//      mat_E = esolver.eigenvalues().real();
//      mat_V = esolver.eigenvectors().real();
//
//      mat_V2 = mat_V;
//
//      is_degenerate = false;
//      float eignThre[6] = {100, 100, 100, 100, 100, 100};
//      for (int i = 0; i < 6; ++i) {
//        if (mat_E(0, i) < eignThre[i]) {
//          for (int j = 0; j < 6; ++j) {
//            mat_V2(i, j) = 0;
//          }
//          is_degenerate = true;
//          DLOG(WARNING) << "degenerate case";
//          DLOG(INFO) << mat_E;
//        } else {
//          break;
//        }
//      }
//      matP = mat_V2 * mat_V.inverse();
//    }
//
//    if (is_degenerate) {
//      Eigen::Matrix<float, 6, 1> matX2(mat_X);
//      mat_X = matP * matX2;
//    }
//
//    local_transform.pos.x() += mat_X(3, 0);
//    local_transform.pos.y() += mat_X(4, 0);
//    local_transform.pos.z() += mat_X(5, 0);
//
//    local_transform.rot = local_transform.rot * DeltaQ(Eigen::Vector3f(mat_X(0, 0), mat_X(1, 0), mat_X(2, 0)));
//
//    if (!isfinite(local_transform.pos.x())) local_transform.pos.x() = 0.0;
//    if (!isfinite(local_transform.pos.y())) local_transform.pos.y() = 0.0;
//    if (!isfinite(local_transform.pos.z())) local_transform.pos.z() = 0.0;
//
//    float delta_r = RadToDeg(R_SO3.unit_quaternion().angularDistance(local_transform.rot));
//    float delta_t = sqrt(pow(mat_X(3, 0) * 100, 2) + pow(mat_X(4, 0) * 100, 2) + pow(mat_X(5, 0) * 100, 2));
//
//    if (delta_r < 0.05 && delta_t < 0.1) {
//      DLOG(INFO) << "CalculateLaserOdom iter_count: " << iter_count;
//      break;
//    }
//  }
//}
//
//void FeatureManager::CalculateFeatures(const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_surf_from_map,
//                                       const PointCloudPtr &local_surf_points_filtered_ptr,
//                                       const PointCloudPtr &surf_stack,
//                                       const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_corner_from_map,
//                                       const PointCloudPtr &local_corner_points_filtered_ptr,
//                                       const PointCloudPtr &corner_stack,
//                                       const Transform &local_transform,
//                                       vector<PointPlaneFeature> &features) {
//  PointT point_sel, point_ori, point_proj, coeff1, coeff2;
//
//  std::vector<int> point_search_idx(5, 0);
//  std::vector<float> point_search_sq_dis(5, 0);
//  Eigen::Matrix<float, 5, 3> mat_A0;
//  Eigen::Matrix<float, 5, 1> mat_B0;
//  Eigen::Vector3f mat_X0;
//  Eigen::Matrix3f mat_A1;
//  Eigen::Matrix<float, 1, 3> mat_D1;
//  Eigen::Matrix3f mat_V1;
//
//  mat_A0.setZero();
//  mat_B0.setConstant(-1);
//  mat_X0.setZero();
//
//  mat_A1.setZero();
//  mat_D1.setZero();
//  mat_V1.setZero();
//
//  PointCloud laser_cloud_ori;
//  PointCloud coeff_sel;
//  vector<float> scores;
//
//  const PointCloudPtr &origin_surf_points = surf_stack;
//  const Transform &transform_to_local = local_transform;
//  size_t surf_points_size = origin_surf_points->points.size();
//
//  const PointCloudPtr &origin_corner_points = corner_stack;
//  size_t corner_points_size = origin_corner_points->points.size();
//
////    DLOG(INFO) << "transform_to_local: " << transform_to_local;
//
//  for (int i = 0; i < surf_points_size; i++) {
//    point_ori = origin_surf_points->points[i];
//    PointMapping::PointAssociateToMap(point_ori, point_sel, transform_to_local);
//
//    int num_neighbors = 5;
//    kdtree_surf_from_map->nearestKSearch(point_sel, num_neighbors, point_search_idx, point_search_sq_dis);
//
//    if (point_search_sq_dis[num_neighbors - 1] < min_match_sq_dis_) {
//      for (int j = 0; j < num_neighbors; j++) {
//        mat_A0(j, 0) = local_surf_points_filtered_ptr->points[point_search_idx[j]].x;
//        mat_A0(j, 1) = local_surf_points_filtered_ptr->points[point_search_idx[j]].y;
//        mat_A0(j, 2) = local_surf_points_filtered_ptr->points[point_search_idx[j]].z;
//      }
//      mat_X0 = mat_A0.colPivHouseholderQr().solve(mat_B0);
//
//      float pa = mat_X0(0, 0);
//      float pb = mat_X0(1, 0);
//      float pc = mat_X0(2, 0);
//      float pd = 1;
//
//      float ps = sqrt(pa * pa + pb * pb + pc * pc);
//      pa /= ps;
//      pb /= ps;
//      pc /= ps;
//      pd /= ps;
//
//      // NOTE: plane as (x y z)*w+1 = 0
//
//      bool planeValid = true;
//      for (int j = 0; j < num_neighbors; j++) {
//        if (fabs(pa * local_surf_points_filtered_ptr->points[point_search_idx[j]].x +
//            pb * local_surf_points_filtered_ptr->points[point_search_idx[j]].y +
//            pc * local_surf_points_filtered_ptr->points[point_search_idx[j]].z + pd) > min_plane_dis_) {
//          planeValid = false;
//          break;
//        }
//      }
//
//      if (planeValid) {
//
//        float pd2 = pa * point_sel.x + pb * point_sel.y + pc * point_sel.z + pd;
//
//        float s = 1 - 0.9f * fabs(pd2) / sqrt(CalcPointDistance(point_sel));
//
//        coeff1.x = s * pa;
//        coeff1.y = s * pb;
//        coeff1.z = s * pc;
//        coeff1.intensity = s * pd;
//
//        bool is_in_laser_fov = false;
//        PointT transform_pos;
//        PointT point_on_z_axis;
//
//        point_on_z_axis.x = 0.0;
//        point_on_z_axis.y = 0.0;
//        point_on_z_axis.z = 10.0;
//        PointMapping::PointAssociateToMap(point_on_z_axis, point_on_z_axis, transform_to_local);
//
//        transform_pos.x = transform_to_local.pos.x();
//        transform_pos.y = transform_to_local.pos.y();
//        transform_pos.z = transform_to_local.pos.z();
//        float squared_side1 = CalcSquaredDiff(transform_pos, point_sel);
//        float squared_side2 = CalcSquaredDiff(point_on_z_axis, point_sel);
//
//        float check1 = 100.0f + squared_side1 - squared_side2
//            - 10.0f * sqrt(3.0f) * sqrt(squared_side1);
//
//        float check2 = 100.0f + squared_side1 - squared_side2
//            + 10.0f * sqrt(3.0f) * sqrt(squared_side1);
//
//        if (check1 < 0 && check2 > 0) { /// within +-60 degree
//          is_in_laser_fov = true;
//        }
//
//        if (s > 0.1 && is_in_laser_fov) {
//          PointPlaneFeature feature;
//          feature.score = s;
//          feature.point = Eigen::Vector3d{point_ori.x, point_ori.y, point_ori.z};
//          feature.coeffs = Eigen::Vector4d{coeff1.x, coeff1.y, coeff1.z, coeff1.intensity};
//          features.push_back(feature);
//        }
//      }
//    }
//  }
//
//  //region Corner points
//  for (int i = 0; i < corner_points_size; i++) {
//    point_ori = origin_corner_points->points[i];
//    PointAssociateToMap(point_ori, point_sel, transform_to_local);
//    kdtree_corner_from_map->nearestKSearch(point_sel, 5, point_search_idx, point_search_sq_dis);
//
//    if (point_search_sq_dis[4] < min_match_sq_dis_) {
//      Eigen::Vector3f vc(0, 0, 0);
//
//      for (int j = 0; j < 5; j++) {
//        const PointT &point_sel_tmp = local_corner_points_filtered_ptr->points[point_search_idx[j]];
//        vc.x() += point_sel_tmp.x;
//        vc.y() += point_sel_tmp.y;
//        vc.z() += point_sel_tmp.z;
//      }
//      vc /= 5.0;
//
//      Eigen::Matrix3f mat_a;
//      mat_a.setZero();
//
//      for (int j = 0; j < 5; j++) {
//        const PointT &point_sel_tmp = local_corner_points_filtered_ptr->points[point_search_idx[j]];
//        Eigen::Vector3f a;
//        a.x() = point_sel_tmp.x - vc.x();
//        a.y() = point_sel_tmp.y - vc.y();
//        a.z() = point_sel_tmp.z - vc.z();
//
//        mat_a(0, 0) += a.x() * a.x();
//        mat_a(0, 1) += a.x() * a.y();
//        mat_a(0, 2) += a.x() * a.z();
//        mat_a(1, 1) += a.y() * a.y();
//        mat_a(1, 2) += a.y() * a.z();
//        mat_a(2, 2) += a.z() * a.z();
//      }
//      mat_A1 = mat_a / 5.0;
//
//      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esolver(mat_A1);
//      mat_D1 = esolver.eigenvalues().real();
//      mat_V1 = esolver.eigenvectors().real();
//
//      if (mat_D1(0, 2) > 3 * mat_D1(0, 1)) {
//
//        float x0 = point_sel.x;
//        float y0 = point_sel.y;
//        float z0 = point_sel.z;
//        float x1 = vc.x() + 0.1 * mat_V1(0, 2);
//        float y1 = vc.y() + 0.1 * mat_V1(1, 2);
//        float z1 = vc.z() + 0.1 * mat_V1(2, 2);
//        float x2 = vc.x() - 0.1 * mat_V1(0, 2);
//        float y2 = vc.y() - 0.1 * mat_V1(1, 2);
//        float z2 = vc.z() - 0.1 * mat_V1(2, 2);
//
//        Eigen::Vector3f X0(x0, y0, z0);
//        Eigen::Vector3f X1(x1, y1, z1);
//        Eigen::Vector3f X2(x2, y2, z2);
//
//        Eigen::Vector3f a012_vec = (X0 - X1).cross(X0 - X2);
//
//        Eigen::Vector3f normal_to_point = ((X1 - X2).cross(a012_vec)).normalized();
//
//        Eigen::Vector3f normal_cross_point = (X1 - X2).cross(normal_to_point);
//
//        float a012 = a012_vec.norm();
//
//        float l12 = (X1 - X2).norm();
//
//        float la = normal_to_point.x();
//        float lb = normal_to_point.y();
//        float lc = normal_to_point.z();
//
//        float ld2 = a012 / l12;
//
//        point_proj = point_sel;
//        point_proj.x -= la * ld2;
//        point_proj.y -= lb * ld2;
//        point_proj.z -= lc * ld2;
//
//        float ld_p1 = -(normal_to_point.x() * point_proj.x + normal_to_point.y() * point_proj.y
//            + normal_to_point.z() * point_proj.z);
//        float ld_p2 = -(normal_cross_point.x() * point_proj.x + normal_cross_point.y() * point_proj.y
//            + normal_cross_point.z() * point_proj.z);
//
//        float s = 1 - 0.9f * fabs(ld2);
//
//        coeff1.x = s * la;
//        coeff1.y = s * lb;
//        coeff1.z = s * lc;
//        coeff1.intensity = s * ld_p1;
//
//        coeff2.x = s * normal_cross_point.x();
//        coeff2.y = s * normal_cross_point.y();
//        coeff2.z = s * normal_cross_point.z();
//        coeff2.intensity = s * ld_p2;
//
//        bool is_in_laser_fov = false;
//        PointT transform_pos;
//        transform_pos.x = transform_tobe_mapped_.pos.x();
//        transform_pos.y = transform_tobe_mapped_.pos.y();
//        transform_pos.z = transform_tobe_mapped_.pos.z();
//        float squared_side1 = CalcSquaredDiff(transform_pos, point_sel);
//        float squared_side2 = CalcSquaredDiff(point_on_z_axis_, point_sel);
//
//        float check1 = 100.0f + squared_side1 - squared_side2
//            - 10.0f * sqrt(3.0f) * sqrt(squared_side1);
//
//        float check2 = 100.0f + squared_side1 - squared_side2
//            + 10.0f * sqrt(3.0f) * sqrt(squared_side1);
//
//        if (check1 < 0 && check2 > 0) { /// within +-60 degree
//          is_in_laser_fov = true;
//        }
//
//        if (s > 0.1 && is_in_laser_fov) {
//          PointPlaneFeature feature1;
//          feature1.score = s * 0.5;
//          feature1.point = Eigen::Vector3d{point_ori.x, point_ori.y, point_ori.z};
//          feature1.coeffs = Eigen::Vector4d{coeff1.x, coeff1.y, coeff1.z, coeff1.intensity} * 0.5;
//          features.push_back(feature1);
//
//          PointPlaneFeature feature2;
//          feature2.score = s * 0.5;
//          feature2.point = Eigen::Vector3d{point_ori.x, point_ori.y, point_ori.z};
//          feature2.coeffs = Eigen::Vector4d{coeff2.x, coeff2.y, coeff2.z, coeff2.intensity} * 0.5;
//          features.push_back(feature2);
//        }
//      }
//    }
//  }
//  //endregion
//
//}
//
//void FeatureManager::CalculateLaserOdom(const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_surf_from_map,
//                                        const PointCloudPtr &local_surf_points_filtered_ptr,
//                                        const PointCloudPtr &surf_stack,
//                                        const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_corner_from_map,
//                                        const PointCloudPtr &local_corner_points_filtered_ptr,
//                                        const PointCloudPtr &corner_stack,
//                                        const Transform &local_transform,
//                                        vector<PointPlaneFeature> &features) {
//  // TODO: max_iter, delta_r, delta_t
//  bool is_degenerate = false;
//  for (size_t iter_count = 0; iter_count < 10; ++iter_count) {
//
//    CalculateFeatures(kdtree_surf_from_map, local_surf_points_filtered_ptr, surf_stack,
//                      kdtree_corner_from_map, local_corner_points_filtered_ptr, corner_stack,
//                      local_transform, features);
//
//    size_t laser_cloud_sel_size = features.size();
//    Eigen::Matrix<float, Eigen::Dynamic, 6> mat_A(laser_cloud_sel_size, 6);
//    Eigen::Matrix<float, 6, Eigen::Dynamic> mat_At(6, laser_cloud_sel_size);
//    Eigen::Matrix<float, 6, 6> matAtA;
//    Eigen::VectorXf mat_B(laser_cloud_sel_size);
//    Eigen::VectorXf mat_AtB;
//    Eigen::VectorXf mat_X;
//    Eigen::Matrix<float, 6, 6> matP;
//
//    PointT point_sel, point_ori, coeff;
//
//    SO3 R_SO3(local_transform.rot); /// SO3
//
//    for (int i = 0; i < laser_cloud_sel_size; i++) {
//      point_ori.x = features[i].point.x();
//      point_ori.y = features[i].point.y();
//      point_ori.z = features[i].point.z();
//      coeff.x = features[i].coeffs.x();
//      coeff.y = features[i].coeffs.y();
//      coeff.z = features[i].coeffs.z();
//      coeff.intensity = features[i].coeffs.w();
//
//      Eigen::Vector3f p(point_ori.x, point_ori.y, point_ori.z);
//      Eigen::Vector3f w(coeff.x, coeff.y, coeff.z);
//
////      Eigen::Vector3f J_r = w.transpose() * RotationVectorJacobian(R_SO3, p);
//      Eigen::Vector3f J_r = -w.transpose() * (local_transform.rot * SkewSymmetric(p));
//      Eigen::Vector3f J_t = w.transpose();
//
//      float d2 = w.transpose() * (local_transform.rot * p + local_transform.pos) + coeff.intensity;
//
//      mat_A(i, 0) = J_r.x();
//      mat_A(i, 1) = J_r.y();
//      mat_A(i, 2) = J_r.z();
//      mat_A(i, 3) = J_t.x();
//      mat_A(i, 4) = J_t.y();
//      mat_A(i, 5) = J_t.z();
//      mat_B(i, 0) = -d2;
//    }
//
//    mat_At = mat_A.transpose();
//    matAtA = mat_At * mat_A;
//    mat_AtB = mat_At * mat_B;
//    mat_X = matAtA.colPivHouseholderQr().solve(mat_AtB);
//
//    if (iter_count == 0) {
//      Eigen::Matrix<float, 1, 6> mat_E;
//      Eigen::Matrix<float, 6, 6> mat_V;
//      Eigen::Matrix<float, 6, 6> mat_V2;
//
//      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 6, 6>> esolver(matAtA);
//      mat_E = esolver.eigenvalues().real();
//      mat_V = esolver.eigenvectors().real();
//
//      mat_V2 = mat_V;
//
//      is_degenerate = false;
//      float eignThre[6] = {100, 100, 100, 100, 100, 100};
//      for (int i = 0; i < 6; ++i) {
//        if (mat_E(0, i) < eignThre[i]) {
//          for (int j = 0; j < 6; ++j) {
//            mat_V2(i, j) = 0;
//          }
//          is_degenerate = true;
//          DLOG(WARNING) << "degenerate case";
//          DLOG(INFO) << mat_E;
//        } else {
//          break;
//        }
//      }
//      matP = mat_V2 * mat_V.inverse();
//    }
//
//    if (is_degenerate) {
//      Eigen::Matrix<float, 6, 1> matX2(mat_X);
//      mat_X = matP * matX2;
//    }
//
//    local_transform.pos.x() += mat_X(3, 0);
//    local_transform.pos.y() += mat_X(4, 0);
//    local_transform.pos.z() += mat_X(5, 0);
//
//    local_transform.rot = local_transform.rot * DeltaQ(Eigen::Vector3f(mat_X(0, 0), mat_X(1, 0), mat_X(2, 0)));
//
//    if (!isfinite(local_transform.pos.x())) local_transform.pos.x() = 0.0;
//    if (!isfinite(local_transform.pos.y())) local_transform.pos.y() = 0.0;
//    if (!isfinite(local_transform.pos.z())) local_transform.pos.z() = 0.0;
//
//    float delta_r = RadToDeg(R_SO3.unit_quaternion().angularDistance(local_transform.rot));
//    float delta_t = sqrt(pow(mat_X(3, 0) * 100, 2) + pow(mat_X(4, 0) * 100, 2) + pow(mat_X(5, 0) * 100, 2));
//
//    if (delta_r < 0.05 && delta_t < 0.1) {
//      DLOG(INFO) << "CalculateLaserOdom iter_count: " << iter_count;
//      break;
//    }
//  }
//}

}
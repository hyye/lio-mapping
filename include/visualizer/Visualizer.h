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
// Created by hyye on 4/7/18.
//

#ifndef LIO_VISUALIZER_H_
#define LIO_VISUALIZER_H_

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <map>
#include "point_processor/PointMapping.h"

#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

namespace lio {

class Visualizer {
 public:
  Visualizer(std::string vis_name = "visualizer",
             std::vector<double> imu_color = {0.0, 1.0, 0.0},
             std::vector<double> lidar_color = {1.0, 1.0, 0.0});

  void UpdateMarker(visualization_msgs::Marker &marker, const Transform &pose);
  void UpdateMarkers(std::vector<Transform> imu_poses, std::vector<Transform> lidar_poses);
  void UpdateVelocity(double velocity);
  void PublishMarkers();

  ros::NodeHandle nh_;
  ros::Publisher imu_vis_pub_, lidar_vis_pub_, velocity_vis_pub_;

  tf::TransformBroadcaster br_;
  visualization_msgs::Marker imu_marker_, lidar_marker_, velocity_marker_;

  visualization_msgs::MarkerArray imu_markers_;
  visualization_msgs::MarkerArray lidar_markers_;

  std::string vis_name_;

};

class PlaneNormalVisualizer {
 public:
  PlaneNormalVisualizer();
  void Spin();
  void UpdateCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                   std::string cloud_name = "cloud",
                   std::vector<double> cloud_color = {1.0, 0.0, 1.0});

  void UpdateCloudAndNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                             pcl::PointCloud<pcl::Normal>::ConstPtr normals,
                             int ds_ratio = 10,
                             std::string cloud_name = "cloud",
                             std::string normals_name = "normals",
                             std::vector<double> cloud_color = {1.0, 1.0, 1.0},
                             std::vector<double> normals_color = {1.0, 1.0, 0.0});

  void UpdateLines(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1,
                   pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2,
                   std::vector<double> line_color = {0.0, 1.0, 0.0});

  void UpdatePlanes(const std::vector<Eigen::Vector4d,
                                      Eigen::aligned_allocator<Eigen::Vector4d>> &plane_coeffs);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//  pcl::visualization::PCLVisualizer* viewer;
  boost::mutex m;
  bool init = false;
  bool first = false;

  std::vector<std::string> line_names;
  std::vector<std::string> plane_names;
};

template<typename PointT>
void CoeffsToVisualizationMsgs(const multimap<float, pair<PointT, PointT>, greater<float> > &spc_map,
                               const Transform &transform,
                               visualization_msgs::MarkerArray &marker_array) {
  marker_array.markers.clear();

  visualization_msgs::Marker marker;
  {
    marker.header.frame_id = "/camera_init";
    marker.id = 0;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.5;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.ns = "plane";
  }

  typedef multimap<float, pair<PointT, PointT>, greater<float> > ScorePointCoeffMap;
  int j;
  typename ScorePointCoeffMap::const_iterator it_point_coeff;

  for (j = 0, it_point_coeff = spc_map.begin(); it_point_coeff != spc_map.end();
       ++j, ++it_point_coeff) {
    if (j % 10 == 0) {

      const double &s = it_point_coeff->first;
      const PointT &p = it_point_coeff->second.first;
      const PointT &coeff = it_point_coeff->second.second;

      Eigen::Vector4d p_eigen(p.x, p.y, p.z, 1.0);

      p_eigen = (transform.transform().matrix().cast<double>() * p_eigen).eval();

      Eigen::Vector4d coeff_eigen(coeff.x, coeff.y, coeff.z, coeff.intensity);
      double dis = p_eigen.dot(coeff_eigen);
      Eigen::Vector3d proj_point = p_eigen.head<3>() - dis * coeff_eigen.head<3>();
      Eigen::Quaterniond proj_ori =
          Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(1.0, 0.0, 0.0), coeff_eigen.head<3>()).normalized();

      marker.pose.position.x = proj_point.x();
      marker.pose.position.y = proj_point.y();
      marker.pose.position.z = proj_point.z();

      marker.pose.orientation.x = proj_ori.x();
      marker.pose.orientation.y = proj_ori.y();
      marker.pose.orientation.z = proj_ori.z();
      marker.pose.orientation.w = proj_ori.w();

      DLOG(INFO) << coeff_eigen.head<3>().transpose();
      DLOG(INFO) << proj_ori.coeffs().transpose();

      marker_array.markers.push_back(marker);

      marker.id += 1;

      if (marker.id >= 200) {
        break;
      }

    }

  }
//  DLOG(INFO) << "spc_map.size(): " << spc_map.size() << " j: " << j;

  while (marker.id < 200) {
    marker.id += 1;
    marker_array.markers.push_back(marker);
  }

}

template<typename PointT>
void CoeffsToCloudNormal(const multimap<float, pair<PointT, PointT>, greater<float> > &spc_map,
                         const Transform &transform,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                         pcl::PointCloud<pcl::Normal>::Ptr &normals) {
  cloud->clear();
  normals->clear();

  typedef multimap<float, pair<PointT, PointT>, greater<float> > ScorePointCoeffMap;
  int j;
  typename ScorePointCoeffMap::const_iterator it_point_coeff;

  for (j = 0, it_point_coeff = spc_map.begin(); it_point_coeff != spc_map.end();
       ++j, ++it_point_coeff) {

    const double &s = it_point_coeff->first;
    const PointT &p = it_point_coeff->second.first;
    const PointT &coeff = it_point_coeff->second.second;

    Eigen::Vector4d p_eigen(p.x, p.y, p.z, 1.0);

    p_eigen = (transform.transform().matrix().cast<double>() * p_eigen).eval();

    Eigen::Vector4d coeff_eigen(coeff.x, coeff.y, coeff.z, coeff.intensity);
    double dis = p_eigen.dot(coeff_eigen);
    Eigen::Vector3d proj_point = p_eigen.head<3>() - dis * coeff_eigen.head<3>();
    Eigen::Quaterniond proj_ori =
        Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(1.0, 0.0, 0.0), coeff_eigen.head<3>()).normalized();

    if (coeff_eigen.head<3>().dot(proj_point) > 0) {
      coeff_eigen.head<3>() = -(coeff_eigen.head<3>()).eval();
    }

    pcl::Normal normal_world(coeff_eigen.x(), coeff_eigen.y(), coeff_eigen.z());
    pcl::PointXYZ p_world;
    p_world.x = proj_point.x();
    p_world.y = proj_point.y();
    p_world.z = proj_point.z();
    cloud->push_back(p_world);

    normals->push_back(normal_world);
  }
}

} // namespace lio

#endif //LIO_VISUALIZER_H_

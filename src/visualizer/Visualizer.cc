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

#include "visualizer/Visualizer.h"

namespace lio {

Visualizer::Visualizer(std::string vis_name,
                       std::vector<double> imu_color,
                       std::vector<double> lidar_color) {
  // initial
  vis_name_ = vis_name;
  nh_ = ros::NodeHandle(vis_name_);
  imu_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("imu_markers", 10);
  lidar_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("lidar_markers", 10);
  velocity_vis_pub_ = nh_.advertise<visualization_msgs::Marker>("velocity_marker", 10);

//  un_imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/un_imu/data", 10);
//  vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/twist_test", 10);

  imu_marker_.header.frame_id = "/world";
  imu_marker_.header.stamp = ros::Time();
  imu_marker_.ns = "imu_marker";
  imu_marker_.type = visualization_msgs::Marker::ARROW;
  imu_marker_.action = visualization_msgs::Marker::ADD;
  imu_marker_.pose.position.x = 0;
  imu_marker_.pose.position.y = 0;
  imu_marker_.pose.position.z = 0;
  imu_marker_.pose.orientation.x = 0.0;
  imu_marker_.pose.orientation.y = 0.0;
  imu_marker_.pose.orientation.z = 0.0;
  imu_marker_.pose.orientation.w = 1.0;
  imu_marker_.scale.x = 0.2;
  imu_marker_.scale.y = 0.05;
  imu_marker_.scale.z = 0.05;
  imu_marker_.color.a = 1.0; // Don't forget to set the alpha!
  imu_marker_.color.r = imu_color[0];
  imu_marker_.color.g = imu_color[1];
  imu_marker_.color.b = imu_color[2];

  lidar_marker_.header.frame_id = "/world";
  lidar_marker_.header.stamp = ros::Time();
  lidar_marker_.ns = "lidar_markers";
  lidar_marker_.type = visualization_msgs::Marker::ARROW;
  lidar_marker_.action = visualization_msgs::Marker::ADD;
  lidar_marker_.pose.position.x = 0;
  lidar_marker_.pose.position.y = 0;
  lidar_marker_.pose.position.z = 0;
  lidar_marker_.pose.orientation.x = 0.0;
  lidar_marker_.pose.orientation.y = 0.0;
  lidar_marker_.pose.orientation.z = 0.0;
  lidar_marker_.pose.orientation.w = 1.0;
  lidar_marker_.scale.x = 0.2;
  lidar_marker_.scale.y = 0.05;
  lidar_marker_.scale.z = 0.05;
  lidar_marker_.color.a = 1.0; // Don't forget to set the alpha!
  lidar_marker_.color.r = lidar_color[0];
  lidar_marker_.color.g = lidar_color[1];
  lidar_marker_.color.b = lidar_color[2];

  velocity_marker_.header.frame_id = "/world";
  velocity_marker_.header.stamp = ros::Time();
  velocity_marker_.ns = "velocity_marker";
  velocity_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  velocity_marker_.action = visualization_msgs::Marker::ADD;
  velocity_marker_.pose.position.x = 0;
  velocity_marker_.pose.position.y = 0;
  velocity_marker_.pose.position.z = 0;
  velocity_marker_.pose.orientation.x = 0.0;
  velocity_marker_.pose.orientation.y = 0.0;
  velocity_marker_.pose.orientation.z = 0.0;
  velocity_marker_.pose.orientation.w = 1.0;
  velocity_marker_.scale.x = 5;
  velocity_marker_.scale.y = 5;
  velocity_marker_.scale.z = 5;
  velocity_marker_.color.a = 1.0; // Don't forget to set the alpha!
  velocity_marker_.color.r = 1;
  velocity_marker_.color.g = 1;
  velocity_marker_.color.b = 0;
}

void Visualizer::UpdateMarker(visualization_msgs::Marker &marker, const lio::Transform &pose) {
  marker.pose.position.x = pose.pos.x();
  marker.pose.position.y = pose.pos.y();
  marker.pose.position.z = pose.pos.z();
  marker.pose.orientation.x = pose.rot.x();
  marker.pose.orientation.y = pose.rot.y();
  marker.pose.orientation.z = pose.rot.z();
  marker.pose.orientation.w = pose.rot.w();
}

void Visualizer::UpdateMarkers(std::vector<lio::Transform> imu_poses, std::vector<lio::Transform> lidar_poses) {
  imu_markers_.markers.clear();
  lidar_markers_.markers.clear();
  imu_marker_.color.a = 1.0;
  lidar_marker_.color.a = 1.0;
  for (int i = int(imu_poses.size() - 1); i >= 0; --i) {
    imu_marker_.id = i;
    imu_marker_.color.a *= 0.9;
    lidar_marker_.id = i;
    lidar_marker_.color.a *= 0.9;
    UpdateMarker(imu_marker_, imu_poses[i]);
    UpdateMarker(lidar_marker_, lidar_poses[i]);
    imu_markers_.markers.push_back(imu_marker_);
    lidar_markers_.markers.push_back(lidar_marker_);
  }
}

void Visualizer::UpdateVelocity(double velocity) {
  if (!imu_markers_.markers.empty()) {
    velocity_marker_.id = 0;
    velocity_marker_.pose.position = imu_markers_.markers.front().pose.position;
    velocity_marker_.pose.position.y += 15;
    velocity_marker_.pose.position.z += 15;
    std::string output_text = std::to_string(velocity * 3.6) + " km/h";
    velocity_marker_.text = output_text;
  }
}

void Visualizer::PublishMarkers() {
  imu_vis_pub_.publish(imu_markers_);
  lidar_vis_pub_.publish(lidar_markers_);
  velocity_vis_pub_.publish(velocity_marker_);
}

void PlaneNormalVisualizer::UpdateCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                        std::string cloud_name,
                                        std::vector<double> cloud_color) {
  boost::mutex::scoped_lock lk(m);

//  DLOG(INFO) << ">>>>>>> update <<<<<<<";
//  DLOG(INFO) << cloud->size();

  if (cloud->size() == 0) {
    DLOG(INFO) << ">>>>>>> no points <<<<<<<";
    return;
  }

  if (!viewer->updatePointCloud(cloud, cloud_name)) {
    viewer->addPointCloud<pcl::PointXYZ>(cloud, cloud_name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud_name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                             cloud_color[0],
                                             cloud_color[1],
                                             cloud_color[2],
                                             cloud_name);
  }

}

void PlaneNormalVisualizer::UpdateCloudAndNormals(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                                                  pcl::PointCloud<pcl::Normal>::ConstPtr normals,
                                                  int ds_ratio,
                                                  std::string cloud_name,
                                                  std::string normals_name,
                                                  std::vector<double> cloud_color,
                                                  std::vector<double> normals_color) {
  boost::mutex::scoped_lock lk(m);

//  DLOG(INFO) << ">>>>>>> update <<<<<<<";
//  DLOG(INFO) << cloud->size();
//  DLOG(INFO) << normals->size();

  if (cloud->size() == 0 || normals->size() == 0) {
    DLOG(INFO) << ">>>>>>> no points <<<<<<<";
    return;
  }

  if (!viewer->updatePointCloud(cloud, cloud_name)) {
    viewer->addPointCloud<pcl::PointXYZ>(cloud, cloud_name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud_name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                             cloud_color[0],
                                             cloud_color[1],
                                             cloud_color[2],
                                             cloud_name);
  }
  viewer->removePointCloud(normals_name, 0);
  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, normals, ds_ratio, 0.5, normals_name);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                           normals_color[0],
                                           normals_color[1],
                                           normals_color[2],
                                           normals_name);

}

void PlaneNormalVisualizer::UpdateLines(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1,
                                        pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2,
                                        std::vector<double> line_color) {
  boost::mutex::scoped_lock lk(m);

//  DLOG(INFO) << ">>>>>>> update <<<<<<<";

  int num_cloud1 = cloud1->size();
  int num_cloud2 = cloud2->size();
  if (num_cloud1 == 0 || num_cloud2 == 0 || num_cloud1 != num_cloud2) {
    DLOG(INFO) << ">>>>>>> no points or sizes are not the same <<<<<<<";
    LOG_IF(INFO, num_cloud1 != num_cloud2) << num_cloud1 << " != " << num_cloud2;
    return;
  }

  for (const auto line_name : line_names) {
    viewer->removeShape(line_name);
  }

  line_names.clear();

  for (int i = 0; i < num_cloud1; ++i) {
    std::stringstream line_name_ss;
    line_name_ss << "line" << i;
    std::string line_name = line_name_ss.str();
    viewer->addLine(cloud1->at(i), cloud2->at(i), line_name);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                        line_color[0],
                                        line_color[1],
                                        line_color[2],
                                        line_name);
    line_names.push_back(line_name);
  }

}

void PlaneNormalVisualizer::UpdatePlanes(const std::vector<Eigen::Vector4d,
                                                           Eigen::aligned_allocator<Eigen::Vector4d>> &plane_coeffs) {

  boost::mutex::scoped_lock lk(m);

//  DLOG(INFO) << ">>>>>>> update <<<<<<<";

  int size = plane_coeffs.size();
  if (size == 0) {
    DLOG(INFO) << ">>>>>>> no planes <<<<<<<";
    return;
  }

  for (const auto plane_name : plane_names) {
    viewer->removeShape(plane_name);
  }

  plane_names.clear();

  for (int i = 0; i < size; ++i) {
    Eigen::Vector4d coeffs_eigen = plane_coeffs[i];
    std::stringstream plane_name_ss;
    plane_name_ss << "plane" << i;
    std::string plane_name = plane_name_ss.str();
    pcl::ModelCoefficients coeffs;
    coeffs.values.push_back(coeffs_eigen.x());
    coeffs.values.push_back(coeffs_eigen.y());
    coeffs.values.push_back(coeffs_eigen.z());
    coeffs.values.push_back(coeffs_eigen.w());
    viewer->addPlane(coeffs, plane_name);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.01, plane_name);
    plane_names.push_back(plane_name);
  }
}

PlaneNormalVisualizer::PlaneNormalVisualizer() {
//  boost::mutex::scoped_lock lk(m);
//  viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("3D Debug Viewer");
//  viewer->setBackgroundColor(0, 0, 0);
//  viewer->addCoordinateSystem(1.0);
//  viewer->addText("debugger by Kitkat7", 10, 10, "debugger text", 0);
//  viewer->initCameraParameters();
//  init = true;
}

void PlaneNormalVisualizer::Spin() {

  {
    boost::mutex::scoped_lock lk(m);
    viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("3D Debug Viewer");
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->addText("debugger by Kitkat7", 10, 10, "debugger text", 0);
    viewer->initCameraParameters();
    init = true;
  }

  while (!viewer->wasStopped()) {
    {
      boost::mutex::scoped_lock lk(m);
//      DLOG(INFO) << ">>>>>>> spin <<<<<<<";
      viewer->spinOnce(100);
    }
    boost::this_thread::sleep(boost::posix_time::microseconds(10000));
  }
}

} // namespace lio

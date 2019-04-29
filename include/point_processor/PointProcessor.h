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
// Created by hyye on 3/14/18.
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

#ifndef LIO_POINTPROCESSOR_H_
#define LIO_POINTPROCESSOR_H_

#include <glog/logging.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/features/normal_3d.h>

#include "utils/common_ros.h"
#include "utils/TicToc.h"
#include "utils/math_utils.h"

#include "utils/CircularBuffer.h"

#include "point_types.h"

#include <std_msgs/Float32.h>

namespace lio {

using namespace std;
using namespace mathutils;
typedef pcl::PointXYZI PointT;
typedef typename pcl::PointCloud<PointT> PointCloud;
typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef typename pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;

typedef lio::PointXYZIR PointIR;

typedef std::pair<size_t, size_t> IndexRange;

// adapted from LOAM
/** Point label options. */
enum PointLabel {
  CORNER_SHARP = 2,       ///< sharp corner point
  CORNER_LESS_SHARP = 1,  ///< less sharp corner point
  SURFACE_LESS_FLAT = 0,  ///< less flat surface point
  SURFACE_FLAT = -1       ///< flat surface point
};

struct PointProcessorConfig {
  bool deskew = false;
  double scan_period = 0.1;
  int num_scan_subregions = 8;
  int num_curvature_regions = 5;
  float surf_curv_th = 0.1;
  int max_corner_sharp = 2;
  int max_corner_less_sharp = 10 * max_corner_sharp;
  int max_surf_flat = 4;
  float less_flat_filter_size = 0.2;

  string capture_frame_id = "/map";

  double rad_diff = 0.2;

  bool infer_start_ori_ = false;
};

class PointProcessor {

 public:

  PointProcessor();
  PointProcessor(float lower_bound, float upper_bound, int num_rings, bool uneven = false);

  // WARNING: is it useful to separate Process and SetInputCloud?
  // void Process(const PointCloudConstPtr &cloud_in, PointCloud &cloud_out);
  void Process();

  void PointCloudHandler(const sensor_msgs::PointCloud2ConstPtr &raw_points_msg);

  void SetupConfig(PointProcessorConfig config) {
    config_ = config;
  }

  void SetupRos(ros::NodeHandle &nh);

  void SetInputCloud(const PointCloudConstPtr &cloud_in, ros::Time time_in = ros::Time::now());
  void SetInputCloud(const pcl::PointCloud<PointIR>::Ptr &cloud_in, ros::Time time_in = ros::Time::now());

  void PointToRing();
  void PointToRing(const PointCloudConstPtr &cloud_in,
                   vector<PointCloudPtr> &ring_out,
                   vector<PointCloudPtr> &intensity_out);

  void PointToRing(const pcl::PointCloud<PointIR>::Ptr &cloud_in,
                   vector<PointCloudPtr> &ring_out,
                   vector<PointCloudPtr> &intensity_out);

  inline int ElevationToRing(float rad) {
    double in = (RadToDeg(rad) - lower_bound_) * factor_ + 0.5;
    return int((RadToDeg(rad) - lower_bound_) * factor_ + 0.5);
  }

  void SetDeskew(bool deskew);

  // TODO: Add IMU compensation, by quaternion interpolation
  // TODO: To implement, for better feature extraction
  // WARNING: Adding IMU here will require adding IMU in the odometry part
  void DeSkew(PointT &point, double rel_time);

  void ExtractFeaturePoints();

  void PublishResults();

  // TODO: not necessary data?
  vector<PointCloudPtr> laser_scans;
  vector<PointCloudPtr> intensity_scans;
  vector<IndexRange> scan_ranges;

 protected:

  ros::Time sweep_start_;
  ros::Time scan_time_;

  float lower_bound_;
  float upper_bound_;
  int num_rings_;
  float factor_;
  PointProcessorConfig config_;
  TicToc tic_toc_;

  PointCloudConstPtr cloud_ptr_;
  pcl::PointCloud<PointIR>::Ptr cloud_ir_ptr_;

  PointCloud cloud_in_rings_;

  PointCloud corner_points_sharp_;
  PointCloud corner_points_less_sharp_;
  PointCloud surface_points_flat_;
  PointCloud surface_points_less_flat_;

  // the following will be assigened or resized
  vector<int> scan_ring_mask_;
  vector<pair<float, size_t> > curvature_idx_pairs_; // in subregion
  vector<PointLabel> subregion_labels_;     ///< point label buffer

//  void PrepareRing(const size_t idx_ring);
//  void PrepareSubregion(const size_t idx_ring, const size_t idx_start, const size_t idx_end);
//  void MaskPickedInRing(const size_t idx_ring, const size_t in_scan_idx);

  void Reset(const ros::Time &scan_time, const bool &is_new_sweep = true);
  void PrepareRing(const PointCloud &scan);
  void PrepareSubregion(const PointCloud &scan, const size_t idx_start, const size_t idx_end);
  void MaskPickedInRing(const PointCloud &scan, const size_t in_scan_idx);

  ros::Subscriber sub_raw_points_;   ///< input cloud message subscriber

  ros::Publisher pub_full_cloud_;              ///< full resolution cloud message publisher
  ros::Publisher pub_corner_points_sharp_;       ///< sharp corner cloud message publisher
  ros::Publisher pub_corner_points_less_sharp_;   ///< less sharp corner cloud message publisher
  ros::Publisher pub_surf_points_flat_;          ///< flat surface cloud message publisher
  ros::Publisher pub_surf_points_less_flat_;      ///< less flat surface cloud message publisher

  bool is_ros_setup_ = false;

  bool uneven_ = false;

 private:
  float start_ori_, end_ori_;
  CircularBuffer<float> start_ori_buf1_{10};
  CircularBuffer<float> start_ori_buf2_{10};
  ros::Publisher pub_start_ori_;
  ros::Publisher pub_start_ori_inferred_;

};

} // namespace lio

#endif //LIO_POINTPROCESSOR_H_

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
// Created by hyye on 3/27/18.
//

#ifndef LIO_ESTIMATOR_H_
#define LIO_ESTIMATOR_H_

#include<Eigen/StdVector>

#include "imu_processor/MeasurementManager.h"
#include "imu_processor/IntegrationBase.h"
#include "imu_processor/ImuInitializer.h"
#include "point_processor/PointMapping.h"

#include "factor/PoseLocalParameterization.h"
#include "factor/GravityLocalParameterization.h"
#include "factor/ImuFactor.h"
#include "factor/PointDistanceFactor.h"
#include "factor/PlaneProjectionFactor.h"
#include "factor/PriorFactor.h"
#include "factor/MarginalizationFactor.h"
#include "factor/PlaneToPlaneFactor.h"
#include "factor/PivotPointPlaneFactor.h"

#include <std_srvs/SetBool.h>

#include "visualizer/Visualizer.h"

//#define FIX_MAP
//#define USE_CORNER

namespace lio {

using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;

typedef multimap<float, pair<PointT, PointT>, greater<float> > ScorePointCoeffMap;

enum EstimatorStageFlag {
  NOT_INITED,
  INITED,
};

struct StampedTransform {
  double time;
  Transform transform;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EstimatorConfig {
  size_t window_size = 15;
  size_t opt_window_size = 5;
  int init_window_factor = 3;
  int estimate_extrinsic = 2;

  float corner_filter_size = 0.2;
  float surf_filter_size = 0.4;
  float map_filter_size = 0.6;

  float min_match_sq_dis = 1.0;
  float min_plane_dis = 0.2;
  Transform transform_lb{Eigen::Quaternionf(1, 0, 0, 0), Eigen::Vector3f(0, 0, -0.1)};

  bool opt_extrinsic = false;

  bool run_optimization = true;
  bool update_laser_imu = true;
  bool gravity_fix = true;
  bool plane_projection_factor = true;
  bool imu_factor = true;
  bool point_distance_factor = false;
  bool prior_factor = false;
  bool marginalization_factor = true;
  bool pcl_viewer = false;

  bool enable_deskew = true; ///< if disable, deskew from PointOdometry will be used
  bool cutoff_deskew = false;
  bool keep_features = false;

  IntegrationBaseConfig pim_config;
};

class Estimator : public MeasurementManager, public PointMapping {
 public:
  Estimator();
  Estimator(EstimatorConfig config, MeasurementManagerConfig mm_config = MeasurementManagerConfig());
  ~Estimator();
  void ClearState();
  void SetupRos(ros::NodeHandle &nh);

  void SetupAllEstimatorConfig(const EstimatorConfig &config,
                               const MeasurementManagerConfig &mm_config);

  void ProcessEstimation();
  void ProcessImu(double dt,
                  const Vector3d &linear_acceleration,
                  const Vector3d &angular_velocity,
                  const std_msgs::Header &header);
  void ProcessLaserOdom(const Transform &transform_in, const std_msgs::Header &header);
  void ProcessCompactData(const sensor_msgs::PointCloud2ConstPtr &compact_data, const std_msgs::Header &header);

  void BuildLocalMap(vector<FeaturePerFrame> &feature_frames);

#ifdef USE_CORNER
  void CalculateFeatures(const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_surf_from_map,
                         const PointCloudPtr &local_surf_points_filtered_ptr,
                         const PointCloudPtr &surf_stack,
                         const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_corner_from_map,
                         const PointCloudPtr &local_corner_points_filtered_ptr,
                         const PointCloudPtr &corner_stack,
                         const Transform &local_transform,
                         vector<unique_ptr<Feature>> &features);

  void CalculateLaserOdom(const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_surf_from_map,
                          const PointCloudPtr &local_surf_points_filtered_ptr,
                          const PointCloudPtr &surf_stack,
                          const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_corner_from_map,
                          const PointCloudPtr &local_corner_points_filtered_ptr,
                          const PointCloudPtr &corner_stack,
                          Transform &local_transform,
                          vector<unique_ptr<Feature>> &features);
#else
  void CalculateFeatures(const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_surf_from_map,
                         const PointCloudPtr &local_surf_points_filtered_ptr,
                         const PointCloudPtr &surf_stack,
                         const Transform &local_transform,
                         vector<unique_ptr<Feature>> &features);

  void CalculateLaserOdom(const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_surf_from_map,
                          const PointCloudPtr &local_surf_points_filtered_ptr,
                          const PointCloudPtr &surf_stack,
                          Transform &local_transform,
                          vector<unique_ptr<Feature>> &features);
#endif

  void SolveOptimization();

  void SlideWindow();

  void VectorToDouble();
  void DoubleToVector();

  bool RunInitialization();

  PlaneNormalVisualizer normal_vis;

  int extrinsic_stage_ = 2;

  PointCloudPtr local_surf_points_ptr_, local_surf_points_filtered_ptr_;
  PointCloudPtr local_corner_points_ptr_, local_corner_points_filtered_ptr_;

 protected:
  EstimatorStageFlag stage_flag_ = NOT_INITED;
  EstimatorConfig estimator_config_;

  bool first_imu_ = false;
  double initial_time_ = -1;

  size_t cir_buf_count_ = 0;
  size_t laser_odom_recv_count_ = 0;

  Vector3d acc_last_, gyr_last_;
  Vector3d g_vec_;

  shared_ptr<IntegrationBase> tmp_pre_integration_;

  //  map<double, LaserFrame> all_laser_frames;

  CircularBuffer<PairTimeLaserTransform> all_laser_transforms_{estimator_config_.window_size + 1};

  CircularBuffer<Vector3d> Ps_{estimator_config_.window_size + 1};
  CircularBuffer<Matrix3d> Rs_{estimator_config_.window_size + 1};
  CircularBuffer<Vector3d> Vs_{estimator_config_.window_size + 1};
  CircularBuffer<Vector3d> Bas_{estimator_config_.window_size + 1};
  CircularBuffer<Vector3d> Bgs_{estimator_config_.window_size + 1};

  //region fix the map
#ifdef FIX_MAP
  CircularBuffer<Vector3d> Ps_linearized_{estimator_config_.window_size + 1};
  CircularBuffer<Matrix3d> Rs_linearized_{estimator_config_.window_size + 1};
#endif
  CircularBuffer<size_t> size_surf_stack_{estimator_config_.window_size + 1};
  CircularBuffer<size_t> size_corner_stack_{estimator_config_.window_size + 1};
  bool init_local_map_ = false;
  //endregion

  CircularBuffer<std_msgs::Header> Headers_{estimator_config_.window_size + 1};

  CircularBuffer<vector<double> > dt_buf_{estimator_config_.window_size + 1};
  CircularBuffer<vector<Vector3d> > linear_acceleration_buf_{estimator_config_.window_size + 1};
  CircularBuffer<vector<Vector3d> > angular_velocity_buf_{estimator_config_.window_size + 1};

  CircularBuffer<shared_ptr<IntegrationBase> > pre_integrations_{estimator_config_.window_size + 1};
  CircularBuffer<PointCloudPtr> surf_stack_{estimator_config_.window_size + 1};
  CircularBuffer<PointCloudPtr> corner_stack_{estimator_config_.window_size + 1};
  CircularBuffer<PointCloudPtr> full_stack_{estimator_config_.window_size + 1};

  ///> optimization buffers
  CircularBuffer<bool> opt_point_coeff_mask_{estimator_config_.opt_window_size + 1};
  CircularBuffer<ScorePointCoeffMap> opt_point_coeff_map_{estimator_config_.opt_window_size + 1};
  CircularBuffer<CubeCenter> opt_cube_centers_{estimator_config_.opt_window_size + 1};
  CircularBuffer<Transform> opt_transforms_{estimator_config_.opt_window_size + 1};
  CircularBuffer<vector<size_t> > opt_valid_idx_{estimator_config_.opt_window_size + 1};
  CircularBuffer<PointCloudPtr> opt_corner_stack_{estimator_config_.opt_window_size + 1};
  CircularBuffer<PointCloudPtr> opt_surf_stack_{estimator_config_.opt_window_size + 1};

  CircularBuffer<Eigen::Matrix<double, 6, 6>> opt_matP_{estimator_config_.opt_window_size + 1};
  ///< optimization buffers

//  Transform transform_lb_{Eigen::Quaternionf(1, 0, 0, 0), Eigen::Vector3f(-0.05, 0, 0.05)}; ///< Base to laser transform
  Transform transform_lb_{Eigen::Quaternionf(1, 0, 0, 0), Eigen::Vector3f(0, 0, -0.1)}; ///< Base to laser transform

  Eigen::Matrix3d R_WI_; ///< R_WI is the rotation from the inertial frame into Lidar's world frame
  Eigen::Quaterniond Q_WI_; ///< Q_WI is the rotation from the inertial frame into Lidar's world frame

  tf::StampedTransform wi_trans_, laser_local_trans_, laser_predict_trans_;
  tf::TransformBroadcaster tf_broadcaster_est_;

  ros::Publisher pub_predict_odom_;
  nav_msgs::Odometry predict_odom_;

  ros::Publisher pub_local_odom_;
  nav_msgs::Odometry local_odom_;

  ros::Publisher pub_laser_odom_;
  nav_msgs::Odometry laser_odom_;

  ros::Publisher pub_plane_normal_;
  visualization_msgs::MarkerArray plane_normal_array_;

  ros::Publisher pub_local_surf_points_;
  ros::Publisher pub_local_corner_points_;
  ros::Publisher pub_local_full_points_;

  ros::Publisher pub_map_surf_points_;
  ros::Publisher pub_map_corner_points_;
  ros::Publisher pub_predict_surf_points_;
  ros::Publisher pub_predict_corner_points_;
  ros::Publisher pub_predict_full_points_;
  ros::Publisher pub_predict_corrected_full_points_;

  ros::Publisher pub_extrinsic_;

  Visualizer vis_bef_opt{"vis_bef_opt", vector<double>{0.0, 0.0, 1.0}, vector<double>{1.0, 1.0, 1.0}};
  Visualizer vis_aft_opt{"vis_aft_opt"};

  Vector3d P_pivot_;
  Matrix3d R_pivot_;

  bool convergence_flag_ = false;

  CircularBuffer<StampedTransform> imu_stampedtransforms{100};

 private:
  double **para_pose_;
  double **para_speed_bias_;
  double para_ex_pose_[SIZE_POSE];
//  double para_qwi_[SIZE_QUAT];
  double g_norm_;
  bool gravity_fixed_ = false;

  Transform transform_tobe_mapped_bef_;
  Transform transform_es_;

  // for marginalization
  MarginalizationInfo *last_marginalization_info;
  vector<double *> last_marginalization_parameter_blocks;
  vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > marg_coeffi, marg_coeffj;
  vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > marg_pointi, marg_pointj;
  vector<double> marg_score;

};

}

#endif //LIO_ESTIMATOR_H_

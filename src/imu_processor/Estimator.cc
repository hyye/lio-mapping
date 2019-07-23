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

#include <ceres/ceres.h>
#include "imu_processor/Estimator.h"

#include <pcl/common/transforms.h>
#include <pcl/search/impl/flann_search.hpp>
#include <pcl/filters/extract_indices.h>

namespace lio {

#define CHECK_JACOBIAN 0

//void PointOdometry::TransformToStart(PointCloudPtr &cloud, Twist<float> transform_es, float time_factor) {
//  size_t cloud_size = cloud->points.size();
//
//  for (size_t i = 0; i < cloud_size; i++) {
//    PointT &point = cloud->points[i];
//    float s = time_factor * (point.intensity - int(point.intensity));
//    point.x = point.x - s * transform_es.pos.x();
//    point.y = point.y - s * transform_es.pos.y();
//    point.z = point.z - s * transform_es.pos.z();
//
//    Eigen::Quaternionf q_id, q_s, q_e;
//    q_e = transform_es.rot;
//    q_id.setIdentity();
//    q_s = q_id.slerp(s, q_e);
//
//    RotatePoint(q_s.conjugate(), point);
//  }
//}

// WARNING to half actually
size_t TransformToEnd(PointCloudPtr &cloud, Twist<float> transform_es, float time_factor, bool keep_intensity = false) {
  size_t cloud_size = cloud->points.size();

  for (size_t i = 0; i < cloud_size; i++) {
    PointT &point = cloud->points[i];

    float s = time_factor * (point.intensity - int(point.intensity));

//    DLOG(INFO) << "s: " << s;
    if (s < 0 || s > 1 + 1e-3) {
      LOG(ERROR) << "point.intensity: " << point.intensity;
      LOG(ERROR) << "time ratio error: " << s;
    }

    point.x -= s * transform_es.pos.x();
    point.y -= s * transform_es.pos.y();
    point.z -= s * transform_es.pos.z();
    if (!keep_intensity) {
      point.intensity -= int(point.intensity);
    }

    Eigen::Quaternionf q_id, q_s, q_e, q_half;
    q_e = transform_es.rot;
    q_id.setIdentity();
    q_s = q_id.slerp(s, q_e);

    RotatePoint(q_s.conjugate().normalized(), point);

//    q_half = q_id.slerp(0.5, q_e);
//    RotatePoint(q_half, point);
//    point.x += 0.5 * transform_es.pos.x();
//    point.y += 0.5 * transform_es.pos.y();
//    point.z += 0.5 * transform_es.pos.z();
     RotatePoint(q_e, point);

     point.x += transform_es.pos.x();
     point.y += transform_es.pos.y();
     point.z += transform_es.pos.z();
  }

  return cloud_size;
}

Estimator::Estimator() {
  ROS_DEBUG(">>>>>>> Estimator started! <<<<<<<");

  para_pose_ = new double *[estimator_config_.opt_window_size + 1];
  para_speed_bias_ = new double *[estimator_config_.opt_window_size + 1];
  for (int i = 0; i < estimator_config_.opt_window_size + 1;
       ++i) {
    para_pose_[i] = new double[SIZE_POSE];
    para_speed_bias_[i] = new double[SIZE_SPEED_BIAS];
  }

  ClearState();
}

Estimator::Estimator(EstimatorConfig config, MeasurementManagerConfig mm_config) {
  ROS_DEBUG(">>>>>>> Estimator started! <<<<<<<");

  SetupAllEstimatorConfig(config, mm_config);

  para_pose_ = new double *[estimator_config_.opt_window_size + 1];
  para_speed_bias_ = new double *[estimator_config_.opt_window_size + 1];
  for (int i = 0; i < estimator_config_.opt_window_size + 1;
       ++i) {
    para_pose_[i] = new double[SIZE_POSE];
    para_speed_bias_[i] = new double[SIZE_SPEED_BIAS];
  }

  ClearState();
}

Estimator::~Estimator() {
  for (int i = 0; i < estimator_config_.opt_window_size + 1;
       ++i) {
    delete[] para_pose_[i];
    delete[] para_speed_bias_[i];
  }
  delete[] para_pose_;
  delete[] para_speed_bias_;
}

void Estimator::SetupAllEstimatorConfig(const EstimatorConfig &config, const MeasurementManagerConfig &mm_config) {

  this->mm_config_ = mm_config;

  if (estimator_config_.window_size != config.window_size) {
    all_laser_transforms_.Reset(config.window_size + 1);
    Ps_.Reset(config.window_size + 1);
    Rs_.Reset(config.window_size + 1);
    Vs_.Reset(config.window_size + 1);
    Bas_.Reset(config.window_size + 1);
    Bgs_.Reset(config.window_size + 1);
    Headers_.Reset(config.window_size + 1);
    dt_buf_.Reset(config.window_size + 1);
    linear_acceleration_buf_.Reset(config.window_size + 1);
    angular_velocity_buf_.Reset(config.window_size + 1);
    pre_integrations_.Reset(config.window_size + 1);
    surf_stack_.Reset(config.window_size + 1);
    corner_stack_.Reset(config.window_size + 1);
    full_stack_.Reset(config.window_size + 1);
    size_surf_stack_.Reset(config.window_size + 1);
    size_corner_stack_.Reset(config.window_size + 1);

    //region fix the map
#ifdef FIX_MAP
    Ps_linearized_.Reset(config.window_size + 1);
    Rs_linearized_.Reset(config.window_size + 1);
#endif
    //endregion
  }

  if (estimator_config_.opt_window_size != config.opt_window_size) {
    ///> optimization buffers
    opt_point_coeff_mask_.Reset(config.opt_window_size + 1);
    opt_point_coeff_map_.Reset(config.opt_window_size + 1);
    opt_cube_centers_.Reset(config.opt_window_size + 1);
    opt_transforms_.Reset(config.opt_window_size + 1);
    opt_valid_idx_.Reset(config.opt_window_size + 1);
    opt_corner_stack_.Reset(config.opt_window_size + 1);
    opt_surf_stack_.Reset(config.opt_window_size + 1);

    opt_matP_.Reset(config.opt_window_size + 1);
    ///< optimization buffers
  }

  down_size_filter_corner_.setLeafSize(config.corner_filter_size, config.corner_filter_size, config.corner_filter_size);
  down_size_filter_surf_.setLeafSize(config.surf_filter_size, config.surf_filter_size, config.surf_filter_size);
  down_size_filter_map_.setLeafSize(config.map_filter_size, config.map_filter_size, config.map_filter_size);
  transform_lb_ = config.transform_lb;

  min_match_sq_dis_ = config.min_match_sq_dis;
  min_plane_dis_ = config.min_plane_dis;
  extrinsic_stage_ = config.estimate_extrinsic;

  if (!config.imu_factor) {
    this->mm_config_.enable_imu = false;
    if (!first_imu_) {
      first_imu_ = true;
      acc_last_;
      gyr_last_;
      dt_buf_.push(dt_buf_[0]);
      linear_acceleration_buf_.push(linear_acceleration_buf_[0]);
      angular_velocity_buf_.push(angular_velocity_buf_[0]);

      Ps_.push(Ps_[0]);
      Rs_.push(Rs_[0]);
      Vs_.push(Vs_[0]);
      Bgs_.push(Bgs_[0]);
      Bas_.push(Bas_[0]);
//      pre_integrations_.push(std::make_shared<IntegrationBase>(IntegrationBase(acc_last_,
//                                                                               gyr_last_,
//                                                                               Bas_[cir_buf_count_],
//                                                                               Bgs_[cir_buf_count_],
//                                                                               estimator_config_.pim_config)));

      //region fix the map
#ifdef FIX_MAP
      Ps_linearized_.push(Ps_linearized_[0]);
      Rs_linearized_.push(Rs_linearized_[0]);
#endif
      //endregion
    }
  }

  estimator_config_ = config;
}

void Estimator::ClearState() {
  // TODO: CirclarBuffer should have clear method

  for (size_t i = 0; i < estimator_config_.window_size + 1;
       ++i) {
    Rs_[i].setIdentity();
    Ps_[i].setZero();
    Vs_[i].setZero();
    Bas_[i].setZero();
    Bgs_[i].setZero();
    dt_buf_[i].clear();
    linear_acceleration_buf_[i].clear();
    angular_velocity_buf_[i].clear();

    surf_stack_[i].reset();
    corner_stack_[i].reset();
    full_stack_[i].reset();
    size_surf_stack_[i] = 0;
    size_corner_stack_[i] = 0;
    init_local_map_ = false;

    if (pre_integrations_[i] != nullptr) {
      pre_integrations_[i].reset();
    }
  }

  for (size_t i = 0; i < estimator_config_.opt_window_size + 1;
       ++i) {
//    opt_point_coeff_mask_[i] = false;
//    opt_cube_centers_[i];
//    opt_valid_idx_[i];
    opt_point_coeff_map_[i].clear();
    opt_corner_stack_[i].reset();
    opt_surf_stack_[i].reset();
  }

  stage_flag_ = NOT_INITED;
  first_imu_ = false;
  cir_buf_count_ = 0;

  tmp_pre_integration_.reset();
  tmp_pre_integration_ = std::make_shared<IntegrationBase>(IntegrationBase(acc_last_,
                                                                           gyr_last_,
                                                                           Bas_[cir_buf_count_],
                                                                           Bgs_[cir_buf_count_],
                                                                           estimator_config_.pim_config));

  // TODO: make shared?
  last_marginalization_info = nullptr;

  R_WI_.setIdentity();
  Q_WI_ = R_WI_;

  // WARNING: g_norm should be set before clear
  g_norm_ = tmp_pre_integration_->config_.g_norm;

  convergence_flag_ = false;
}

void Estimator::SetupRos(ros::NodeHandle &nh) {
  MeasurementManager::SetupRos(nh);
  PointMapping::SetupRos(nh, false);

  wi_trans_.frame_id_ = "/camera_init";
  wi_trans_.child_frame_id_ = "/world";

  wi_trans_.setRotation(tf::Quaternion(0, 0, 0, 1));
  wi_trans_.setOrigin(tf::Vector3(0, 0, 0));

  laser_local_trans_.frame_id_ = "/world";
  laser_local_trans_.child_frame_id_ = "/laser_local";

  laser_local_trans_.setRotation(tf::Quaternion(0, 0, 0, 1));
  laser_local_trans_.setOrigin(tf::Vector3(0, 0, 0));

  laser_predict_trans_.frame_id_ = "/laser_local";
  laser_predict_trans_.child_frame_id_ = "/laser_predict";
  laser_predict_trans_.setIdentity();

  predict_odom_.header.frame_id = "/world";
  predict_odom_.child_frame_id = "/imu_predict";
  pub_predict_odom_ = nh.advertise<nav_msgs::Odometry>("/predict_odom", 100);

  laser_odom_.header.frame_id = "/world";
  laser_odom_.child_frame_id = "/laser_predict";
  pub_laser_odom_ = nh.advertise<nav_msgs::Odometry>("/predict_laser_odom", 100);

  local_odom_.header.frame_id = "/world";
  local_odom_.child_frame_id = "/laser_predict";
  pub_local_odom_ = nh.advertise<nav_msgs::Odometry>("/local_laser_odom", 100);

  pub_plane_normal_ = nh.advertise<visualization_msgs::MarkerArray>("/debug/plane_normal", 5);

  pub_local_surf_points_ = nh.advertise<sensor_msgs::PointCloud2>("/local/surf_points", 2);
  pub_local_corner_points_ = nh.advertise<sensor_msgs::PointCloud2>("/local/corner_points", 2);
  pub_local_full_points_ = nh.advertise<sensor_msgs::PointCloud2>("/local/full_points", 2);

  pub_map_surf_points_ = nh.advertise<sensor_msgs::PointCloud2>("/map/surf_points", 2);
  pub_map_corner_points_ = nh.advertise<sensor_msgs::PointCloud2>("/map/corner_points", 2);
  pub_predict_surf_points_ = nh.advertise<sensor_msgs::PointCloud2>("/predict/surf_points", 2);
  pub_predict_corner_points_ = nh.advertise<sensor_msgs::PointCloud2>("/predict/corner_points", 2);
  pub_predict_full_points_ = nh.advertise<sensor_msgs::PointCloud2>("/predict/full_points", 2);
  pub_predict_corrected_full_points_ = nh.advertise<sensor_msgs::PointCloud2>("/predict/corrected_full_points", 2);

  pub_extrinsic_ = nh.advertise<geometry_msgs::PoseStamped>("/extrinsic_lb", 10);
}

void Estimator::ProcessImu(double dt,
                           const Vector3d &linear_acceleration,
                           const Vector3d &angular_velocity,
                           const std_msgs::Header &header) {
  if (!first_imu_) {
    first_imu_ = true;
    acc_last_ = linear_acceleration;
    gyr_last_ = angular_velocity;
    dt_buf_.push(vector<double>());
    linear_acceleration_buf_.push(vector<Vector3d>());
    angular_velocity_buf_.push(vector<Vector3d>());

    Eigen::Matrix3d I3x3;
    I3x3.setIdentity();
    Ps_.push(Vector3d{0, 0, 0});
    Rs_.push(I3x3);
    Vs_.push(Vector3d{0, 0, 0});
    Bgs_.push(Vector3d{0, 0, 0});
    Bas_.push(Vector3d{0, 0, 0});
//    pre_integrations_.push(std::make_shared<IntegrationBase>(IntegrationBase(acc_last_,
//                                                                             gyr_last_,
//                                                                             Bas_[cir_buf_count_],
//                                                                             Bgs_[cir_buf_count_],
//                                                                             estimator_config_.pim_config)));
    //region fix the map
#ifdef FIX_MAP
    Ps_linearized_.push(Vector3d{0, 0, 0});
    Rs_linearized_.push(I3x3);
#endif
    //endregion
  }

//  if (!pre_integrations_[cir_buf_count_]) {
//    pre_integrations_.push(std::make_shared<IntegrationBase>(IntegrationBase(acc_last_,
//                                                                             gyr_last_,
//                                                                             Bas_[cir_buf_count_],
//                                                                             Bgs_[cir_buf_count_],
//                                                                             estimator_config_.pim_config)));
//  }

  // NOTE: Do not update tmp_pre_integration_ until first laser comes
  if (cir_buf_count_ != 0) {

    tmp_pre_integration_->push_back(dt, linear_acceleration, angular_velocity);

    dt_buf_[cir_buf_count_].push_back(dt);
    linear_acceleration_buf_[cir_buf_count_].push_back(linear_acceleration);
    angular_velocity_buf_[cir_buf_count_].push_back(angular_velocity);

    size_t j = cir_buf_count_;
    Vector3d un_acc_0 = Rs_[j] * (acc_last_ - Bas_[j]) + g_vec_;
    Vector3d un_gyr = 0.5 * (gyr_last_ + angular_velocity) - Bgs_[j];
    Rs_[j] *= DeltaQ(un_gyr * dt).toRotationMatrix();
    Vector3d un_acc_1 = Rs_[j] * (linear_acceleration - Bas_[j]) + g_vec_;
    Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    Ps_[j] += dt * Vs_[j] + 0.5 * dt * dt * un_acc;
    Vs_[j] += dt * un_acc;

    StampedTransform imu_tt;
    imu_tt.time = header.stamp.toSec();
    imu_tt.transform.pos = Ps_[j].cast<float>();
    imu_tt.transform.rot = Eigen::Quaternionf(Rs_[j].cast<float>());
    imu_stampedtransforms.push(imu_tt);
//    DLOG(INFO) << imu_tt.transform;
  }
  acc_last_ = linear_acceleration;
  gyr_last_ = angular_velocity;

  if (stage_flag_ == INITED) {
    predict_odom_.header.stamp = header.stamp;
    predict_odom_.header.seq += 1;
    Eigen::Quaterniond quat(Rs_.last());
    predict_odom_.pose.pose.orientation.x = quat.x();
    predict_odom_.pose.pose.orientation.y = quat.y();
    predict_odom_.pose.pose.orientation.z = quat.z();
    predict_odom_.pose.pose.orientation.w = quat.w();
    predict_odom_.pose.pose.position.x = Ps_.last().x();
    predict_odom_.pose.pose.position.y = Ps_.last().y();
    predict_odom_.pose.pose.position.z = Ps_.last().z();
    predict_odom_.twist.twist.linear.x = Vs_.last().x();
    predict_odom_.twist.twist.linear.y = Vs_.last().y();
    predict_odom_.twist.twist.linear.z = Vs_.last().z();
    predict_odom_.twist.twist.angular.x = Bas_.last().x();
    predict_odom_.twist.twist.angular.y = Bas_.last().y();
    predict_odom_.twist.twist.angular.z = Bas_.last().z();

    pub_predict_odom_.publish(predict_odom_);
  }

}

// TODO: this function can be simplified
void Estimator::ProcessLaserOdom(const Transform &transform_in, const std_msgs::Header &header) {

  ROS_DEBUG(">>>>>>> new laser odom coming <<<<<<<");

  ++laser_odom_recv_count_;

  if (stage_flag_ != INITED
      && laser_odom_recv_count_ % estimator_config_.init_window_factor != 0) { /// better for initialization
    return;
  }

  Headers_.push(header);

  // TODO: LaserFrame Object
  // LaserFrame laser_frame(laser, header.stamp.toSec());

  LaserTransform laser_transform(header.stamp.toSec(), transform_in);

  laser_transform.pre_integration = tmp_pre_integration_;
  pre_integrations_.push(tmp_pre_integration_);

  // reset tmp_pre_integration_
  tmp_pre_integration_.reset();
  tmp_pre_integration_ = std::make_shared<IntegrationBase>(IntegrationBase(acc_last_,
                                                                           gyr_last_,
                                                                           Bas_[cir_buf_count_],
                                                                           Bgs_[cir_buf_count_],
                                                                           estimator_config_.pim_config));

  all_laser_transforms_.push(make_pair(header.stamp.toSec(), laser_transform));



  // TODO: check extrinsic parameter estimation

  // NOTE: push PointMapping's point_coeff_map_
  ///> optimization buffers
  opt_point_coeff_mask_.push(false); // default new frame
  opt_point_coeff_map_.push(score_point_coeff_);
  opt_cube_centers_.push(CubeCenter{laser_cloud_cen_length_, laser_cloud_cen_width_, laser_cloud_cen_height_});
  opt_transforms_.push(laser_transform.transform);
  opt_valid_idx_.push(laser_cloud_valid_idx_);

  // TODO: avoid memory allocation?
  if (stage_flag_ != INITED || (!estimator_config_.enable_deskew && !estimator_config_.cutoff_deskew)) {
    surf_stack_.push(boost::make_shared<PointCloud>(*laser_cloud_surf_stack_downsampled_));
    size_surf_stack_.push(laser_cloud_surf_stack_downsampled_->size());

    corner_stack_.push(boost::make_shared<PointCloud>(*laser_cloud_corner_stack_downsampled_));
    size_corner_stack_.push(laser_cloud_corner_stack_downsampled_->size());
  }

  full_stack_.push(boost::make_shared<PointCloud>(*full_cloud_));

  opt_surf_stack_.push(surf_stack_.last());
  opt_corner_stack_.push(corner_stack_.last());

  opt_matP_.push(matP_.cast<double>());
  ///< optimization buffers

  if (estimator_config_.run_optimization) {
    switch (stage_flag_) {
      case NOT_INITED: {

        {
          DLOG(INFO) << "surf_stack_: " << surf_stack_.size();
          DLOG(INFO) << "corner_stack_: " << corner_stack_.size();
          DLOG(INFO) << "pre_integrations_: " << pre_integrations_.size();
          DLOG(INFO) << "Ps_: " << Ps_.size();
          DLOG(INFO) << "size_surf_stack_: " << size_surf_stack_.size();
          DLOG(INFO) << "size_corner_stack_: " << size_corner_stack_.size();
          DLOG(INFO) << "all_laser_transforms_: " << all_laser_transforms_.size();
        }

        bool init_result = false;
        if (cir_buf_count_ == estimator_config_.window_size) {
          tic_toc_.Tic();

          if (!estimator_config_.imu_factor) {
            init_result = true;
            // TODO: update states Ps_
            for (size_t i = 0; i < estimator_config_.window_size + 1;
                 ++i) {
              const Transform &trans_li = all_laser_transforms_[i].second.transform;
              Transform trans_bi = trans_li * transform_lb_;
              Ps_[i] = trans_bi.pos.template cast<double>();
              Rs_[i] = trans_bi.rot.normalized().toRotationMatrix().template cast<double>();
            }
          } else {

            if (extrinsic_stage_ == 2) {
              // TODO: move before initialization
              bool extrinsic_result = ImuInitializer::EstimateExtrinsicRotation(all_laser_transforms_, transform_lb_);
              LOG(INFO) << ">>>>>>> extrinsic calibration"
                        << (extrinsic_result ? " successful"
                                             : " failed")
                        << " <<<<<<<";
              if (extrinsic_result) {
                extrinsic_stage_ = 1;
                DLOG(INFO) << "change extrinsic stage to 1";
              }
            }

            if (extrinsic_stage_ != 2 && (header.stamp.toSec() - initial_time_) > 0.1) {
              DLOG(INFO) << "EXTRINSIC STAGE: " << extrinsic_stage_;
              init_result = RunInitialization();
              initial_time_ = header.stamp.toSec();
            }
          }

          DLOG(INFO) << "initialization time: " << tic_toc_.Toc() << " ms";

          if (init_result) {
            stage_flag_ = INITED;
            SetInitFlag(true);

            Q_WI_ = R_WI_;
//            wi_trans_.setRotation(tf::Quaternion{Q_WI_.x(), Q_WI_.y(), Q_WI_.z(), Q_WI_.w()});

            ROS_WARN_STREAM(">>>>>>> IMU initialized <<<<<<<");

            if (estimator_config_.enable_deskew || estimator_config_.cutoff_deskew) {
              ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>("/enable_odom");
              std_srvs::SetBool srv;
              srv.request.data = 0;
              if (client.call(srv)) {
                DLOG(INFO) << "TURN OFF THE ORIGINAL LASER ODOM";
              } else {
                LOG(FATAL) << "FAILED TO CALL TURNING OFF THE ORIGINAL LASER ODOM";
              }
            }

            for (size_t i = 0; i < estimator_config_.window_size + 1;
                 ++i) {
              Twist<double> transform_lb = transform_lb_.cast<double>();

              Quaterniond Rs_li(Rs_[i] * transform_lb.rot.inverse());
              Eigen::Vector3d Ps_li = Ps_[i] - Rs_li * transform_lb.pos;

              Twist<double> trans_li{Rs_li, Ps_li};

              DLOG(INFO) << "TEST trans_li " << i << ": " << trans_li;
              DLOG(INFO) << "TEST all_laser_transforms " << i << ": " << all_laser_transforms_[i].second.transform;
            }

            SolveOptimization();

            SlideWindow();

            for (size_t i = 0; i < estimator_config_.window_size + 1;
                 ++i) {
              const Transform &trans_li = all_laser_transforms_[i].second.transform;
              Transform trans_bi = trans_li * transform_lb_;
              DLOG(INFO) << "TEST " << i << ": " << trans_bi.pos.transpose();
            }

            for (size_t i = 0; i < estimator_config_.window_size + 1;
                 ++i) {
              Twist<double> transform_lb = transform_lb_.cast<double>();

              Quaterniond Rs_li(Rs_[i] * transform_lb.rot.inverse());
              Eigen::Vector3d Ps_li = Ps_[i] - Rs_li * transform_lb.pos;

              Twist<double> trans_li{Rs_li, Ps_li};

              DLOG(INFO) << "TEST trans_li " << i << ": " << trans_li;
            }

            // WARNING

          } else {
            SlideWindow();
          }
        } else {
          DLOG(INFO) << "Ps size: " << Ps_.size();
          DLOG(INFO) << "pre size: " << pre_integrations_.size();
          DLOG(INFO) << "all_laser_transforms_ size: " << all_laser_transforms_.size();

          SlideWindow();

          DLOG(INFO) << "Ps size: " << Ps_.size();
          DLOG(INFO) << "pre size: " << pre_integrations_.size();

          ++cir_buf_count_;
        }

        opt_point_coeff_mask_.last() = true;

        break;
      }
      case INITED: {

        // TODO

        // WARNING

        if (opt_point_coeff_map_.size() == estimator_config_.opt_window_size + 1) {

          if (estimator_config_.enable_deskew || estimator_config_.cutoff_deskew) {
            TicToc t_deskew;
            t_deskew.Tic();
            // TODO: the coefficients to be parameterized
            DLOG(INFO) << ">>>>>>> de-skew points <<<<<<<";
            LOG_ASSERT(imu_stampedtransforms.size() > 0) << "no imu data";
            double time_e = imu_stampedtransforms.last().time;
            Transform transform_e = imu_stampedtransforms.last().transform;
            double time_s = imu_stampedtransforms.last().time;
            Transform transform_s = imu_stampedtransforms.last().transform;
            for (int i = int(imu_stampedtransforms.size()) - 1; i >= 0; --i) {
              time_s = imu_stampedtransforms[i].time;
              transform_s = imu_stampedtransforms[i].transform;
              if (time_e - imu_stampedtransforms[i].time >= 0.1) {
                break;
              }
            }
//            Eigen::Vector3d body_velocity, vel1, vel2;
//            vel1 = Rs_[estimator_config_.window_size - 1].transpose() * Vs_[estimator_config_.window_size - 1];
//            vel2 = Rs_[estimator_config_.window_size].transpose() * Vs_[estimator_config_.window_size];
//            body_velocity = (vel1 + vel2) / 2;

            Transform transform_body_es = transform_e.inverse() * transform_s;
//            transform_body_es.pos = -0.1 * body_velocity.cast<float>();
            {
              float s = 0.1 / (time_e - time_s);
              Eigen::Quaternionf q_id, q_s, q_e, q_half;
              q_e = transform_body_es.rot;
              q_id.setIdentity();
              q_s = q_id.slerp(s, q_e);
              transform_body_es.rot = q_s;
              transform_body_es.pos = s * transform_body_es.pos;
            }

            transform_es_ = transform_lb_ * transform_body_es * transform_lb_.inverse();
            DLOG(INFO) << "time diff: " << time_e - time_s;
            DLOG(INFO) << "transform diff: " << transform_es_;
            DLOG(INFO) << "transform diff norm: " << transform_es_.pos.norm();

            if (!estimator_config_.cutoff_deskew) {
              TransformToEnd(laser_cloud_surf_last_, transform_es_, 10);

              TransformToEnd(laser_cloud_corner_last_, transform_es_, 10);
#ifdef USE_CORNER
              TransformToEnd(corner_stack_.last(), transform_es_, 10);
#endif
            } else {
              DLOG(INFO) << "cutoff_deskew";
            }

            laser_cloud_surf_stack_downsampled_->clear();
            down_size_filter_surf_.setInputCloud(laser_cloud_surf_last_);
            down_size_filter_surf_.filter(*laser_cloud_surf_stack_downsampled_);
            size_t laser_cloud_surf_stack_ds_size = laser_cloud_surf_stack_downsampled_->points.size();

            // down sample feature stack clouds
            laser_cloud_corner_stack_downsampled_->clear();
            down_size_filter_corner_.setInputCloud(laser_cloud_corner_last_);
            down_size_filter_corner_.filter(*laser_cloud_corner_stack_downsampled_);
            size_t laser_cloud_corner_stack_ds_size = laser_cloud_corner_stack_downsampled_->points.size();

            surf_stack_.push(boost::make_shared<PointCloud>(*laser_cloud_surf_stack_downsampled_));
            size_surf_stack_.push(laser_cloud_surf_stack_downsampled_->size());

            corner_stack_.push(boost::make_shared<PointCloud>(*laser_cloud_corner_stack_downsampled_));
            size_corner_stack_.push(laser_cloud_corner_stack_downsampled_->size());

            ROS_DEBUG_STREAM("deskew time: " << t_deskew.Toc());

            DLOG(INFO) << "deskew time: " << t_deskew.Toc();
          }

          DLOG(INFO) << ">>>>>>> solving optimization <<<<<<<";
          SolveOptimization();

          if (!opt_point_coeff_mask_.first()) {
            UpdateMapDatabase(opt_corner_stack_.first(),
                              opt_surf_stack_.first(),
                              opt_valid_idx_.first(),
                              opt_transforms_.first(),
                              opt_cube_centers_.first());

            DLOG(INFO) << "all_laser_transforms_: " << all_laser_transforms_[estimator_config_.window_size
                - estimator_config_.opt_window_size].second.transform;
            DLOG(INFO) << "opt_transforms_: " << opt_transforms_.first();

          }

        } else {
          LOG(ERROR) << "opt_point_coeff_map_.size(): " << opt_point_coeff_map_.size()
                     << " != estimator_config_.opt_window_size + 1: " << estimator_config_.opt_window_size + 1;
        }

        PublishResults();

        SlideWindow();

        {
          int pivot_idx = estimator_config_.window_size - estimator_config_.opt_window_size;
          local_odom_.header.stamp = Headers_[pivot_idx + 1].stamp;
          local_odom_.header.seq += 1;
          Twist<double> transform_lb = transform_lb_.cast<double>();
          Eigen::Vector3d Ps_pivot = Ps_[pivot_idx];
          Eigen::Matrix3d Rs_pivot = Rs_[pivot_idx];
          Quaterniond rot_pivot(Rs_pivot * transform_lb.rot.inverse());
          Eigen::Vector3d pos_pivot = Ps_pivot - rot_pivot * transform_lb.pos;
          Twist<double> transform_pivot = Twist<double>(rot_pivot, pos_pivot);
          local_odom_.pose.pose.orientation.x = transform_pivot.rot.x();
          local_odom_.pose.pose.orientation.y = transform_pivot.rot.y();
          local_odom_.pose.pose.orientation.z = transform_pivot.rot.z();
          local_odom_.pose.pose.orientation.w = transform_pivot.rot.w();
          local_odom_.pose.pose.position.x = transform_pivot.pos.x();
          local_odom_.pose.pose.position.y = transform_pivot.pos.y();
          local_odom_.pose.pose.position.z = transform_pivot.pos.z();
          pub_local_odom_.publish(local_odom_);

          laser_odom_.header.stamp = header.stamp;
          laser_odom_.header.seq += 1;
          Eigen::Vector3d Ps_last = Ps_.last();
          Eigen::Matrix3d Rs_last = Rs_.last();
          Quaterniond rot_last(Rs_last * transform_lb.rot.inverse());
          Eigen::Vector3d pos_last = Ps_last - rot_last * transform_lb.pos;
          Twist<double> transform_last = Twist<double>(rot_last, pos_last);
          laser_odom_.pose.pose.orientation.x = transform_last.rot.x();
          laser_odom_.pose.pose.orientation.y = transform_last.rot.y();
          laser_odom_.pose.pose.orientation.z = transform_last.rot.z();
          laser_odom_.pose.pose.orientation.w = transform_last.rot.w();
          laser_odom_.pose.pose.position.x = transform_last.pos.x();
          laser_odom_.pose.pose.position.y = transform_last.pos.y();
          laser_odom_.pose.pose.position.z = transform_last.pos.z();
          pub_laser_odom_.publish(laser_odom_);
        }

        break;
      }
      default: {
        break;
      }

    }
  }

  wi_trans_.setRotation(tf::Quaternion{Q_WI_.x(), Q_WI_.y(), Q_WI_.z(), Q_WI_.w()});
  wi_trans_.stamp_ = header.stamp;
  tf_broadcaster_est_.sendTransform(wi_trans_);

}

void Estimator::ProcessCompactData(const sensor_msgs::PointCloud2ConstPtr &compact_data,
                                   const std_msgs::Header &header) {
  /// 1. process compact data
  PointMapping::CompactDataHandler(compact_data);

  if (stage_flag_ == INITED) {
    Transform trans_prev(Eigen::Quaterniond(Rs_[estimator_config_.window_size - 1]).cast<float>(),
                         Ps_[estimator_config_.window_size - 1].cast<float>());
    Transform trans_curr(Eigen::Quaterniond(Rs_.last()).cast<float>(),
                         Ps_.last().cast<float>());

    Transform d_trans = trans_prev.inverse() * trans_curr;

    Transform transform_incre(transform_bef_mapped_.inverse() * transform_sum_.transform());

//    DLOG(INFO) << "base incre in laser world: " << d_trans;
//    DLOG(INFO) << "incre in laser world: " << transform_incre;
//    DLOG(INFO) << "before opt: " << transform_tobe_mapped_ * transform_lb_ * d_trans * transform_lb_.inverse();
//
//    DLOG(INFO) << "curr * lb: " << trans_curr * transform_lb_.inverse();
//
//    DLOG(INFO) << "tobe: " << transform_tobe_mapped_ * transform_incre;

    if (estimator_config_.imu_factor) {
      //    // WARNING: or using direct date?
      transform_tobe_mapped_bef_ = transform_tobe_mapped_ * transform_lb_ * d_trans * transform_lb_.inverse();
      transform_tobe_mapped_ = transform_tobe_mapped_bef_;
    } else {
      TransformAssociateToMap();
      DLOG(INFO) << ">>>>> transform original tobe <<<<<: " << transform_tobe_mapped_;
    }

  }

  if (stage_flag_ != INITED || !estimator_config_.imu_factor) {
    /// 2. process decoded data
    PointMapping::Process();
  } else {
//    for (int i = 0; i < laser_cloud_surf_last_->size(); ++i) {
//      PointT &p = laser_cloud_surf_last_->at(i);
//      p.intensity = p.intensity - int(p.intensity);
//    }
//
//    for (int i = 0; i < laser_cloud_corner_last_->size(); ++i) {
//      PointT &p = laser_cloud_corner_last_->at(i);
//      p.intensity = p.intensity - int(p.intensity);
//    }
//
//    laser_cloud_surf_stack_downsampled_->clear();
//    down_size_filter_surf_.setInputCloud(laser_cloud_surf_last_);
//    down_size_filter_surf_.filter(*laser_cloud_surf_stack_downsampled_);
//    size_t laser_cloud_surf_stack_ds_size = laser_cloud_surf_stack_downsampled_->points.size();
//
//    // down sample feature stack clouds
//    laser_cloud_corner_stack_downsampled_->clear();
//    down_size_filter_corner_.setInputCloud(laser_cloud_corner_last_);
//    down_size_filter_corner_.filter(*laser_cloud_corner_stack_downsampled_);
//    size_t laser_cloud_corner_stack_ds_size = laser_cloud_corner_stack_downsampled_->points.size();
  }

  DLOG(INFO) << "laser_cloud_surf_last_[" << header.stamp.toSec() << "]: "
            << laser_cloud_surf_last_->size();
  DLOG(INFO) << "laser_cloud_corner_last_[" << header.stamp.toSec() << "]: "
            << laser_cloud_corner_last_->size();

  DLOG(INFO) << endl << "transform_aft_mapped_[" << header.stamp.toSec() << "]: " << transform_aft_mapped_;
  DLOG(INFO) << "laser_cloud_surf_stack_downsampled_[" << header.stamp.toSec() << "]: "
            << laser_cloud_surf_stack_downsampled_->size();
  DLOG(INFO) << "laser_cloud_corner_stack_downsampled_[" << header.stamp.toSec() << "]: "
            << laser_cloud_corner_stack_downsampled_->size();

  Transform transform_to_init_ = transform_aft_mapped_;
  ProcessLaserOdom(transform_to_init_, header);

// NOTE: will be updated in PointMapping's OptimizeTransformTobeMapped
//  if (stage_flag_ == INITED && !estimator_config_.imu_factor) {
//    TransformUpdate();
//    DLOG(INFO) << ">>>>> transform sum <<<<<: " << transform_sum_;
//  }

}

bool Estimator::RunInitialization() {

  // NOTE: check IMU observibility, adapted from VINS-mono
  {
    PairTimeLaserTransform laser_trans_i, laser_trans_j;
    Vector3d sum_g;

    for (size_t i = 0; i < estimator_config_.window_size;
         ++i) {
      laser_trans_j = all_laser_transforms_[i + 1];

      double dt = laser_trans_j.second.pre_integration->sum_dt_;
      Vector3d tmp_g = laser_trans_j.second.pre_integration->delta_v_ / dt;
      sum_g += tmp_g;
    }

    Vector3d aver_g;
    aver_g = sum_g * 1.0 / (estimator_config_.window_size);
    double var = 0;

    for (size_t i = 0; i < estimator_config_.window_size;
         ++i) {
      laser_trans_j = all_laser_transforms_[i + 1];
      double dt = laser_trans_j.second.pre_integration->sum_dt_;
      Vector3d tmp_g = laser_trans_j.second.pre_integration->delta_v_ / dt;
      var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
    }

    var = sqrt(var / (estimator_config_.window_size));

    DLOG(INFO) << "IMU variation: " << var;

    if (var < 0.25) {
      ROS_INFO("IMU excitation not enough!");
      return false;
    }
  }

  Eigen::Vector3d g_vec_in_laser;
  bool init_result
      = ImuInitializer::Initialization(all_laser_transforms_, Vs_, Bas_, Bgs_, g_vec_in_laser, transform_lb_, R_WI_);
//  init_result = false;

//  Q_WI_ = R_WI_;
//  g_vec_ = R_WI_ * Eigen::Vector3d(0.0, 0.0, -1.0) * g_norm_;
//  g_vec_ = Eigen::Vector3d(0.0, 0.0, -1.0) * g_norm_;

  // TODO: update states Ps_
  for (size_t i = 0; i < estimator_config_.window_size + 1;
       ++i) {
    const Transform &trans_li = all_laser_transforms_[i].second.transform;
    Transform trans_bi = trans_li * transform_lb_;
    Ps_[i] = trans_bi.pos.template cast<double>();
    Rs_[i] = trans_bi.rot.normalized().toRotationMatrix().template cast<double>();

    //region fix the map
#ifdef FIX_MAP
    Ps_linearized_[i] = Ps_[i];
    Rs_linearized_[i] = Rs_[i];
#endif
    //endregion
  }

  Matrix3d R0 = R_WI_.transpose();

  double yaw = R2ypr(R0 * Rs_[0]).x();
  R0 = (ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0).eval();

  R_WI_ = R0.transpose();
  Q_WI_ = R_WI_;

  g_vec_ = R0 * g_vec_in_laser;

  for (int i = 0; i <= cir_buf_count_; i++) {
    pre_integrations_[i]->Repropagate(Bas_[i], Bgs_[i]);
  }

  Matrix3d rot_diff = R0;
  for (int i = 0; i <= cir_buf_count_; i++) {
    Ps_[i] = (rot_diff * Ps_[i]).eval();
    Rs_[i] = (rot_diff * Rs_[i]).eval();
    Vs_[i] = (rot_diff * Vs_[i]).eval();

    //region fix the map
#ifdef FIX_MAP
    Ps_linearized_[i] = Ps_[i];
    Rs_linearized_[i] = Rs_[i];
#endif
    //endregion
  }

  DLOG(WARNING) << "refined gravity:  " << g_vec_.transpose();

  if (!init_result) {
    DLOG(WARNING) << "Imu initialization failed!";
    return false;
  } else {
    DLOG(WARNING) << "Imu initialization successful!";
    return true;
  }
}

#ifdef USE_CORNER
void Estimator::CalculateFeatures(const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_surf_from_map,
                                  const PointCloudPtr &local_surf_points_filtered_ptr,
                                  const PointCloudPtr &surf_stack,
                                  const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_corner_from_map,
                                  const PointCloudPtr &local_corner_points_filtered_ptr,
                                  const PointCloudPtr &corner_stack,
                                  const Transform &local_transform,
                                  vector<unique_ptr<Feature>> &features) {
#else
void Estimator::CalculateFeatures(const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_surf_from_map,
                                  const PointCloudPtr &local_surf_points_filtered_ptr,
                                  const PointCloudPtr &surf_stack,
                                  const Transform &local_transform,
                                  vector<unique_ptr<Feature>> &features) {
#endif

  PointT point_sel, point_ori, point_proj, coeff1, coeff2;
  if (!estimator_config_.keep_features) {
    features.clear();
  }

  std::vector<int> point_search_idx(5, 0);
  std::vector<float> point_search_sq_dis(5, 0);
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

  PointCloud laser_cloud_ori;
  PointCloud coeff_sel;
  vector<float> scores;

  const PointCloudPtr &origin_surf_points = surf_stack;
  const Transform &transform_to_local = local_transform;
  size_t surf_points_size = origin_surf_points->points.size();

#ifdef USE_CORNER
  const PointCloudPtr &origin_corner_points = corner_stack;
  size_t corner_points_size = origin_corner_points->points.size();
#endif

//    DLOG(INFO) << "transform_to_local: " << transform_to_local;

  for (int i = 0; i < surf_points_size; i++) {
    point_ori = origin_surf_points->points[i];
    PointAssociateToMap(point_ori, point_sel, transform_to_local);

    int num_neighbors = 5;
    kdtree_surf_from_map->nearestKSearch(point_sel, num_neighbors, point_search_idx, point_search_sq_dis);

    if (point_search_sq_dis[num_neighbors - 1] < min_match_sq_dis_) {
      for (int j = 0; j < num_neighbors; j++) {
        mat_A0(j, 0) = local_surf_points_filtered_ptr->points[point_search_idx[j]].x;
        mat_A0(j, 1) = local_surf_points_filtered_ptr->points[point_search_idx[j]].y;
        mat_A0(j, 2) = local_surf_points_filtered_ptr->points[point_search_idx[j]].z;
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
        if (fabs(pa * local_surf_points_filtered_ptr->points[point_search_idx[j]].x +
            pb * local_surf_points_filtered_ptr->points[point_search_idx[j]].y +
            pc * local_surf_points_filtered_ptr->points[point_search_idx[j]].z + pd) > min_plane_dis_) {
          planeValid = false;
          break;
        }
      }

      if (planeValid) {

        float pd2 = pa * point_sel.x + pb * point_sel.y + pc * point_sel.z + pd;

        float s = 1 - 0.9f * fabs(pd2) / sqrt(CalcPointDistance(point_sel));

        coeff1.x = s * pa;
        coeff1.y = s * pb;
        coeff1.z = s * pc;
        coeff1.intensity = s * pd;

        bool is_in_laser_fov = false;
        PointT transform_pos;
        PointT point_on_z_axis;

        point_on_z_axis.x = 0.0;
        point_on_z_axis.y = 0.0;
        point_on_z_axis.z = 10.0;
        PointAssociateToMap(point_on_z_axis, point_on_z_axis, transform_to_local);

        transform_pos.x = transform_to_local.pos.x();
        transform_pos.y = transform_to_local.pos.y();
        transform_pos.z = transform_to_local.pos.z();
        float squared_side1 = CalcSquaredDiff(transform_pos, point_sel);
        float squared_side2 = CalcSquaredDiff(point_on_z_axis, point_sel);

        float check1 = 100.0f + squared_side1 - squared_side2
            - 10.0f * sqrt(3.0f) * sqrt(squared_side1);

        float check2 = 100.0f + squared_side1 - squared_side2
            + 10.0f * sqrt(3.0f) * sqrt(squared_side1);

        if (check1 < 0 && check2 > 0) { /// within +-60 degree
          is_in_laser_fov = true;
        }

        if (s > 0.1 && is_in_laser_fov) {
          unique_ptr<PointPlaneFeature> feature = std::make_unique<PointPlaneFeature>();
          feature->score = s;
          feature->point = Eigen::Vector3d{point_ori.x, point_ori.y, point_ori.z};
          feature->coeffs = Eigen::Vector4d{coeff1.x, coeff1.y, coeff1.z, coeff1.intensity};
          features.push_back(std::move(feature));
        }
      }
    }
  }

#ifdef USE_CORNER
  //region Corner points
  for (int i = 0; i < corner_points_size; i++) {
    point_ori = origin_corner_points->points[i];
    PointAssociateToMap(point_ori, point_sel, transform_to_local);
    kdtree_corner_from_map->nearestKSearch(point_sel, 5, point_search_idx, point_search_sq_dis);

    if (point_search_sq_dis[4] < min_match_sq_dis_) {
      Eigen::Vector3f vc(0, 0, 0);

      for (int j = 0; j < 5; j++) {
        const PointT &point_sel_tmp = local_corner_points_filtered_ptr->points[point_search_idx[j]];
        vc.x() += point_sel_tmp.x;
        vc.y() += point_sel_tmp.y;
        vc.z() += point_sel_tmp.z;
      }
      vc /= 5.0;

      Eigen::Matrix3f mat_a;
      mat_a.setZero();

      for (int j = 0; j < 5; j++) {
        const PointT &point_sel_tmp = local_corner_points_filtered_ptr->points[point_search_idx[j]];
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

        Eigen::Vector3f a012_vec = (X0 - X1).cross(X0 - X2);

        Eigen::Vector3f normal_to_point = ((X1 - X2).cross(a012_vec)).normalized();

        Eigen::Vector3f normal_cross_point = (X1 - X2).cross(normal_to_point);

        float a012 = a012_vec.norm();

        float l12 = (X1 - X2).norm();

        float la = normal_to_point.x();
        float lb = normal_to_point.y();
        float lc = normal_to_point.z();

        float ld2 = a012 / l12;

        point_proj = point_sel;
        point_proj.x -= la * ld2;
        point_proj.y -= lb * ld2;
        point_proj.z -= lc * ld2;

        float ld_p1 = -(normal_to_point.x() * point_proj.x + normal_to_point.y() * point_proj.y
            + normal_to_point.z() * point_proj.z);
        float ld_p2 = -(normal_cross_point.x() * point_proj.x + normal_cross_point.y() * point_proj.y
            + normal_cross_point.z() * point_proj.z);

        float s = 1 - 0.9f * fabs(ld2);

        coeff1.x = s * la;
        coeff1.y = s * lb;
        coeff1.z = s * lc;
        coeff1.intensity = s * ld_p1;

        coeff2.x = s * normal_cross_point.x();
        coeff2.y = s * normal_cross_point.y();
        coeff2.z = s * normal_cross_point.z();
        coeff2.intensity = s * ld_p2;

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
          unique_ptr<PointPlaneFeature> feature1 = std::make_unique<PointPlaneFeature>();
          feature1->score = s * 0.5;
          feature1->point = Eigen::Vector3d{point_ori.x, point_ori.y, point_ori.z};
          feature1->coeffs = Eigen::Vector4d{coeff1.x, coeff1.y, coeff1.z, coeff1.intensity} * 0.5;
          features.push_back(std::move(feature1));

          unique_ptr<PointPlaneFeature> feature2 = std::make_unique<PointPlaneFeature>();
          feature2->score = s * 0.5;
          feature2->point = Eigen::Vector3d{point_ori.x, point_ori.y, point_ori.z};
          feature2->coeffs = Eigen::Vector4d{coeff2.x, coeff2.y, coeff2.z, coeff2.intensity} * 0.5;
          features.push_back(std::move(feature2));
        }
      }
    }
  }
  //endregion
#endif
}

#ifdef USE_CORNER
void Estimator::CalculateLaserOdom(const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_surf_from_map,
                                   const PointCloudPtr &local_surf_points_filtered_ptr,
                                   const PointCloudPtr &surf_stack,
                                   const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_corner_from_map,
                                   const PointCloudPtr &local_corner_points_filtered_ptr,
                                   const PointCloudPtr &corner_stack,
                                   Transform &local_transform,
                                   vector<unique_ptr<Feature>> &features) {
#else
void Estimator::CalculateLaserOdom(const pcl::KdTreeFLANN<PointT>::Ptr &kdtree_surf_from_map,
                                   const PointCloudPtr &local_surf_points_filtered_ptr,
                                   const PointCloudPtr &surf_stack,
                                   Transform &local_transform,
                                   vector<unique_ptr<Feature>> &features) {
#endif

  bool is_degenerate = false;
  for (size_t iter_count = 0; iter_count < num_max_iterations_; ++iter_count) {

#ifdef USE_CORNER
    CalculateFeatures(kdtree_surf_from_map, local_surf_points_filtered_ptr, surf_stack,
                      kdtree_corner_from_map, local_corner_points_filtered_ptr, corner_stack,
                      local_transform, features);
#else
    CalculateFeatures(kdtree_surf_from_map, local_surf_points_filtered_ptr, surf_stack,
                      local_transform, features);
#endif

    size_t laser_cloud_sel_size = features.size();
    Eigen::Matrix<float, Eigen::Dynamic, 6> mat_A(laser_cloud_sel_size, 6);
    Eigen::Matrix<float, 6, Eigen::Dynamic> mat_At(6, laser_cloud_sel_size);
    Eigen::Matrix<float, 6, 6> matAtA;
    Eigen::VectorXf mat_B(laser_cloud_sel_size);
    Eigen::VectorXf mat_AtB;
    Eigen::VectorXf mat_X;
    Eigen::Matrix<float, 6, 6> matP;

    PointT point_sel, point_ori, coeff;

    SO3 R_SO3(local_transform.rot); /// SO3

    for (int i = 0; i < laser_cloud_sel_size; i++) {
      PointPlaneFeature feature_i;
      features[i]->GetFeature(&feature_i);
      point_ori.x = feature_i.point.x();
      point_ori.y = feature_i.point.y();
      point_ori.z = feature_i.point.z();
      coeff.x = feature_i.coeffs.x();
      coeff.y = feature_i.coeffs.y();
      coeff.z = feature_i.coeffs.z();
      coeff.intensity = feature_i.coeffs.w();

      Eigen::Vector3f p(point_ori.x, point_ori.y, point_ori.z);
      Eigen::Vector3f w(coeff.x, coeff.y, coeff.z);

//      Eigen::Vector3f J_r = w.transpose() * RotationVectorJacobian(R_SO3, p);
      Eigen::Vector3f J_r = -w.transpose() * (local_transform.rot * SkewSymmetric(p));
      Eigen::Vector3f J_t = w.transpose();

      float d2 = w.transpose() * (local_transform.rot * p + local_transform.pos) + coeff.intensity;

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

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 6, 6>> esolver(matAtA);
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
      matP = mat_V2 * mat_V.inverse();
    }

    if (is_degenerate) {
      Eigen::Matrix<float, 6, 1> matX2(mat_X);
      mat_X = matP * matX2;
    }

    local_transform.pos.x() += mat_X(3, 0);
    local_transform.pos.y() += mat_X(4, 0);
    local_transform.pos.z() += mat_X(5, 0);

    local_transform.rot = local_transform.rot * DeltaQ(Eigen::Vector3f(mat_X(0, 0), mat_X(1, 0), mat_X(2, 0)));

    if (!isfinite(local_transform.pos.x())) local_transform.pos.x() = 0.0;
    if (!isfinite(local_transform.pos.y())) local_transform.pos.y() = 0.0;
    if (!isfinite(local_transform.pos.z())) local_transform.pos.z() = 0.0;

    float delta_r = RadToDeg(R_SO3.unit_quaternion().angularDistance(local_transform.rot));
    float delta_t = sqrt(pow(mat_X(3, 0) * 100, 2) + pow(mat_X(4, 0) * 100, 2) + pow(mat_X(5, 0) * 100, 2));

    if (delta_r < delta_r_abort_ && delta_t < delta_t_abort_) {
      DLOG(INFO) << "CalculateLaserOdom iter_count: " << iter_count;
      break;
    }
  }
}

void Estimator::BuildLocalMap(vector<FeaturePerFrame> &feature_frames) {
  feature_frames.clear();

  TicToc t_build_map;

  local_surf_points_ptr_.reset();
  local_surf_points_ptr_ = boost::make_shared<PointCloud>(PointCloud());

  local_surf_points_filtered_ptr_.reset();
  local_surf_points_filtered_ptr_ = boost::make_shared<PointCloud>(PointCloud());

#ifdef USE_CORNER
  local_corner_points_ptr_.reset();
  local_corner_points_ptr_ = boost::make_shared<PointCloud>(PointCloud());

  local_corner_points_filtered_ptr_.reset();
  local_corner_points_filtered_ptr_ = boost::make_shared<PointCloud>(PointCloud());
#endif

//  PointCloudPtr local_surf_points_filtered_ptr_(new PointCloud());
  PointCloud local_normal;

  vector<Transform> local_transforms;
  int pivot_idx = estimator_config_.window_size - estimator_config_.opt_window_size;

  Twist<double> transform_lb = transform_lb_.cast<double>();

  Eigen::Vector3d Ps_pivot = Ps_[pivot_idx];
  Eigen::Matrix3d Rs_pivot = Rs_[pivot_idx];

  Quaterniond rot_pivot(Rs_pivot * transform_lb.rot.inverse());
  Eigen::Vector3d pos_pivot = Ps_pivot - rot_pivot * transform_lb.pos;

  Twist<double> transform_pivot = Twist<double>(rot_pivot, pos_pivot);

  {
    //region fix the map
#ifdef FIX_MAP
    Eigen::Vector3d Ps_linearized_pivot = Ps_linearized_[pivot_idx];
    Eigen::Matrix3d Rs_linearized_pivot = Rs_linearized_[pivot_idx];

    Quaterniond rot_linearized_pivot(Rs_linearized_pivot * transform_lb.rot.inverse());
    Eigen::Vector3d pos_linearized_pivot = Ps_linearized_pivot - rot_linearized_pivot * transform_lb.pos;

    Twist<double> transform_linearized_pivot = Twist<double>(rot_linearized_pivot, pos_linearized_pivot);
#endif
    //endregion

    if (!init_local_map_) {
      PointCloud transformed_cloud_surf, tmp_cloud_surf;

#ifdef USE_CORNER
      PointCloud transformed_cloud_corner, tmp_cloud_corner;
#endif

      for (int i = 0; i <= pivot_idx; ++i) {
        Eigen::Vector3d Ps_i = Ps_[i];
        Eigen::Matrix3d Rs_i = Rs_[i];

        Quaterniond rot_li(Rs_i * transform_lb.rot.inverse());
        Eigen::Vector3d pos_li = Ps_i - rot_li * transform_lb.pos;

        Twist<double> transform_li = Twist<double>(rot_li, pos_li);
        Eigen::Affine3f transform_pivot_i = (transform_pivot.inverse() * transform_li).cast<float>().transform();
        pcl::transformPointCloud(*(surf_stack_[i]), transformed_cloud_surf, transform_pivot_i);
        tmp_cloud_surf += transformed_cloud_surf;

#ifdef USE_CORNER
        pcl::transformPointCloud(*(corner_stack_[i]), transformed_cloud_corner, transform_pivot_i);
        tmp_cloud_corner += transformed_cloud_corner;
#endif
      }

      *(surf_stack_[pivot_idx]) = tmp_cloud_surf;

#ifdef USE_CORNER
      *(corner_stack_[pivot_idx]) = tmp_cloud_corner;
#endif

      init_local_map_ = true;
    }

    for (int i = 0; i < estimator_config_.window_size + 1; ++i) {

      Eigen::Vector3d Ps_i = Ps_[i];
      Eigen::Matrix3d Rs_i = Rs_[i];

      Quaterniond rot_li(Rs_i * transform_lb.rot.inverse());
      Eigen::Vector3d pos_li = Ps_i - rot_li * transform_lb.pos;

      Twist<double> transform_li = Twist<double>(rot_li, pos_li);
      Eigen::Affine3f transform_pivot_i = (transform_pivot.inverse() * transform_li).cast<float>().transform();

      Transform local_transform = transform_pivot_i;
      local_transforms.push_back(local_transform);

      if (i < pivot_idx) {
        continue;
      }

      PointCloud transformed_cloud_surf, transformed_cloud_corner;
      //region fix the map
#ifdef FIX_MAP
      Eigen::Vector3d Ps_linearized_i = Ps_linearized_[i];
      Eigen::Matrix3d Rs_linearized_i = Rs_linearized_[i];

      Quaterniond rot_linearized_i(Rs_linearized_i * transform_lb.rot.inverse());
      Eigen::Vector3d pos_linearized_i = Ps_linearized_i - rot_linearized_i * transform_lb.pos;

      Twist<double> transform_linearized_i = Twist<double>(rot_linearized_i, pos_linearized_i);
      Eigen::Affine3f transform_linearized_pivot_i =
          (transform_linearized_pivot.inverse() * transform_linearized_i).cast<float>().transform();
      DLOG(INFO) << "transform_li: " << transform_li;
      DLOG(INFO) << "transform_linearized_i: " << transform_linearized_i;
#endif
      //endregion


      // NOTE: exclude the latest one
      if (i != estimator_config_.window_size) {
        if (i == pivot_idx) {
          *local_surf_points_ptr_ += *(surf_stack_[i]);
//          down_size_filter_surf_.setInputCloud(local_surf_points_ptr_);
//          down_size_filter_surf_.filter(transformed_cloud_surf);
//          *local_surf_points_ptr_ = transformed_cloud_surf;
#ifdef USE_CORNER
          *local_corner_points_ptr_ += *(corner_stack_[i]);
#endif
          continue;
        }
        //region fix the map
#ifdef FIX_MAP
        pcl::transformPointCloud(*(surf_stack_[i]), transformed_cloud_surf, transform_linearized_pivot_i);
#ifdef USE_CORNER
        pcl::transformPointCloud(*(corner_stack_[i]), transformed_cloud_corner, transform_linearized_pivot_i);
#endif
#else
        pcl::transformPointCloud(*(surf_stack_[i]), transformed_cloud_surf, transform_pivot_i);
#ifdef USE_CORNER
        pcl::transformPointCloud(*(corner_stack_[i]), transformed_cloud_corner, transform_pivot_i);
#endif
#endif
        //endregion
        for (int p_idx = 0; p_idx < transformed_cloud_surf.size(); ++p_idx) {
          transformed_cloud_surf[p_idx].intensity = i;
        }
        *local_surf_points_ptr_ += transformed_cloud_surf;
#ifdef USE_CORNER
        for (int p_idx = 0; p_idx < transformed_cloud_corner.size(); ++p_idx) {
          transformed_cloud_corner[p_idx].intensity = i;
        }
        *local_corner_points_ptr_ += transformed_cloud_corner;
#endif
      }
    }

    DLOG(INFO) << "local_surf_points_ptr_->size() bef: " << local_surf_points_ptr_->size();
    down_size_filter_surf_.setInputCloud(local_surf_points_ptr_);
    down_size_filter_surf_.filter(*local_surf_points_filtered_ptr_);
    DLOG(INFO) << "local_surf_points_ptr_->size() aft: " << local_surf_points_filtered_ptr_->size();

#ifdef USE_CORNER
    DLOG(INFO) << "local_corner_points_ptr_->size() bef: " << local_corner_points_ptr_->size();
    down_size_filter_corner_.setInputCloud(local_corner_points_ptr_);
    down_size_filter_corner_.filter(*local_corner_points_filtered_ptr_);
    DLOG(INFO) << "local_corner_points_ptr_->size() aft: " << local_corner_points_filtered_ptr_->size();
#endif

  }

  ROS_DEBUG_STREAM("t_build_map cost: " << t_build_map.Toc() << " ms");
  DLOG(INFO) << "t_build_map cost: " << t_build_map.Toc() << " ms";

  //region Visualization
  if (estimator_config_.pcl_viewer) {
    if (normal_vis.init) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr point_world_i_xyz(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud(*local_surf_points_filtered_ptr_, *point_world_i_xyz);
      normal_vis.UpdateCloud(point_world_i_xyz, "cloud_all");
    }
  }
  //endregion

  pcl::KdTreeFLANN<PointT>::Ptr kdtree_surf_from_map(new pcl::KdTreeFLANN<PointT>());
  kdtree_surf_from_map->setInputCloud(local_surf_points_filtered_ptr_);

#ifdef USE_CORNER
  pcl::KdTreeFLANN<PointT>::Ptr kdtree_corner_from_map(new pcl::KdTreeFLANN<PointT>());
  kdtree_corner_from_map->setInputCloud(local_corner_points_filtered_ptr_);
#endif

  for (int idx = 0; idx < estimator_config_.window_size + 1; ++idx) {

    FeaturePerFrame feature_per_frame;
    vector<unique_ptr<Feature>> features;
//    vector<unique_ptr<Feature>> &features = feature_per_frame.features;

    TicToc t_features;

    if (idx > pivot_idx) {
      if (idx != estimator_config_.window_size || !estimator_config_.imu_factor) {
#ifdef USE_CORNER
        CalculateFeatures(kdtree_surf_from_map, local_surf_points_filtered_ptr_, surf_stack_[idx],
                          kdtree_corner_from_map, local_corner_points_filtered_ptr_, corner_stack_[idx],
                          local_transforms[idx], features);
#else
        CalculateFeatures(kdtree_surf_from_map, local_surf_points_filtered_ptr_, surf_stack_[idx],
                          local_transforms[idx], features);
#endif
      } else {
        DLOG(INFO) << "local_transforms[idx] bef" << local_transforms[idx];

#ifdef USE_CORNER
        CalculateLaserOdom(kdtree_surf_from_map, local_surf_points_filtered_ptr_, surf_stack_[idx],
                           kdtree_corner_from_map, local_corner_points_filtered_ptr_, corner_stack_[idx],
                           local_transforms[idx], features);
#else
        CalculateLaserOdom(kdtree_surf_from_map, local_surf_points_filtered_ptr_, surf_stack_[idx],
                           local_transforms[idx], features);
#endif

        DLOG(INFO) << "local_transforms[idx] aft" << local_transforms[idx];

//      int pivot_idx = int(estimator_config_.window_size - estimator_config_.opt_window_size);
//
//      Twist<double> transform_lb = transform_lb_.cast<double>();
//
//      Eigen::Vector3d Ps_pivot = Ps_[pivot_idx];
//      Eigen::Matrix3d Rs_pivot = Rs_[pivot_idx];
//
//      Quaterniond rot_pivot(Rs_pivot * transform_lb.rot.inverse());
//      Eigen::Vector3d pos_pivot = Ps_pivot - rot_pivot * transform_lb.pos;
//
//      Twist<double> transform_pivot = Twist<double>(rot_pivot, pos_pivot);
//
//      Twist<double> transform_bi = transform_pivot * local_transforms[idx].cast<double>() * transform_lb;
//      Rs_[idx] = transform_bi.rot.normalized();
//      Ps_[idx] = transform_bi.pos;
      }
    } else {
      // NOTE: empty features
    }

    //region Visualization
    std::vector<Eigen::Vector4d,
                Eigen::aligned_allocator<Eigen::Vector4d >> plane_coeffs;
    if (estimator_config_.pcl_viewer) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud_sel(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::Normal>::Ptr tmp_normals_sel(new pcl::PointCloud<pcl::Normal>);
      for (int i = 0; i < features.size(); ++i) {
        PointPlaneFeature f;
        features[i]->GetFeature(&f);
        pcl::PointXYZI p_ori;
        p_ori.x = f.point.x();
        p_ori.y = f.point.y();
        p_ori.z = f.point.z();
        pcl::PointXYZI p_sel;
        PointAssociateToMap(p_ori, p_sel, local_transforms[idx]);
        tmp_cloud_sel->push_back(pcl::PointXYZ{p_sel.x, p_sel.y, p_sel.z});
        tmp_normals_sel->push_back(pcl::Normal{float(f.coeffs.x()), float(f.coeffs.y()),
                                               float(f.coeffs.z())});
        Eigen::Vector4d coeffs_normalized = f.coeffs;
        double s_normal = coeffs_normalized.head<3>().norm();
        coeffs_normalized = coeffs_normalized / s_normal;
        plane_coeffs.push_back(coeffs_normalized);
//        DLOG(INFO) << p_sel.x * f.coeffs.x() + p_sel.y * f.coeffs.y() + p_sel.z * f.coeffs.z() + f.coeffs.w();
      }

      if (normal_vis.init) {
//        pcl::PointCloud<pcl::PointXYZ>::Ptr point_world_i_xyz(new pcl::PointCloud<pcl::PointXYZ>);
//        pcl::copyPointCloud(p_world_i, *point_world_i_xyz);
        normal_vis.UpdateCloudAndNormals(tmp_cloud_sel, tmp_normals_sel, 10, "cloud1", "normal1");
//        normal_vis.UpdatePlanes(plane_coeffs);
      }
    }
    //endregion

    feature_per_frame.id = idx;
//    feature_per_frame.features = std::move(features);
    feature_per_frame.features.assign(make_move_iterator(features.begin()), make_move_iterator(features.end()));
    feature_frames.push_back(std::move(feature_per_frame));

    ROS_DEBUG_STREAM("feature cost: " << t_features.Toc() << " ms");
  }

}

void Estimator::SolveOptimization() {
  if (cir_buf_count_ < estimator_config_.window_size && estimator_config_.imu_factor) {
    LOG(ERROR) << "enter optimization before enough count: " << cir_buf_count_ << " < "
               << estimator_config_.window_size;
    return;
  }

  TicToc tic_toc_opt;

  bool turn_off = true;
//  Vector3d P_last0, P_last; /// for convergence check

  ceres::Problem problem;
  ceres::LossFunction *loss_function;
  // NOTE: indoor test
//  loss_function = new ceres::HuberLoss(0.5);
  loss_function = new ceres::CauchyLoss(1.0);

  // NOTE: update from laser transform
  if (estimator_config_.update_laser_imu) {
    DLOG(INFO) << "======= bef opt =======";

    if (!estimator_config_.imu_factor) {
      Twist<double>
          incre = (transform_lb_.inverse() * all_laser_transforms_[cir_buf_count_ - 1].second.transform.inverse()
          * all_laser_transforms_[cir_buf_count_].second.transform * transform_lb_).cast<double>();
      Ps_[cir_buf_count_] = Rs_[cir_buf_count_ - 1] * incre.pos + Ps_[cir_buf_count_ - 1];
      Rs_[cir_buf_count_] = Rs_[cir_buf_count_ - 1] * incre.rot;
    }

    Twist<double> transform_lb = transform_lb_.cast<double>();
    int pivot_idx = int(estimator_config_.window_size - estimator_config_.opt_window_size);

    Eigen::Vector3d Ps_pivot = Ps_[pivot_idx];
    Eigen::Vector3d Vs_pivot = Vs_[pivot_idx];
    Eigen::Matrix3d Rs_pivot = Rs_[pivot_idx];

    Quaterniond rot_pivot(Rs_pivot * transform_lb.rot.inverse());
    Eigen::Vector3d pos_pivot = Ps_pivot - rot_pivot * transform_lb.pos;

    Twist<double> transform_pivot = Twist<double>(rot_pivot, pos_pivot);

    vector<Transform> imu_poses, lidar_poses;

    for (int i = 0; i < estimator_config_.opt_window_size + 1; ++i) {
      int opt_i = int(estimator_config_.window_size - estimator_config_.opt_window_size + i);

      Quaterniond rot_li(Rs_[opt_i] * transform_lb.rot.inverse());
      Eigen::Vector3d pos_li = Ps_[opt_i] - rot_li * transform_lb.pos;
      Twist<double> transform_li = Twist<double>(rot_li, pos_li);

      // DLOG(INFO) << "Ps_[" << opt_i << "] bef: " << Ps_[opt_i].transpose();
      // DLOG(INFO) << "Vs_[" << opt_i << "]: bef " << Vs_[opt_i].transpose();
      /*
      DLOG(INFO) << "Vs_[" << opt_i << "]: " << Vs_[opt_i].transpose();
      DLOG(INFO) << "Rs_[" << opt_i << "]: " << Eigen::Quaterniond(Rs_[opt_i]).coeffs().transpose();
      DLOG(INFO) << "Bas_[" << opt_i << "]: " << Bas_[opt_i].transpose();
      DLOG(INFO) << "Bgs_[" << opt_i << "]: " << Bgs_[opt_i].transpose();
      */
      // DLOG(INFO) << "transform_lb_: " << transform_lb_;
      // DLOG(INFO) << "gravity in world: " << g_vec_.transpose();

      Twist<double> transform_bi = Twist<double>(Eigen::Quaterniond(Rs_[opt_i]), Ps_[opt_i]);
      imu_poses.push_back(transform_bi.cast<float>());
      lidar_poses.push_back(transform_li.cast<float>());
    }

    //region Check for imu res
//    for (int i = 0; i < estimator_config_.window_size; ++i) {
//
//      typedef Eigen::Matrix<double, 15, 15> M15;
//      typedef Eigen::Matrix<double, 15, 1> V15;
//      M15 sqrt_info =
//          Eigen::LLT<M15>(pre_integrations_[i + 1]->covariance_.inverse()).matrixL().transpose();
//
//      V15 res = (pre_integrations_[i + 1]->Evaluate(
//          Ps_[i], Eigen::Quaterniond(Rs_[i]), Vs_[i], Bas_[i], Bgs_[i + 1],
//          Ps_[i + 1], Eigen::Quaterniond(Rs_[i + 1]), Vs_[i + 1], Bas_[i + 1], Bgs_[i + 1]));
//      // DLOG(INFO) << "sqrt_info: " << endl << sqrt_info;
//
//      DLOG(INFO) << "imu res bef: " << res.transpose();
//      // DLOG(INFO) << "weighted pre: " << (sqrt_info * res).transpose();
//      // DLOG(INFO) << "weighted pre: " << (sqrt_info * res).squaredNorm();
//    }
    //endregion

    vis_bef_opt.UpdateMarkers(imu_poses, lidar_poses);
    vis_bef_opt.PublishMarkers();

    DLOG(INFO) << "====================================";
  }

  vector<FeaturePerFrame> feature_frames;

  BuildLocalMap(feature_frames);

  vector<double *> para_ids;

  //region Add pose and speed bias parameters
  for (int i = 0; i < estimator_config_.opt_window_size + 1;
       ++i) {
    ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
    problem.AddParameterBlock(para_pose_[i], SIZE_POSE, local_parameterization);
    problem.AddParameterBlock(para_speed_bias_[i], SIZE_SPEED_BIAS);
    para_ids.push_back(para_pose_[i]);
    para_ids.push_back(para_speed_bias_[i]);
  }
  //endregion

  //region Add extrinsic parameters
  {
    ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
    problem.AddParameterBlock(para_ex_pose_, SIZE_POSE, local_parameterization);
    para_ids.push_back(para_ex_pose_);
    if (extrinsic_stage_ == 0 || estimator_config_.opt_extrinsic == false) {
      DLOG(INFO) << "fix extrinsic param";
      problem.SetParameterBlockConstant(para_ex_pose_);
    } else {
      DLOG(INFO) << "estimate extrinsic param";
    }
  }
  //endregion

//  P_last0 = Ps_.last();

  VectorToDouble();

  vector<ceres::internal::ResidualBlock *> res_ids_marg;
  ceres::internal::ResidualBlock *res_id_marg = NULL;

  //region Marginalization residual
  if (estimator_config_.marginalization_factor) {
    if (last_marginalization_info) {
      // construct new marginlization_factor
      MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
      res_id_marg = problem.AddResidualBlock(marginalization_factor, NULL,
                                             last_marginalization_parameter_blocks);
      res_ids_marg.push_back(res_id_marg);
    }
  }
  //endregion

  vector<ceres::internal::ResidualBlock *> res_ids_pim;

  if (estimator_config_.imu_factor) {

    for (int i = 0; i < estimator_config_.opt_window_size;
         ++i) {
      int j = i + 1;
      int opt_i = int(estimator_config_.window_size - estimator_config_.opt_window_size + i);
      int opt_j = opt_i + 1;
      if (pre_integrations_[opt_j]->sum_dt_ > 10.0) {
        continue;
      }

      ImuFactor *f = new ImuFactor(pre_integrations_[opt_j]);
//    {
//      double **tmp_parameters = new double *[5];
//      tmp_parameters[0] = para_pose_[i];
//      tmp_parameters[1] = para_speed_bias_[i];
//      tmp_parameters[2] = para_pose_[j];
//      tmp_parameters[3] = para_speed_bias_[j];
//      tmp_parameters[4] = para_qwi_;
//      f->Check(tmp_parameters);
//      delete[] tmp_parameters;
//    }

      // TODO: is it better to use g_vec_ as global parameter?
      ceres::internal::ResidualBlock *res_id =
          problem.AddResidualBlock(f,
                                   NULL,
                                   para_pose_[i],
                                   para_speed_bias_[i],
                                   para_pose_[j],
                                   para_speed_bias_[j]
          );

      res_ids_pim.push_back(res_id);
    }
  }

  vector<ceres::internal::ResidualBlock *> res_ids_proj;

  if (estimator_config_.point_distance_factor) {
    for (int i = 0; i < estimator_config_.opt_window_size + 1; ++i) {
      int opt_i = int(estimator_config_.window_size - estimator_config_.opt_window_size + i);

      FeaturePerFrame &feature_per_frame = feature_frames[opt_i];
      LOG_ASSERT(opt_i == feature_per_frame.id);

      vector<unique_ptr<Feature>> &features = feature_per_frame.features;

      DLOG(INFO) << "features.size(): " << features.size();

      for (int j = 0; j < features.size(); ++j) {
        PointPlaneFeature feature_j;
        features[j]->GetFeature(&feature_j);

        const double &s = feature_j.score;

        const Eigen::Vector3d &p_eigen = feature_j.point;
        const Eigen::Vector4d &coeff_eigen = feature_j.coeffs;

        Eigen::Matrix<double, 6, 6> info_mat_in;

        if (i == 0) {
//          Eigen::Matrix<double, 6, 6> mat_in;
//          PointDistanceFactor *f = new PointDistanceFactor(p_eigen,
//                                                           coeff_eigen,
//                                                           mat_in);
//          ceres::internal::ResidualBlock *res_id =
//              problem.AddResidualBlock(f,
//                                       loss_function,
////                                     NULL,
//                                       para_pose_[i],
//                                       para_ex_pose_);
//
//          res_ids_proj.push_back(res_id);
        } else {
          PivotPointPlaneFactor *f = new PivotPointPlaneFactor(p_eigen,
                                                               coeff_eigen);
          ceres::internal::ResidualBlock *res_id =
              problem.AddResidualBlock(f,
                                       loss_function,
//                                     NULL,
                                       para_pose_[0],
                                       para_pose_[i],
                                       para_ex_pose_);

          res_ids_proj.push_back(res_id);
        }

//      {
//        double **tmp_parameters = new double *[3];
//        tmp_parameters[0] = para_pose_[0];
//        tmp_parameters[1] = para_pose_[i];
//        tmp_parameters[2] = para_ex_pose_;
//        f->Check(tmp_parameters);
//      }
      }
    }
  }

  if (estimator_config_.prior_factor) {
    {
      Twist<double> trans_tmp = transform_lb_.cast<double>();
      PriorFactor *f = new PriorFactor(trans_tmp.pos, trans_tmp.rot);
      problem.AddResidualBlock(f,
                               NULL,
                               para_ex_pose_);
      //    {
      //      double **tmp_parameters = new double *[1];
      //      tmp_parameters[0] = para_ex_pose_;
      //      f->Check(tmp_parameters);
      //    }
    }
  }

  DLOG(INFO) << "prepare for ceres: " << tic_toc_opt.Toc() << " ms";
  ROS_DEBUG_STREAM("prepare for ceres: " << tic_toc_opt.Toc() << " ms");

  ceres::Solver::Options options;

  options.linear_solver_type = ceres::DENSE_SCHUR;
//  options.linear_solver_type = ceres::DENSE_QR;
//  options.num_threads = 8;
  options.trust_region_strategy_type = ceres::DOGLEG;
//  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.max_num_iterations = 10;
  //options.use_explicit_schur_complement = true;
  //options.minimizer_progress_to_stdout = true;
  //options.use_nonmonotonic_steps = true;

  options.max_solver_time_in_seconds = 0.10;

  //region residual before optimization
  {
    double cost_pim = 0.0, cost_ppp = 0.0, cost_marg = 0.0;
    ///< Bef
    ceres::Problem::EvaluateOptions e_option;
    if (estimator_config_.imu_factor) {
      e_option.parameter_blocks = para_ids;
      e_option.residual_blocks = res_ids_pim;
      problem.Evaluate(e_option, &cost_pim, NULL, NULL, NULL);
      DLOG(INFO) << "bef_pim: " << cost_pim;

//      if (cost > 1e3 || !convergence_flag_) {
      if (cost_pim > 1e3) {
        turn_off = true;
      } else {
        turn_off = false;
      }
    }
    if (estimator_config_.point_distance_factor) {
      e_option.parameter_blocks = para_ids;
      e_option.residual_blocks = res_ids_proj;
      problem.Evaluate(e_option, &cost_ppp, NULL, NULL, NULL);
      DLOG(INFO) << "bef_proj: " << cost_ppp;
    }
    if (estimator_config_.marginalization_factor) {
      if (last_marginalization_info) {
        e_option.parameter_blocks = para_ids;
        e_option.residual_blocks = res_ids_marg;
        problem.Evaluate(e_option, &cost_marg, NULL, NULL, NULL);
        DLOG(INFO) << "bef_marg: " << cost_marg;
        ///>
      }
    }

    {
      double ratio = cost_marg / (cost_ppp + cost_pim);

      if (!convergence_flag_ && !turn_off && ratio <= 2 && ratio != 0) {
        DLOG(WARNING) << "CONVERGE RATIO: " << ratio;
        convergence_flag_ = true;
      }

      if (!convergence_flag_) {
        ///<
        problem.SetParameterBlockConstant(para_ex_pose_);
        DLOG(WARNING) << "TURN OFF EXTRINSIC AND MARGINALIZATION";
        DLOG(WARNING) << "RATIO: " << ratio;

        if (last_marginalization_info) {
          delete last_marginalization_info;
          last_marginalization_info = nullptr;
        }

        if (res_id_marg) {
          problem.RemoveResidualBlock(res_id_marg);
          res_ids_marg.clear();
        }
      }

    }

  }
  //endregion

  TicToc t_opt;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  DLOG(INFO) << summary.BriefReport();

  ROS_DEBUG_STREAM("t_opt: " << t_opt.Toc() << " ms");
  DLOG(INFO) <<"t_opt: " << t_opt.Toc() << " ms";

  //region residual after optimization
  {
    ///< Aft
    double cost = 0.0;
    ceres::Problem::EvaluateOptions e_option;
    if (estimator_config_.imu_factor) {
      e_option.parameter_blocks = para_ids;
      e_option.residual_blocks = res_ids_pim;
      problem.Evaluate(e_option, &cost, NULL, NULL, NULL);
      DLOG(INFO) << "aft_pim: " << cost;
    }
    if (estimator_config_.point_distance_factor) {
      e_option.parameter_blocks = para_ids;
      e_option.residual_blocks = res_ids_proj;
      problem.Evaluate(e_option, &cost, NULL, NULL, NULL);
      DLOG(INFO) << "aft_proj: " << cost;
    }
    if (estimator_config_.marginalization_factor) {
      if (last_marginalization_info && !res_ids_marg.empty()) {
        e_option.parameter_blocks = para_ids;
        e_option.residual_blocks = res_ids_marg;
        problem.Evaluate(e_option, &cost, NULL, NULL, NULL);
        DLOG(INFO) << "aft_marg: " << cost;
      }
    }
  }
  //endregion

  // FIXME: Is marginalization needed in this framework? Yes, needed for extrinsic parameters.

  DoubleToVector();

//  P_last = Ps_.last();
//  if ((P_last - P_last0).norm() < 0.1) {
//    convergence_flag_ = true;
//  } else {
//    convergence_flag_ = false;
//    if (last_marginalization_info) {
//      delete last_marginalization_info;
//      last_marginalization_info = nullptr;
//    }
//  }

  //region Constraint Marginalization
  if (estimator_config_.marginalization_factor && !turn_off) {

    TicToc t_whole_marginalization;

    MarginalizationInfo *marginalization_info = new MarginalizationInfo();

//    {
//      MarginalizationInfo *marginalization_info0 = new MarginalizationInfo();
//      if (last_marginalization_info) {
//        vector<int> drop_set;
//        for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++) {
//          if (last_marginalization_parameter_blocks[i] == para_pose_[0] ||
//              last_marginalization_parameter_blocks[i] == para_speed_bias_[0])
//            drop_set.push_back(i);
//        }
//        // construct new marginlization_factor
//        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
//        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
//                                                                       last_marginalization_parameter_blocks,
//                                                                       drop_set);
//
//        marginalization_info0->AddResidualBlockInfo(residual_block_info);
//      }
//
//      if (estimator_config_.imu_factor) {
//        int pivot_idx = estimator_config_.window_size - estimator_config_.opt_window_size;
//        if (pre_integrations_[pivot_idx + 1]->sum_dt_ < 10.0) {
//          ImuFactor *imu_factor = new ImuFactor(pre_integrations_[pivot_idx + 1]);
//          ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
//                                                                         vector<double *>{para_pose_[0],
//                                                                                          para_speed_bias_[0],
//                                                                                          para_pose_[1],
//                                                                                          para_speed_bias_[1]},
//                                                                         vector<int>{0, 1});
//          marginalization_info0->AddResidualBlockInfo(residual_block_info);
//        }
//      }
//
//      if (estimator_config_.point_distance_factor) {
//        for (int i = 1; i < estimator_config_.opt_window_size + 1; ++i) {
//          int opt_i = int(estimator_config_.window_size - estimator_config_.opt_window_size + i);
//
//          FeaturePerFrame &feature_per_frame = feature_frames[opt_i];
//          LOG_ASSERT(opt_i == feature_per_frame.id);
//
//          vector<unique_ptr<Feature>> &features = feature_per_frame.features;
//
////        DLOG(INFO) << "features.size(): " << features.size();
//
//          for (int j = 0; j < features.size(); ++j) {
//
//            PointPlaneFeature feature_j;
//            features[j]->GetFeature(&feature_j);
//
//            const double &s = feature_j.score;
//
//            const Eigen::Vector3d &p_eigen = feature_j.point;
//            const Eigen::Vector4d &coeff_eigen = feature_j.coeffs;
//
//            PivotPointPlaneFactor *pivot_point_plane_factor = new PivotPointPlaneFactor(p_eigen,
//                                                                                        coeff_eigen);
//
//            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(pivot_point_plane_factor, loss_function,
//                                                                           vector<double *>{para_pose_[0],
//                                                                                            para_pose_[i],
//                                                                                            para_ex_pose_},
//                                                                           vector<int>{0});
//            marginalization_info0->AddResidualBlockInfo(residual_block_info);
//
//          }
//
//        }
//      }
//
//      TicToc t_pre_margin;
//      marginalization_info0->PreMarginalize();
//      DLOG(INFO) << "pre marginalization: " << t_pre_margin.Toc();
//
//      TicToc t_margin;
//      marginalization_info0->Marginalize();
//      DLOG(INFO) << "marginalization: " << t_margin.Toc();
//
//      {
//        std::unordered_map<long, double *> addr_shift2;
//        for (int i = 1; i < estimator_config_.opt_window_size + 1; ++i) {
//          addr_shift2[reinterpret_cast<long>(para_pose_[i])] = para_pose_[i];
//          addr_shift2[reinterpret_cast<long>(para_speed_bias_[i])] = para_speed_bias_[i];
//        }
//        addr_shift2[reinterpret_cast<long>(para_ex_pose_)] = para_ex_pose_;
//
//        vector<double *> parameter_blocks2 = marginalization_info0->GetParameterBlocks(addr_shift2);
//
//        vector<ceres::internal::ResidualBlock *> res_ids_marg2;
//        ceres::internal::ResidualBlock *res_id_marg2 = NULL;
////      ceres::Problem problem2;
//        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(marginalization_info0);
//        res_id_marg2 = problem.AddResidualBlock(marginalization_factor, NULL,
//                                                parameter_blocks2);
//        res_ids_marg2.push_back(res_id_marg2);
//
//        double aft_cost_marg;
//        ceres::Problem::EvaluateOptions e_option;
//        e_option.parameter_blocks = para_ids;
//        e_option.residual_blocks = res_ids_marg2;
//        problem.Evaluate(e_option, &aft_cost_marg, NULL, NULL, NULL);
//        DLOG(INFO) << "bef_cost_marg: " << aft_cost_marg;
//      }
//
//      if (marginalization_info0)
//        delete marginalization_info0;
//    }

    VectorToDouble();

    if (last_marginalization_info) {
      vector<int> drop_set;
      for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++) {
        if (last_marginalization_parameter_blocks[i] == para_pose_[0] ||
            last_marginalization_parameter_blocks[i] == para_speed_bias_[0])
          drop_set.push_back(i);
      }
      // construct new marginlization_factor
      MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
      ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                     last_marginalization_parameter_blocks,
                                                                     drop_set);

      marginalization_info->AddResidualBlockInfo(residual_block_info);
    }

    if (estimator_config_.imu_factor) {
      int pivot_idx = estimator_config_.window_size - estimator_config_.opt_window_size;
      if (pre_integrations_[pivot_idx + 1]->sum_dt_ < 10.0) {
        ImuFactor *imu_factor = new ImuFactor(pre_integrations_[pivot_idx + 1]);
        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                       vector<double *>{para_pose_[0],
                                                                                        para_speed_bias_[0],
                                                                                        para_pose_[1],
                                                                                        para_speed_bias_[1]},
                                                                       vector<int>{0, 1});
        marginalization_info->AddResidualBlockInfo(residual_block_info);
      }
    }

    if (estimator_config_.point_distance_factor) {
      for (int i = 1; i < estimator_config_.opt_window_size + 1; ++i) {
        int opt_i = int(estimator_config_.window_size - estimator_config_.opt_window_size + i);

        FeaturePerFrame &feature_per_frame = feature_frames[opt_i];
        LOG_ASSERT(opt_i == feature_per_frame.id);

        vector<unique_ptr<Feature>> &features = feature_per_frame.features;

//        DLOG(INFO) << "features.size(): " << features.size();

        for (int j = 0; j < features.size(); ++j) {

          PointPlaneFeature feature_j;
          features[j]->GetFeature(&feature_j);

          const double &s = feature_j.score;

          const Eigen::Vector3d &p_eigen = feature_j.point;
          const Eigen::Vector4d &coeff_eigen = feature_j.coeffs;

          PivotPointPlaneFactor *pivot_point_plane_factor = new PivotPointPlaneFactor(p_eigen,
                                                                                      coeff_eigen);

          ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(pivot_point_plane_factor, loss_function,
                                                                         vector<double *>{para_pose_[0],
                                                                                          para_pose_[i],
                                                                                          para_ex_pose_},
                                                                         vector<int>{0});
          marginalization_info->AddResidualBlockInfo(residual_block_info);

        }

      }
    }

    TicToc t_pre_margin;
    marginalization_info->PreMarginalize();
    ROS_DEBUG("pre marginalization %f ms", t_pre_margin.Toc());
    ROS_DEBUG_STREAM("pre marginalization: " << t_pre_margin.Toc() << " ms");

    TicToc t_margin;
    marginalization_info->Marginalize();
    ROS_DEBUG("marginalization %f ms", t_margin.Toc());
    ROS_DEBUG_STREAM("marginalization: " << t_margin.Toc() << " ms");

    std::unordered_map<long, double *> addr_shift;
    for (int i = 1; i < estimator_config_.opt_window_size + 1; ++i) {
      addr_shift[reinterpret_cast<long>(para_pose_[i])] = para_pose_[i - 1];
      addr_shift[reinterpret_cast<long>(para_speed_bias_[i])] = para_speed_bias_[i - 1];
    }

    addr_shift[reinterpret_cast<long>(para_ex_pose_)] = para_ex_pose_;

    vector<double *> parameter_blocks = marginalization_info->GetParameterBlocks(addr_shift);

    if (last_marginalization_info) {
      delete last_marginalization_info;
    }
    last_marginalization_info = marginalization_info;
    last_marginalization_parameter_blocks = parameter_blocks;

    DLOG(INFO) << "whole marginalization costs: " << t_whole_marginalization.Toc();
    ROS_DEBUG_STREAM("whole marginalization costs: " << t_whole_marginalization.Toc() << " ms");

//    {
//      std::unordered_map<long, double *> addr_shift2;
//      for (int i = 1; i < estimator_config_.opt_window_size + 1; ++i) {
//        addr_shift2[reinterpret_cast<long>(para_pose_[i])] = para_pose_[i];
//        addr_shift2[reinterpret_cast<long>(para_speed_bias_[i])] = para_speed_bias_[i];
//      }
//      addr_shift2[reinterpret_cast<long>(para_ex_pose_)] = para_ex_pose_;
//
//      vector<double *> parameter_blocks2 = marginalization_info->GetParameterBlocks(addr_shift2);
//
//      vector<ceres::internal::ResidualBlock *> res_ids_marg2;
//      ceres::internal::ResidualBlock *res_id_marg2 = NULL;
////      ceres::Problem problem2;
//      MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
//      res_id_marg2 = problem.AddResidualBlock(marginalization_factor, NULL,
//                                              parameter_blocks2);
//      res_ids_marg2.push_back(res_id_marg2);
//
//      double aft_cost_marg;
//      ceres::Problem::EvaluateOptions e_option;
//      e_option.parameter_blocks = para_ids;
//      e_option.residual_blocks = res_ids_marg2;
//      problem.Evaluate(e_option, &aft_cost_marg, NULL, NULL, NULL);
//      DLOG(INFO) << "aft_cost_marg: " << aft_cost_marg;
//    }

  }
  //endregion

  // NOTE: update to laser transform
  if (estimator_config_.update_laser_imu) {
    DLOG(INFO) << "======= aft opt =======";
    Twist<double> transform_lb = transform_lb_.cast<double>();
    Transform &opt_l0_transform = opt_transforms_[0];
    int opt_0 = int(estimator_config_.window_size - estimator_config_.opt_window_size + 0);
    Quaterniond rot_l0(Rs_[opt_0] * transform_lb.rot.conjugate().normalized());
    Eigen::Vector3d pos_l0 = Ps_[opt_0] - rot_l0 * transform_lb.pos;
    opt_l0_transform = Twist<double>{rot_l0, pos_l0}.cast<float>(); // for updating the map

    vector<Transform> imu_poses, lidar_poses;

    for (int i = 0; i < estimator_config_.opt_window_size + 1; ++i) {
      int opt_i = int(estimator_config_.window_size - estimator_config_.opt_window_size + i);

      Quaterniond rot_li(Rs_[opt_i] * transform_lb.rot.conjugate().normalized());
      Eigen::Vector3d pos_li = Ps_[opt_i] - rot_li * transform_lb.pos;
      Twist<double> transform_li = Twist<double>(rot_li, pos_li);

      // DLOG(INFO) << "Ps_[" << opt_i << "]: " << Ps_[opt_i].transpose();
      // DLOG(INFO) << "Vs_[" << opt_i << "]: " << Vs_[opt_i].transpose();
      /*
      DLOG(INFO) << "Vs_[" << opt_i << "]: " << Vs_[opt_i].transpose();
      DLOG(INFO) << "Rs_[" << opt_i << "]: " << Eigen::Quaterniond(Rs_[opt_i]).coeffs().transpose();
      DLOG(INFO) << "Bas_[" << opt_i << "]: " << Bas_[opt_i].transpose();
      DLOG(INFO) << "Bgs_[" << opt_i << "]: " << Bgs_[opt_i].transpose();
      */
//      DLOG(INFO) << "velocity: " << Vs_.last().norm();
//      DLOG(INFO) << "transform_lb_: " << transform_lb_;
      // DLOG(INFO) << "gravity in world: " << g_vec_.transpose();

      Twist<double> transform_bi = Twist<double>(Eigen::Quaterniond(Rs_[opt_i]), Ps_[opt_i]);
      imu_poses.push_back(transform_bi.cast<float>());
      lidar_poses.push_back(transform_li.cast<float>());

    }

    DLOG(INFO) << "velocity: " << Vs_.last().norm();
    DLOG(INFO) << "transform_lb_: " << transform_lb_;

    ROS_DEBUG_STREAM("lb in world: " << (rot_l0.normalized() * transform_lb.pos).transpose());

    //region Check for imu res
//    for (int i = 0; i < estimator_config_.window_size; ++i) {
//
//      typedef Eigen::Matrix<double, 15, 15> M15;
//      typedef Eigen::Matrix<double, 15, 1> V15;
//      M15 sqrt_info =
//          Eigen::LLT<M15>(pre_integrations_[i + 1]->covariance_.inverse()).matrixL().transpose();
//
//      V15 res = (pre_integrations_[i + 1]->Evaluate(
//          Ps_[i], Eigen::Quaterniond(Rs_[i]), Vs_[i], Bas_[i], Bgs_[i + 1],
//          Ps_[i + 1], Eigen::Quaterniond(Rs_[i + 1]), Vs_[i + 1], Bas_[i + 1], Bgs_[i + 1]));
//      // DLOG(INFO) << "sqrt_info: " << endl << sqrt_info;
//
//      DLOG(INFO) << "imu res aft: " << res.transpose();
//      // DLOG(INFO) << "weighted pre: " << (sqrt_info * res).transpose();
//      // DLOG(INFO) << "weighted pre: " << (sqrt_info * res).squaredNorm();
//    }
    //endregion

    vis_aft_opt.UpdateMarkers(imu_poses, lidar_poses);
    vis_aft_opt.UpdateVelocity(Vs_.last().norm());
    vis_aft_opt.PublishMarkers();

    {
      geometry_msgs::PoseStamped ex_lb_msg;
      ex_lb_msg.header = Headers_.last();
      ex_lb_msg.pose.position.x = transform_lb.pos.x();
      ex_lb_msg.pose.position.y = transform_lb.pos.y();
      ex_lb_msg.pose.position.z = transform_lb.pos.z();
      ex_lb_msg.pose.orientation.w = transform_lb.rot.w();
      ex_lb_msg.pose.orientation.x = transform_lb.rot.x();
      ex_lb_msg.pose.orientation.y = transform_lb.rot.y();
      ex_lb_msg.pose.orientation.z = transform_lb.rot.z();
      pub_extrinsic_.publish(ex_lb_msg);

      int pivot_idx = estimator_config_.window_size - estimator_config_.opt_window_size;

      Eigen::Vector3d Ps_pivot = Ps_[pivot_idx];
      Eigen::Matrix3d Rs_pivot = Rs_[pivot_idx];

      Quaterniond rot_pivot(Rs_pivot * transform_lb.rot.inverse());
      Eigen::Vector3d pos_pivot = Ps_pivot - rot_pivot * transform_lb.pos;
      PublishCloudMsg(pub_local_surf_points_,
                      *surf_stack_[pivot_idx + 1],
                      Headers_[pivot_idx + 1].stamp,
                      "/laser_local");

      PublishCloudMsg(pub_local_corner_points_,
                      *corner_stack_[pivot_idx + 1],
                      Headers_[pivot_idx + 1].stamp,
                      "/laser_local");

      PublishCloudMsg(pub_local_full_points_,
                      *full_stack_[pivot_idx + 1],
                      Headers_[pivot_idx + 1].stamp,
                      "/laser_local");

      PublishCloudMsg(pub_map_surf_points_,
                      *local_surf_points_filtered_ptr_,
                      Headers_.last().stamp,
                      "/laser_local");

#ifdef USE_CORNER
      PublishCloudMsg(pub_map_corner_points_,
                      *local_corner_points_filtered_ptr_,
                      Headers_.last().stamp,
                      "/laser_local");
#endif

      laser_local_trans_.setOrigin(tf::Vector3{pos_pivot.x(), pos_pivot.y(), pos_pivot.z()});
      laser_local_trans_.setRotation(tf::Quaternion{rot_pivot.x(), rot_pivot.y(), rot_pivot.z(), rot_pivot.w()});
      laser_local_trans_.stamp_ = Headers_.last().stamp;
      tf_broadcaster_est_.sendTransform(laser_local_trans_);

      Eigen::Vector3d Ps_last = Ps_.last();
      Eigen::Matrix3d Rs_last = Rs_.last();

      Quaterniond rot_last(Rs_last * transform_lb.rot.inverse());
      Eigen::Vector3d pos_last = Ps_last - rot_last * transform_lb.pos;

      Quaterniond rot_predict = (rot_pivot.inverse() * rot_last).normalized();
      Eigen::Vector3d pos_predict = rot_pivot.inverse() * (Ps_last - Ps_pivot);

      PublishCloudMsg(pub_predict_surf_points_, *(surf_stack_.last()), Headers_.last().stamp, "/laser_predict");
      PublishCloudMsg(pub_predict_full_points_, *(full_stack_.last()), Headers_.last().stamp, "/laser_predict");

      {
        // NOTE: full stack into end of the scan
//        PointCloudPtr tmp_points_ptr = boost::make_shared<PointCloud>(PointCloud());
//        *tmp_points_ptr = *(full_stack_.last());
//        TransformToEnd(tmp_points_ptr, transform_es_, 10);
//        PublishCloudMsg(pub_predict_corrected_full_points_,
//                        *tmp_points_ptr,
//                        Headers_.last().stamp,
//                        "/laser_predict");

        TransformToEnd(full_stack_.last(), transform_es_, 10, true);
        PublishCloudMsg(pub_predict_corrected_full_points_,
                        *(full_stack_.last()),
                        Headers_.last().stamp,
                        "/laser_predict");
      }

#ifdef USE_CORNER
      PublishCloudMsg(pub_predict_corner_points_, *(corner_stack_.last()), Headers_.last().stamp, "/laser_predict");
#endif
      laser_predict_trans_.setOrigin(tf::Vector3{pos_predict.x(), pos_predict.y(), pos_predict.z()});
      laser_predict_trans_.setRotation(tf::Quaternion{rot_predict.x(), rot_predict.y(), rot_predict.z(),
                                                      rot_predict.w()});
      laser_predict_trans_.stamp_ = Headers_.last().stamp;
      tf_broadcaster_est_.sendTransform(laser_predict_trans_);
    }

  }

  DLOG(INFO) << "tic_toc_opt: " << tic_toc_opt.Toc() << " ms";
  ROS_DEBUG_STREAM("tic_toc_opt: " << tic_toc_opt.Toc() << " ms");

}

void Estimator::VectorToDouble() {
  int i, opt_i, pivot_idx = int(estimator_config_.window_size - estimator_config_.opt_window_size);
  P_pivot_ = Ps_[pivot_idx];
  R_pivot_ = Rs_[pivot_idx];
  for (i = 0, opt_i = pivot_idx; i < estimator_config_.opt_window_size + 1; ++i, ++opt_i) {
    para_pose_[i][0] = Ps_[opt_i].x();
    para_pose_[i][1] = Ps_[opt_i].y();
    para_pose_[i][2] = Ps_[opt_i].z();
    Quaterniond q{Rs_[opt_i]};
    para_pose_[i][3] = q.x();
    para_pose_[i][4] = q.y();
    para_pose_[i][5] = q.z();
    para_pose_[i][6] = q.w();

    para_speed_bias_[i][0] = Vs_[opt_i].x();
    para_speed_bias_[i][1] = Vs_[opt_i].y();
    para_speed_bias_[i][2] = Vs_[opt_i].z();

    para_speed_bias_[i][3] = Bas_[opt_i].x();
    para_speed_bias_[i][4] = Bas_[opt_i].y();
    para_speed_bias_[i][5] = Bas_[opt_i].z();

    para_speed_bias_[i][6] = Bgs_[opt_i].x();
    para_speed_bias_[i][7] = Bgs_[opt_i].y();
    para_speed_bias_[i][8] = Bgs_[opt_i].z();
  }

  {
    /// base to lidar
    para_ex_pose_[0] = transform_lb_.pos.x();
    para_ex_pose_[1] = transform_lb_.pos.y();
    para_ex_pose_[2] = transform_lb_.pos.z();
    para_ex_pose_[3] = transform_lb_.rot.x();
    para_ex_pose_[4] = transform_lb_.rot.y();
    para_ex_pose_[5] = transform_lb_.rot.z();
    para_ex_pose_[6] = transform_lb_.rot.w();
  }
}

void Estimator::DoubleToVector() {

// FIXME: do we need to optimize the first state?
// WARNING: not just yaw angle rot_diff; if it is compared with global features, there should be no need for rot_diff

//  Quaterniond origin_R0{Rs_[0]};
  int pivot_idx = int(estimator_config_.window_size - estimator_config_.opt_window_size);
  Vector3d origin_P0 = Ps_[pivot_idx];
  Vector3d origin_R0 = R2ypr(Rs_[pivot_idx]);

  Vector3d origin_R00 = R2ypr(Quaterniond(para_pose_[0][6],
                                          para_pose_[0][3],
                                          para_pose_[0][4],
                                          para_pose_[0][5]).normalized().toRotationMatrix());
  // Z-axix R00 to R0, regard para_pose's R as rotate along the Z-axis first
  double y_diff = origin_R0.x() - origin_R00.x();

  //TODO
  Matrix3d rot_diff = ypr2R(Vector3d(y_diff, 0, 0));
  if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0) {
    ROS_DEBUG("euler singular point!");
    rot_diff = Rs_[pivot_idx] * Quaterniond(para_pose_[0][6],
                                            para_pose_[0][3],
                                            para_pose_[0][4],
                                            para_pose_[0][5]).normalized().toRotationMatrix().transpose();
  }

//  DLOG(INFO) << "origin_P0" << origin_P0.transpose();

  {
    Eigen::Vector3d Ps_pivot = Ps_[pivot_idx];
    Eigen::Matrix3d Rs_pivot = Rs_[pivot_idx];
    Twist<double> trans_pivot{Eigen::Quaterniond{Rs_pivot}, Ps_pivot};

    Matrix3d R_opt_pivot = rot_diff * Quaterniond(para_pose_[0][6],
                                                  para_pose_[0][3],
                                                  para_pose_[0][4],
                                                  para_pose_[0][5]).normalized().toRotationMatrix();
    Vector3d P_opt_pivot = origin_P0;

    Twist<double> trans_opt_pivot{Eigen::Quaterniond{R_opt_pivot}, P_opt_pivot};
    for (int idx = 0; idx < pivot_idx; ++idx) {
      Twist<double> trans_idx{Eigen::Quaterniond{Rs_[idx]}, Ps_[idx]};
      Twist<double> trans_opt_idx = trans_opt_pivot * trans_pivot.inverse() * trans_idx;
      Ps_[idx] = trans_opt_idx.pos;
      Rs_[idx] = trans_opt_idx.rot.normalized().toRotationMatrix();

      // wrong -- below
//      Twist<double> trans_pivot_idx = trans_pivot.inverse() * trans_idx;
//      Ps_[idx] = rot_diff * trans_pivot_idx.pos + origin_P0;
//      Rs_[idx] = rot_diff * trans_pivot_idx.rot.normalized().toRotationMatrix();
    }

  }

  int i, opt_i;
  for (i = 0, opt_i = pivot_idx; i < estimator_config_.opt_window_size + 1; ++i, ++opt_i) {
//    DLOG(INFO) << "para aft: " << Vector3d(para_pose_[i][0], para_pose_[i][1], para_pose_[i][2]).transpose();

    Rs_[opt_i] = rot_diff * Quaterniond(para_pose_[i][6],
                                        para_pose_[i][3],
                                        para_pose_[i][4],
                                        para_pose_[i][5]).normalized().toRotationMatrix();

    Ps_[opt_i] = rot_diff * Vector3d(para_pose_[i][0] - para_pose_[0][0],
                                     para_pose_[i][1] - para_pose_[0][1],
                                     para_pose_[i][2] - para_pose_[0][2]) + origin_P0;

    Vs_[opt_i] = rot_diff * Vector3d(para_speed_bias_[i][0],
                                     para_speed_bias_[i][1],
                                     para_speed_bias_[i][2]);

    Bas_[opt_i] = Vector3d(para_speed_bias_[i][3],
                           para_speed_bias_[i][4],
                           para_speed_bias_[i][5]);

    Bgs_[opt_i] = Vector3d(para_speed_bias_[i][6],
                           para_speed_bias_[i][7],
                           para_speed_bias_[i][8]);
  }
  {
    transform_lb_.pos = Vector3d(para_ex_pose_[0],
                                 para_ex_pose_[1],
                                 para_ex_pose_[2]).template cast<float>();
    transform_lb_.rot = Quaterniond(para_ex_pose_[6],
                                    para_ex_pose_[3],
                                    para_ex_pose_[4],
                                    para_ex_pose_[5]).template cast<float>();
  }
}

void Estimator::SlideWindow() { // NOTE: this function is only for the states and the local map

  {
    if (init_local_map_) {
      int pivot_idx = estimator_config_.window_size - estimator_config_.opt_window_size;

      Twist<double> transform_lb = transform_lb_.cast<double>();

      Eigen::Vector3d Ps_pivot = Ps_[pivot_idx];
      Eigen::Matrix3d Rs_pivot = Rs_[pivot_idx];

      Quaterniond rot_pivot(Rs_pivot * transform_lb.rot.inverse());
      Eigen::Vector3d pos_pivot = Ps_pivot - rot_pivot * transform_lb.pos;

      Twist<double> transform_pivot = Twist<double>(rot_pivot, pos_pivot);

      PointCloudPtr transformed_cloud_surf_ptr(new PointCloud);
      PointCloudPtr transformed_cloud_corner_ptr(new PointCloud);
      PointCloud filtered_surf_points;
      PointCloud filtered_corner_points;

      int i = pivot_idx + 1; // the index of the next pivot
      Eigen::Vector3d Ps_i = Ps_[i];
      Eigen::Matrix3d Rs_i = Rs_[i];

      Quaterniond rot_li(Rs_i * transform_lb.rot.inverse());
      Eigen::Vector3d pos_li = Ps_i - rot_li * transform_lb.pos;

      Twist<double> transform_li = Twist<double>(rot_li, pos_li);
      Eigen::Affine3f transform_i_pivot = (transform_li.inverse() * transform_pivot).cast<float>().transform();
      pcl::ExtractIndices<PointT> extract;

      pcl::transformPointCloud(*(surf_stack_[pivot_idx]), *transformed_cloud_surf_ptr, transform_i_pivot);
      pcl::PointIndices::Ptr inliers_surf(new pcl::PointIndices());

      for (int i = 0; i < size_surf_stack_[0]; ++i) {
        inliers_surf->indices.push_back(i);
      }
      extract.setInputCloud(transformed_cloud_surf_ptr);
      extract.setIndices(inliers_surf);
      extract.setNegative(true);
      extract.filter(filtered_surf_points);

      filtered_surf_points += *(surf_stack_[i]);

      *(surf_stack_[i]) = filtered_surf_points;

#ifdef USE_CORNER
      pcl::transformPointCloud(*(corner_stack_[pivot_idx]), *transformed_cloud_corner_ptr, transform_i_pivot);
      pcl::PointIndices::Ptr inliers_corner(new pcl::PointIndices());

      for (int i = 0; i < size_corner_stack_[0]; ++i) {
        inliers_corner->indices.push_back(i);
      }
      extract.setInputCloud(transformed_cloud_corner_ptr);
      extract.setIndices(inliers_corner);
      extract.setNegative(true);
      extract.filter(filtered_corner_points);

      filtered_corner_points += *(corner_stack_[i]);

      *(corner_stack_[i]) = filtered_corner_points;
#endif
    }

  }

//region fix the map
#ifdef FIX_MAP
  Ps_linearized_[cir_buf_count_] = Ps_[cir_buf_count_];
  Rs_linearized_[cir_buf_count_] = Rs_[cir_buf_count_];
  Ps_linearized_.push(Ps_linearized_[cir_buf_count_]);
  Rs_linearized_.push(Rs_linearized_[cir_buf_count_]);
#endif
//endregion

  dt_buf_.push(vector<double>());
  linear_acceleration_buf_.push(vector<Vector3d>());
  angular_velocity_buf_.push(vector<Vector3d>());

//  Headers_.push(Headers_[cir_buf_count_]);
  Ps_.push(Ps_[cir_buf_count_]);
  Vs_.push(Vs_[cir_buf_count_]);
  Rs_.push(Rs_[cir_buf_count_]);
  Bas_.push(Bas_[cir_buf_count_]);
  Bgs_.push(Bgs_[cir_buf_count_]);

//  pre_integrations_.push(std::make_shared<IntegrationBase>(IntegrationBase(acc_last_, gyr_last_,
//                                                                           Bas_[cir_buf_count_],
//                                                                           Bgs_[cir_buf_count_],
//                                                                           estimator_config_.pim_config)));

//  all_laser_transforms_.push(all_laser_transforms_[cir_buf_count_]);

// TODO: slide new lidar points

}

void Estimator::ProcessEstimation() {

  while (true) {
    PairMeasurements measurements;
    std::unique_lock<std::mutex> buf_lk(buf_mutex_);
    con_.wait(buf_lk, [&] {
      return (measurements = GetMeasurements()).size() != 0;
    });
    buf_lk.unlock();

//    DLOG(INFO) << "measurement obtained: " << measurements.size() << ", first imu data size: "
//              << measurements.front().first.size();

    thread_mutex_.lock();
    for (auto &measurement : measurements) {
      ROS_DEBUG_STREAM("measurements ratio: 1:" << measurement.first.size());
      CompactDataConstPtr compact_data_msg = measurement.second;
      double ax = 0, ay = 0, az = 0, rx = 0, ry = 0, rz = 0;
      TicToc tic_toc_imu;
      tic_toc_imu.Tic();
      for (auto &imu_msg : measurement.first) {
        double imu_time = imu_msg->header.stamp.toSec();
        double laser_odom_time = compact_data_msg->header.stamp.toSec() + mm_config_.msg_time_delay;
        if (imu_time <= laser_odom_time) {

          if (curr_time_ < 0) {
            curr_time_ = imu_time;
          }

          double dt = imu_time - curr_time_;
          ROS_ASSERT(dt >= 0);
          curr_time_ = imu_time;
          ax = imu_msg->linear_acceleration.x;
          ay = imu_msg->linear_acceleration.y;
          az = imu_msg->linear_acceleration.z;
          rx = imu_msg->angular_velocity.x;
          ry = imu_msg->angular_velocity.y;
          rz = imu_msg->angular_velocity.z;
          ProcessImu(dt, Vector3d(ax, ay, az), Vector3d(rx, ry, rz), imu_msg->header);

        } else {

          // NOTE: interpolate imu measurement
          double dt_1 = laser_odom_time - curr_time_;
          double dt_2 = imu_time - laser_odom_time;
          curr_time_ = laser_odom_time;
          ROS_ASSERT(dt_1 >= 0);
          ROS_ASSERT(dt_2 >= 0);
          ROS_ASSERT(dt_1 + dt_2 > 0);
          double w1 = dt_2 / (dt_1 + dt_2);
          double w2 = dt_1 / (dt_1 + dt_2);
          ax = w1 * ax + w2 * imu_msg->linear_acceleration.x;
          ay = w1 * ay + w2 * imu_msg->linear_acceleration.y;
          az = w1 * az + w2 * imu_msg->linear_acceleration.z;
          rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
          ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
          rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
          ProcessImu(dt_1, Vector3d(ax, ay, az), Vector3d(rx, ry, rz), imu_msg->header);

        }
      }

      DLOG(INFO) << "per imu time: " << tic_toc_imu.Toc() / measurement.first.size() << " ms";

      ROS_DEBUG("processing laser data with stamp %f \n", compact_data_msg->header.stamp.toSec());

      TicToc t_s;

      this->ProcessCompactData(compact_data_msg, compact_data_msg->header);

//      const auto &pos_from_msg = compact_data_msg->pose.pose.position;
//      const auto &quat_from_msg = compact_data_msg->pose.pose.orientation;
//      transform_to_init_.pos.x() = pos_from_msg.x;
//      transform_to_init_.pos.y() = pos_from_msg.y;
//      transform_to_init_.pos.z() = pos_from_msg.z;
//
//      transform_to_init_.rot.w() = quat_from_msg.w;
//      transform_to_init_.rot.x() = quat_from_msg.x;
//      transform_to_init_.rot.y() = quat_from_msg.y;
//      transform_to_init_.rot.z() = quat_from_msg.z;

      // TODO: move all the processes into node?

//      DLOG(INFO) << transform_to_init_;
//      ProcessLaserOdom(transform_to_init_, compact_data_msg->header);

      double whole_t = t_s.Toc();
//      PrintStatistics(this, whole_t);
//      std_msgs::Header header = compact_data_msg->header;
//      header.frame_id = "world";

      // Pub results

//      PubOdometry(estimator, header);
//      PubKeyPoses(estimator, header);
//      PubCameraPose(estimator, header);
//      PubPointCloud(estimator, header);
//      PubTF(estimator, header);
//      pubKeyframe(estimator);
//      if (relo_msg != NULL)
//        PubRelocalization(estimator);
    }
    thread_mutex_.unlock();
    buf_mutex_.lock();
    state_mutex_.lock();

//    if (after initialization) {
//      // update the predicted
//      Update();
//    }

    state_mutex_.unlock();
    buf_mutex_.unlock();
  }

}

} // namespace lio
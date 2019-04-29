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
// Created by hyye on 5/12/18.
//
#include <gtest/gtest.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "imu_processor/Estimator.h"
#include "utils/TicToc.h"

#include "utils/LoadVirtual.h"

using namespace lio;
using namespace lio_test;
using namespace std;
using namespace mathutils;

#define WINDOW_SIZE 10
#define INTERVAL 20

DEFINE_string(filename, "/home/hyye/dev_ws/src/lio/test/data/imu_pose_vel.txt", "filename to load");
DEFINE_bool(use_opt, false, "whether to use opt");
DEFINE_bool(use_marg, false, "marginalization");
//DEFINE_string(filename, "/home/hyye/dev_ws/src/lio/test/data/imu_pose_vel_noise.txt", "filename to load");

MarginalizationInfo *last_marginalization_info;
vector<double *> last_marginalization_parameter_blocks;

CircularBuffer<Vector3d> Ps{WINDOW_SIZE + 1};
//CircularBuffer<Quaterniond> Qs{WINDOW_SIZE + 1};
CircularBuffer<Matrix3d> Rs{WINDOW_SIZE + 1};
CircularBuffer<Vector3d> Vs{WINDOW_SIZE + 1};
CircularBuffer<Vector3d> Bas{WINDOW_SIZE + 1};
CircularBuffer<Vector3d> Bgs{WINDOW_SIZE + 1};

CircularBuffer<Vector3d> Ps_gt{WINDOW_SIZE + 1};
//CircularBuffer<Matrix3d> Rs_gt{WINDOW_SIZE + 1};
CircularBuffer<Quaterniond> Qs_gt{WINDOW_SIZE + 1};
CircularBuffer<Vector3d> Vs_gt{WINDOW_SIZE + 1};
CircularBuffer<Vector3d> Bas_gt{WINDOW_SIZE + 1};
CircularBuffer<Vector3d> Bgs_gt{WINDOW_SIZE + 1};

Vector3d tmp_Ps{0, 0, 0};
Vector3d tmp_Vs{0, 0, 0};
//Quaterniond tmp_Qs;
Matrix3d tmp_Rs;
Vector3d tmp_Bas{0, 0, 0};
Vector3d tmp_Bgs{0, 0, 0};

double para_pose[WINDOW_SIZE + 1][7];
double para_speed_bias[WINDOW_SIZE + 1][9];

double para_pose_gt[WINDOW_SIZE + 1][7];
double para_speed_bias_gt[WINDOW_SIZE + 1][9];

void Vector2Double() {
  for (int i = 0; i <= WINDOW_SIZE; i++) {
    para_pose[i][0] = Ps[i].x();
    para_pose[i][1] = Ps[i].y();
    para_pose[i][2] = Ps[i].z();
    Eigen::Quaterniond q{Rs[i]};
//    Eigen::Quaterniond q = Qs[i];
    para_pose[i][3] = q.x();
    para_pose[i][4] = q.y();
    para_pose[i][5] = q.z();
    para_pose[i][6] = q.w();

    para_speed_bias[i][0] = Vs[i].x();
    para_speed_bias[i][1] = Vs[i].y();
    para_speed_bias[i][2] = Vs[i].z();

    para_speed_bias[i][3] = Bas[i].x();
    para_speed_bias[i][4] = Bas[i].y();
    para_speed_bias[i][5] = Bas[i].z();

    para_speed_bias[i][6] = Bgs[i].x();
    para_speed_bias[i][7] = Bgs[i].y();
    para_speed_bias[i][8] = Bgs[i].z();
  }
}

void Vector2DoubleGt() {
  for (int i = 0; i <= WINDOW_SIZE; i++) {
    para_pose_gt[i][0] = Ps_gt[i].x();
    para_pose_gt[i][1] = Ps_gt[i].y();
    para_pose_gt[i][2] = Ps_gt[i].z();
//    Eigen::Quaterniond q{Rs_gt[i]};
    Eigen::Quaterniond q = Qs_gt[i];
    para_pose_gt[i][3] = q.x();
    para_pose_gt[i][4] = q.y();
    para_pose_gt[i][5] = q.z();
    para_pose_gt[i][6] = q.w();

    para_speed_bias_gt[i][0] = Vs_gt[i].x();
    para_speed_bias_gt[i][1] = Vs_gt[i].y();
    para_speed_bias_gt[i][2] = Vs_gt[i].z();

    para_speed_bias_gt[i][3] = Bas_gt[i].x();
    para_speed_bias_gt[i][4] = Bas_gt[i].y();
    para_speed_bias_gt[i][5] = Bas_gt[i].z();

    para_speed_bias_gt[i][6] = Bgs_gt[i].x();
    para_speed_bias_gt[i][7] = Bgs_gt[i].y();
    para_speed_bias_gt[i][8] = Bgs_gt[i].z();
  }
}

void Double2Vector() {
  Vector3d origin_R0 = R2ypr(Rs[0]);
  Vector3d origin_P0 = Ps[0];

  Vector3d origin_R00 = R2ypr(Quaterniond(para_pose[0][6],
                                          para_pose[0][3],
                                          para_pose[0][4],
                                          para_pose[0][5]).toRotationMatrix());
  double y_diff = origin_R0.x() - origin_R00.x();
  //TODO
  Matrix3d rot_diff = ypr2R(Vector3d(y_diff, 0, 0));
  if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0) {
    ROS_DEBUG("euler singular point!");
    rot_diff = Rs[0] * Quaterniond(para_pose[0][6],
                                   para_pose[0][3],
                                   para_pose[0][4],
                                   para_pose[0][5]).toRotationMatrix().transpose();
  }

  for (int i = 0; i <= WINDOW_SIZE; i++) {
//    Ps[i].x() = para_pose[i][0];
//    Ps[i].y() = para_pose[i][1];
//    Ps[i].z() = para_pose[i][2];
    Rs[i] = rot_diff * Quaterniond(para_pose[i][6],
                                   para_pose[i][3],
                                   para_pose[i][4],
                                   para_pose[i][5]).normalized().toRotationMatrix();
    Ps[i] = rot_diff * Vector3d(para_pose[i][0] - para_pose[0][0],
                                para_pose[i][1] - para_pose[0][1],
                                para_pose[i][2] - para_pose[0][2]) + origin_P0;
//    Eigen::Quaterniond q;
//    q.x() = para_pose[i][3];
//    q.y() = para_pose[i][4];
//    q.z() = para_pose[i][5];
//    q.w() = para_pose[i][6];
////    Rs[i] = q.toRotationMatrix();
//    Qs[i] = q;

//    Vs[i].x() = para_speed_bias[i][0];
//    Vs[i].y() = para_speed_bias[i][1];
//    Vs[i].z() = para_speed_bias[i][2];

    Vs[i] = rot_diff * Vector3d(para_speed_bias[i][0],
                                para_speed_bias[i][1],
                                para_speed_bias[i][2]);

//    Bas[i].x() = para_speed_bias[i][3];
//    Bas[i].y() = para_speed_bias[i][4];
//    Bas[i].z() = para_speed_bias[i][5];
//
//    Bgs[i].x() = para_speed_bias[i][6];
//    Bgs[i].y() = para_speed_bias[i][7];
//    Bgs[i].z() = para_speed_bias[i][8];
    Bas[i] = Vector3d(para_speed_bias[i][3],
                      para_speed_bias[i][4],
                      para_speed_bias[i][5]);

    Bgs[i] = Vector3d(para_speed_bias[i][6],
                      para_speed_bias[i][7],
                      para_speed_bias[i][8]);
  }
}

TEST(ImuFactorTest, pimtest) {

  std::ofstream save_points;
  save_points.open("/tmp/imu_test.txt");
  save_points << "t qw0 qx0 qy0 qz0 x0 y0 z0 vx0 vy0 vz0 qw1 qx1 qy1 qz1 x1 y1 z1 vx1 vy1 vz1" << std::endl;

  Vector3d acc_last, gyr_last;
  vector<MotionData> imu_data;
  LoadPoseVel(FLAGS_filename, imu_data);

  CircularBuffer<shared_ptr<IntegrationBase>> pre_integrations{WINDOW_SIZE + 1};

  Eigen::Vector3d g_vec{0, 0, -9.81};

  int cir_buf_count = -1;
  double curr_time = 0;

  tmp_Ps = imu_data[0].twb;
//  tmp_Qs = imu_data[0].Rwb;
  tmp_Rs = imu_data[0].Rwb;
  tmp_Vs = imu_data[0].imu_velocity;

  tmp_Bas = imu_data[0].imu_acc_bias;
  tmp_Bgs = imu_data[0].imu_gyro_bias;

  acc_last = imu_data[0].imu_acc;
  gyr_last = imu_data[0].imu_gyro;

  IntegrationBaseConfig pim_config;
  pim_config.g_norm = -g_vec.z();
  shared_ptr<IntegrationBase> tmp_pre_integrations = std::make_shared<IntegrationBase>(IntegrationBase(acc_last,
                                                                                                       gyr_last,
                                                                                                       tmp_Bas,
                                                                                                       tmp_Bgs,
                                                                                                       pim_config));

  for (int data_idx = 0; data_idx < imu_data.size(); ++data_idx) {
    auto &data = imu_data[data_idx];

    Eigen::Vector3d acc = data.imu_acc;
    Eigen::Vector3d gyro = data.imu_gyro;
    double imu_time = data.timestamp;

//    tmp_Bas = data.imu_acc_bias;
//    tmp_Bgs = data.imu_gyro_bias;

//    DLOG(INFO) << "data.twb bef: " << data.twb.transpose();
//    DLOG(INFO) << "tmp_Ps bef: " << tmp_Ps.transpose();

    if (data_idx == 0) {
      acc_last = acc;
      gyr_last = gyro;
      curr_time = imu_time;
    }

    double dt = imu_time - curr_time;
    curr_time = imu_time;

    tmp_pre_integrations->push_back(dt, acc, gyro);

//    DLOG(INFO) << pre_integrations.last()->delta_p_.transpose();
//    DLOG(INFO) << tmp_pre_inPtegrations->delta_p_.transpose();

    Vector3d un_acc_0 = tmp_Rs * (acc_last - tmp_Bas) + g_vec;
    Vector3d un_gyr = 0.5 * (gyr_last + gyro) - tmp_Bgs;
    tmp_Rs *= DeltaQ(un_gyr * dt).toRotationMatrix();
    Vector3d un_acc_1 = tmp_Rs * (acc - tmp_Bas) + g_vec;
    Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    tmp_Ps += dt * tmp_Vs + 0.5 * dt * dt * un_acc;
    tmp_Vs += dt * un_acc;

//    tmp_Qs.normalize();


//    Vector3d omg = gyr_last- tmp_Bgs;
//    omg = omg * dt / 2;
//    Quaterniond dR(1, omg(0), omg(1), omg(2));
//    Vector3d un_acc = tmp_Rs * (acc_last - tmp_Bas) + g_vec;
//    tmp_Rs = (tmp_Rs * dR).eval();
//    tmp_Ps += (dt * tmp_Vs + 0.5 * dt * dt * un_acc).eval();
//    tmp_Vs += (dt * un_acc).eval();


    acc_last = acc;
    gyr_last = gyro;

    if (data_idx % INTERVAL == 0) {
      Ps_gt.push(data.twb);
      Qs_gt.push(Eigen::Quaterniond(data.Rwb));
      Vs_gt.push(data.imu_velocity);
      Bas_gt.push(data.imu_acc_bias);
      Bgs_gt.push(data.imu_gyro_bias);

      Ps.push(tmp_Ps);
//      Qs.push(tmp_Qs);
      Rs.push(tmp_Rs);
      Vs.push(tmp_Vs);
      Bas.push(tmp_Bas);
      Bgs.push(tmp_Bgs);

      if (cir_buf_count < WINDOW_SIZE) {
        ++cir_buf_count;
      }

      pre_integrations.push(tmp_pre_integrations);

      tmp_pre_integrations.reset();
      tmp_pre_integrations = std::make_shared<IntegrationBase>(IntegrationBase(acc_last,
                                                                               gyr_last,
                                                                               Bas[cir_buf_count],
                                                                               Bgs[cir_buf_count],
                                                                               pim_config));

      DLOG(INFO) << "data_idx: " << data_idx;
      DLOG(INFO) << "Ps: " << tmp_Ps.transpose() << endl;
      DLOG(INFO) << "data.twb: " << data.twb.transpose() << endl;
      DLOG(INFO) << "Vs: " << tmp_Vs.transpose() << endl;
      DLOG(INFO) << "data.imu_velocity: " << data.imu_velocity.transpose() << endl;
      DLOG(INFO) << "^^^^^^^ TEST ^^^^^^^";

      Ps.last() = data.twb;
//      Qs.last() = data.Rwb;
      Rs.last() = data.Rwb;
    }

    if (cir_buf_count < WINDOW_SIZE) {
      continue;
    } else if (data_idx % INTERVAL == 0) {
      Ps.last() = data.twb;
//      Qs.last() = data.Rwb;
      Rs.last() = data.Rwb;

      DLOG(INFO) << "data.twb: " << data.twb.transpose();
      DLOG(INFO) << "tmp_Ps bef: " << tmp_Ps.transpose();
      DLOG(INFO) << "data.imu_velocity: " << data.imu_velocity.transpose();
      DLOG(INFO) << "tmp_Vs bef: " << tmp_Vs.transpose();

      Vector2Double();
      Vector2DoubleGt();


//      {
//        for (int i = 0; i < WINDOW_SIZE; ++i) {
//          int j = i + 1;
//          double *res = new double[15];
//          double **jaco = new double *[4];
//          jaco[0] = new double[15 * 7];
//          jaco[1] = new double[15 * 9];
//          jaco[2] = new double[15 * 7];
//          jaco[3] = new double[15 * 9];
//
//          double **tmp_parameters = new double *[4];
//          tmp_parameters[0] = para_pose[i];
//          tmp_parameters[1] = para_speed_bias[i];
//          tmp_parameters[2] = para_pose[j];
//          tmp_parameters[3] = para_speed_bias[j];
//
//          ImuFactor *f = new ImuFactor(pre_integrations[j], g_vec);
//          f->Evaluate(tmp_parameters, res, jaco);
//
//          DLOG(INFO) << "check begins: " << i;
//          DLOG(INFO) << "analytical";
//          DLOG(INFO) << "pre: " << (pre_integrations[j]->Evaluate(Ps[i], Qs[i], Vs[i], Bas[i], Bgs[i],
//                                                               Ps[j], Qs[j], Vs[j], Bas[j], Bgs[j], g_vec)).transpose();
//          DLOG(INFO) << "pre_gt: " << (pre_integrations[j]->Evaluate(Ps_gt[i], Qs_gt[i], Vs_gt[i], Bas_gt[i], Bgs_gt[i],
//                                                                  Ps_gt[j], Qs_gt[j], Vs_gt[j], Bas_gt[j], Bgs_gt[j],
//                                                                  g_vec)).transpose();
//
//          DLOG(INFO) << Eigen::Map<Eigen::Matrix<double, 15, 1>>(res).transpose();
//          DLOG(INFO) << "jaco0:" << std::endl << Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>>(jaco[0]);
//          DLOG(INFO) << "jaco1:" << std::endl << Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>>(jaco[1]);
//          DLOG(INFO) << "jaco2:" << std::endl << Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>>(jaco[2]);
//          DLOG(INFO) << "jaco3:" << std::endl << Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>>(jaco[3]);
//
////          DLOG(INFO) << "covariance: " << endl << pre_integrations[j]->covariance_;
//
//          Eigen::Matrix<double, 15, 15> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>
//              (pre_integrations[j]->covariance_.inverse()).matrixL().transpose();
//
////          DLOG(INFO) << "sqrt_info: " << endl << sqrt_info;
//
//          delete f;
//          delete[] res;
//          delete[] jaco[0];
//          delete[] jaco[1];
//          delete[] jaco[2];
//          delete[] jaco[3];
//          delete[] jaco;
//          delete[] tmp_parameters;
//        }
//      }

      {
        ceres::Problem problem, problem_gt;
        ceres::Solver::Options options;

        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.trust_region_strategy_type = ceres::DOGLEG;
        options.max_num_iterations = 10;

        options.max_solver_time_in_seconds = 1;

        vector<double *> para_ids, para_ids_gt;
        vector<ceres::internal::ResidualBlock *> res_ids_pim, res_ids_pim_gt, res_marg;

        //region Add pose and speed bias parameters
        for (int i = 0; i < WINDOW_SIZE + 1; ++i) {
          ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
          problem.AddParameterBlock(para_pose[i], SIZE_POSE, local_parameterization);
          problem.AddParameterBlock(para_speed_bias[i], SIZE_SPEED_BIAS);
          para_ids.push_back(para_pose[i]);
          para_ids.push_back(para_speed_bias[i]);
        }

        for (int i = 0; i < WINDOW_SIZE; ++i) {
          int j = i + 1;
          ImuFactor *f = new ImuFactor(pre_integrations[j]);
          ceres::internal::ResidualBlock *res_id =
              problem.AddResidualBlock(f,
                                       NULL,
                                       para_pose[i],
                                       para_speed_bias[i],
                                       para_pose[j],
                                       para_speed_bias[j]
              );

          res_ids_pim.push_back(res_id);

          DLOG(INFO) << "pre" << i << ": " << (pre_integrations[j]->Evaluate(Ps[i],
                                                                            Eigen::Quaterniond(Rs[i]),
                                                                            Vs[i],
                                                                            Bas[i],
                                                                            Bgs[i],
                                                                            Ps[j],
                                                                            Eigen::Quaterniond(Rs[j]),
                                                                            Vs[j],
                                                                            Bas[j],
                                                                            Bgs[j])).squaredNorm();

          DLOG(INFO) << "pre_gt" << i << ": " << (pre_integrations[j]->Evaluate(Ps_gt[i],
                                                                               Qs_gt[i],
                                                                               Vs_gt[i],
                                                                               Bas_gt[i],
                                                                               Bgs_gt[i],
                                                                               Ps_gt[j],
                                                                               Qs_gt[j],
                                                                               Vs_gt[j],
                                                                               Bas_gt[j],
                                                                               Bgs_gt[j])).squaredNorm();

//          DLOG(INFO) << "covariance: " << endl << pre_integrations[j]->covariance_;
//
//          Eigen::Matrix<double, 15, 15> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>
//              (pre_integrations[j]->covariance_.inverse()).matrixL().transpose();
//
//          DLOG(INFO) << "sqrt_info: " << endl << sqrt_info;

//          DLOG(INFO) << Ps[i].transpose();
//          DLOG(INFO) << Ps_gt[i].transpose();

//          DLOG(INFO) << "covariance: " << endl << pre_integrations[j]->covariance_;
//
//          Eigen::Matrix<double, 15, 15> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>
//              (pre_integrations[j]->covariance_.inverse()).matrixL().transpose();
//
//          DLOG(INFO) << "sqrt_info: " << endl << sqrt_info;
        }
        //endregion

        //region Add pose and speed bias parameters
        for (int i = 0; i < WINDOW_SIZE + 1; ++i) {
          ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
          problem_gt.AddParameterBlock(para_pose_gt[i], SIZE_POSE, local_parameterization);
          problem_gt.AddParameterBlock(para_speed_bias_gt[i], SIZE_SPEED_BIAS);
          para_ids_gt.push_back(para_pose_gt[i]);
          para_ids_gt.push_back(para_speed_bias_gt[i]);
        }

        for (int i = 0; i < WINDOW_SIZE; ++i) {
          int j = i + 1;
          ImuFactor *f = new ImuFactor(pre_integrations[j]);
          ceres::internal::ResidualBlock *res_id =
              problem_gt.AddResidualBlock(f,
                                          NULL,
                                          para_pose_gt[i],
                                          para_speed_bias_gt[i],
                                          para_pose_gt[j],
                                          para_speed_bias_gt[j]
              );

          res_ids_pim_gt.push_back(res_id);
        }
        //endregion

        //region Marginalization residual
        if (FLAGS_use_marg) {
          if (last_marginalization_info) {
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ceres::internal::ResidualBlock *res_id =
                problem.AddResidualBlock(marginalization_factor, NULL,
                                         last_marginalization_parameter_blocks);

            res_marg.push_back(res_id);
          }
        }
        //endregion

        //region residual before optimization
        {
          ///< Bef
          double cost = 0.0;
          ceres::Problem::EvaluateOptions e_option;
          e_option.parameter_blocks = para_ids;
          e_option.residual_blocks = res_ids_pim;
          problem.Evaluate(e_option, &cost, NULL, NULL, NULL);
          DLOG(INFO) << "bef_pim: " << cost;
          DLOG(INFO) << para_speed_bias[1][3] << " " << para_speed_bias[1][4] << " " << para_speed_bias[1][5];
//          DLOG(INFO) << para_pose[0][0] << " " << para_pose[0][1] << " " << para_pose[0][2];

          if (last_marginalization_info) {
            e_option.parameter_blocks = para_ids;
            e_option.residual_blocks = res_marg;
            problem.Evaluate(e_option, &cost, NULL, NULL, NULL);
            DLOG(INFO) << "bef_marg: " << cost;
          }

          e_option.parameter_blocks = para_ids_gt;
          e_option.residual_blocks = res_ids_pim_gt;
          problem_gt.Evaluate(e_option, &cost, NULL, NULL, NULL);
          DLOG(INFO) << "bef_pim_gt: " << cost;
        }
        //endregion
        if (FLAGS_use_opt) {
          ceres::Solver::Summary summary;
          ceres::Solve(options, &problem, &summary);
          DLOG(INFO) << summary.BriefReport();
          DLOG(INFO) << "use_opt";
        } else {
          DLOG(INFO) << "not use_opt";
        }

        //region residual after optimization
        {
          ///< Aft
          double cost = 0.0;
          ceres::Problem::EvaluateOptions e_option;
          e_option.parameter_blocks = para_ids;
          e_option.residual_blocks = res_ids_pim;
          problem.Evaluate(e_option, &cost, NULL, NULL, NULL);
          DLOG(INFO) << "aft_pim: " << cost;
          DLOG(INFO) << para_speed_bias[1][3] << " " << para_speed_bias[1][4] << " " << para_speed_bias[1][5];
          DLOG(INFO) << Bas[1].transpose();
//          DLOG(INFO) << para_pose[0][0] << " " << para_pose[0][1] << " " << para_pose[0][2];

          if (last_marginalization_info) {
            e_option.parameter_blocks = para_ids;
            e_option.residual_blocks = res_marg;
            problem.Evaluate(e_option, &cost, NULL, NULL, NULL);
            DLOG(INFO) << "aft_marg: " << cost;
          }
        }
        //endregion
      }

      {
        Double2Vector();
        //region Marginalization
        {
          if (FLAGS_use_marg) {

            TicToc t_whole_marginalization;

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            Vector2Double();

            if (last_marginalization_info) {
              vector<int> drop_set;
              for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++) {
                if (last_marginalization_parameter_blocks[i] == para_pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_speed_bias[0])
                  drop_set.push_back(i);
              }
              // construct new marginlization_factor
              MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
              ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                             last_marginalization_parameter_blocks,
                                                                             drop_set);

              marginalization_info->AddResidualBlockInfo(residual_block_info);
            }

            if (pre_integrations[1]->sum_dt_ < 10.0) {
              ImuFactor *imu_factor = new ImuFactor(pre_integrations[1]);
              ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                             vector<double *>{para_pose[0],
                                                                                              para_speed_bias[0],
                                                                                              para_pose[1],
                                                                                              para_speed_bias[1]},
                                                                             vector<int>{0, 1});
              marginalization_info->AddResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            marginalization_info->PreMarginalize();
            ROS_DEBUG("pre marginalization %f ms", t_pre_margin.Toc());

            TicToc t_margin;
            marginalization_info->Marginalize();
            ROS_DEBUG("marginalization %f ms", t_margin.Toc());

            std::unordered_map<long, double *> addr_shift;
            for (int i = 1; i < WINDOW_SIZE + 1; ++i) {
              addr_shift[reinterpret_cast<long>(para_pose[i])] = para_pose[i - 1];
              addr_shift[reinterpret_cast<long>(para_speed_bias[i])] = para_speed_bias[i - 1];
            }

            vector<double *> parameter_blocks = marginalization_info->GetParameterBlocks(addr_shift);

            if (last_marginalization_info) {
              delete last_marginalization_info;
            }
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;

            DLOG(INFO) << "whole marginalization costs: " << t_whole_marginalization.Toc();
          }
        }
        //endregion
        tmp_Ps = Ps.last();
        tmp_Rs = Rs.last();
//        tmp_Qs = Qs.last();
        tmp_Vs = Vs.last();
        tmp_Bas = Bas.last();
        tmp_Bgs = Bgs.last();
      }
//      DLOG(INFO) << "data.twb: " << data.twb.transpose();
//      DLOG(INFO) << "tmp_Ps: " << tmp_Ps.transpose();
//      DLOG(INFO) << "data.imu_velocity: " << data.imu_velocity.transpose();
//      DLOG(INFO) << "tmp_Vs: " << tmp_Vs.transpose();
      DLOG(INFO) << "======= TEST =======";

      Eigen::Quaterniond tmp_q(tmp_Rs);
//      Eigen::Quaterniond tmp_q = tmp_Qs;
//      Eigen::Quaterniond tmp_q_gt(Rs_gt.last());
      Eigen::Quaterniond tmp_q_gt = Qs_gt.last();
      Eigen::Vector3d tmp_p_gt = Ps_gt.last();
      Eigen::Vector3d tmp_v_gt = Vs_gt.last();
      save_points << curr_time << " "
                  << tmp_q_gt.w() << " "
                  << tmp_q_gt.x() << " "
                  << tmp_q_gt.y() << " "
                  << tmp_q_gt.z() << " "
                  << tmp_p_gt(0) << " "
                  << tmp_p_gt(1) << " "
                  << tmp_p_gt(2) << " "
                  << tmp_v_gt(0) << " "
                  << tmp_v_gt(1) << " "
                  << tmp_v_gt(2) << " "
                  << tmp_q.w() << " "
                  << tmp_q.x() << " "
                  << tmp_q.y() << " "
                  << tmp_q.z() << " "
                  << tmp_Ps(0) << " "
                  << tmp_Ps(1) << " "
                  << tmp_Ps(2) << " "
                  << tmp_Vs(0) << " "
                  << tmp_Vs(1) << " "
                  << tmp_Vs(2) << " "
                  << std::endl;

      DLOG(INFO) << "data_idx: " << data_idx;
      DLOG(INFO) << "Ps: " << tmp_Ps.transpose() << endl;
      DLOG(INFO) << "data.twb: " << data.twb.transpose() << endl;
      DLOG(INFO) << "Vs: " << tmp_Vs.transpose() << endl;
      DLOG(INFO) << "data.imu_velocity: " << data.imu_velocity.transpose() << endl;
      DLOG(INFO) << "^^^^^^^ TEST ^^^^^^^";

    }

  }

}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);

  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  FLAGS_alsologtostderr = true;

  return RUN_ALL_TESTS();
}

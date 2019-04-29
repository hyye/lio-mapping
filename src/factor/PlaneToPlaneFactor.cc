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

#include "factor/PlaneToPlaneFactor.h"

namespace lio {

double PlaneToPlaneFactor::sum_t_;

PlaneToPlaneFactor::PlaneToPlaneFactor(const Eigen::Vector3d &pi_local, const Eigen::Vector3d &ni_local,
                                       const Eigen::Vector3d &pj_local, const Eigen::Vector3d &nj_local)
    : pi_local_{pi_local}, ni_local_{ni_local}, pj_local_{pj_local}, nj_local_{nj_local} {
  pfi_ = PointNormalFeature(pi_local_, ni_local_);
  pfj_ = PointNormalFeature(pj_local_, nj_local_);
}

bool PlaneToPlaneFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
  TicToc tic_toc;
  Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
  Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

  Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
  Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

  Eigen::Vector3d tlb{parameters[2][0], parameters[2][1], parameters[2][2]};
  Eigen::Quaterniond qlb(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

  Eigen::Map<Eigen::Vector3d> residual(residuals);

  Eigen::Quaterniond Qli = Qi * qlb.conjugate();
  Eigen::Vector3d Pli = Pi - Qli * tlb;
  Eigen::Quaterniond Qlj = Qj * qlb.conjugate();
  Eigen::Vector3d Plj = Pj - Qlj * tlb;

  Twist<double> transform_li{Qli, Pli};
  Twist<double> transform_lj{Qlj, Plj};
  Twist<double> transform_lb{qlb, tlb};

  Twist<double> transform_b_a = transform_li.inverse() * transform_lj;

  Eigen::Vector3d p_b = pfi_.point3d;
  Eigen::Vector3d p_a = pfj_.point3d;

  Eigen::Matrix3d M; /// mahalanobis matrix

  Eigen::Matrix3d R = transform_b_a.rot.toRotationMatrix();

  Eigen::Matrix3d C_b = pfi_.covariance;
  Eigen::Matrix3d C_a = pfj_.covariance;

  M = R * C_a;
  // temp = M*R' + C_b = R*C_a*R' + C_b
  Eigen::Matrix3d temp = M * R.transpose();
  temp += C_b;
  // M = temp^-1
  M = temp.inverse();

//  DLOG(INFO) << "M" << std::endl << M;

//  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(M);
//  Eigen::VectorXd S = Eigen::VectorXd((saes2.eigenvalues().array() > eps_).select(saes2.eigenvalues().array(), 0));
//  Eigen::VectorXd S_inv = Eigen::VectorXd((saes2.eigenvalues().array() > eps_).select(saes2.eigenvalues().array().inverse(), 0));
//
//  Eigen::VectorXd S_sqrt = S.cwiseSqrt();
//  S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();

  residual = transform_b_a.rot * p_a + transform_b_a.pos - p_b;

  Eigen::Matrix3d sqrt_info = Eigen::LLT<Eigen::Matrix3d>(M).matrixL().transpose();
//  sqrt_info *= (500);

//  sqrt_info *= 100;

//  DLOG(INFO) << "sqrt_info" << std::endl << sqrt_info;

  residual = sqrt_info * residual;

  if (jacobians) {
    Eigen::Matrix3d Ri = Qi.toRotationMatrix();
    Eigen::Matrix3d Rj = Qj.toRotationMatrix();
    Eigen::Matrix3d rlb = qlb.toRotationMatrix();

    if (jacobians[0]) {
      Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor> > jacobian_pose_i(jacobians[0]);
      Eigen::Matrix<double, 3, 6> jaco_i;

      jaco_i.setZero();
      jaco_i.leftCols<3>() = -rlb * Ri.transpose();
      jaco_i.rightCols<3>() = rlb * SkewSymmetric(Ri.transpose() * Rj * rlb.transpose() * p_a)
          + rlb * SkewSymmetric(Ri.transpose() * (Pj - Pi - Rj * rlb.transpose() * tlb));

      jacobian_pose_i.setZero();
      jacobian_pose_i.leftCols<6>() = sqrt_info * jaco_i;
      jacobian_pose_i.rightCols<1>().setZero();
    }

    if (jacobians[1]) {
      Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor> > jacobian_pose_j(jacobians[1]);
      Eigen::Matrix<double, 3, 6> jaco_j;

      jaco_j.setZero();
      jaco_j.leftCols<3>() = rlb * Ri.transpose();
      jaco_j.rightCols<3>() = -rlb * Ri.transpose() * Rj * SkewSymmetric(rlb.transpose() * p_a)
          + rlb * Ri.transpose() * Rj * SkewSymmetric(rlb.transpose() * tlb);

      jacobian_pose_j.setZero();
      jacobian_pose_j.leftCols<6>() = sqrt_info * jaco_j;
      jacobian_pose_j.rightCols<1>().setZero();

    }

    if (jacobians[2]) {
      Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor> > jacobian_pose_ex(jacobians[2]);
      Eigen::Matrix<double, 3, 6> jaco_ex;

      jaco_ex.setZero();

      jaco_ex.leftCols<3>() = -rlb * Ri.transpose() * Rj * rlb.transpose() + Eigen::Matrix3d::Identity();
      Eigen::Vector3d tmp_t = p_a - tlb;
      jaco_ex.rightCols<3>() = rlb * Ri.transpose() * Rj * SkewSymmetric(rlb.transpose() * tmp_t)
          - rlb * SkewSymmetric(Ri.transpose() * Rj * rlb.transpose() * tmp_t) - rlb * SkewSymmetric(Ri.transpose() * (Pj - Pi - tlb));

      jacobian_pose_ex.setZero();
      jacobian_pose_ex.leftCols<6>() = sqrt_info * jaco_ex;
      jacobian_pose_ex.rightCols<1>().setZero();
    }

  }

  sum_t_ += tic_toc.Toc();

  return true;

}

void PlaneToPlaneFactor::Check(double **parameters) {
  double *res = new double[3];
  double **jaco = new double *[3];
  jaco[0] = new double[3 * 7];
  jaco[1] = new double[3 * 7];
  jaco[2] = new double[3 * 7];
  Evaluate(parameters, res, jaco);
  DLOG(INFO) << "check begins";
  DLOG(INFO) << "analytical";

  DLOG(INFO) << Eigen::Map<Eigen::Matrix<double, 3, 1>>(res).transpose();
  DLOG(INFO) << std::endl << Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>>(jaco[0]);
  DLOG(INFO) << std::endl << Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>>(jaco[1]);
  DLOG(INFO) << std::endl << Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>>(jaco[2]);

  Eigen::Vector3d Pi{parameters[0][0], parameters[0][1], parameters[0][2]};
  Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

  Eigen::Vector3d Pj{parameters[1][0], parameters[1][1], parameters[1][2]};
  Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

  Eigen::Vector3d tlb{parameters[2][0], parameters[2][1], parameters[2][2]};
  Eigen::Quaterniond qlb(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

  Eigen::Quaterniond Qli = Qi * qlb.conjugate();
  Eigen::Vector3d Pli = Pi - Qli * tlb;
  Eigen::Quaterniond Qlj = Qj * qlb.conjugate();
  Eigen::Vector3d Plj = Pj - Qlj * tlb;

  Twist<double> transform_li{Qli, Pli};
  Twist<double> transform_lj{Qlj, Plj};
  Twist<double> transform_lb{qlb, tlb};

  Twist<double> transform_b_a = transform_li.inverse() * transform_lj;

  Eigen::Vector3d p_b = pfi_.point3d;
  Eigen::Vector3d p_a = pfj_.point3d;

  Eigen::Matrix3d M; /// mahalanobis matrix

  Eigen::Matrix3d R = transform_b_a.rot.toRotationMatrix();

  Eigen::Matrix3d C_b = pfi_.covariance;
  Eigen::Matrix3d C_a = pfj_.covariance;

  M = R * C_a;
  // temp = M*R' + C_b = R*C_a*R' + C_b
  Eigen::Matrix3d temp = M * R.transpose();
  temp += C_b;
  // M = temp^-1
  M = temp.inverse();

  Eigen::Vector3d residual = transform_b_a.rot * p_a + transform_b_a.pos - p_b;

  Eigen::Matrix3d sqrt_info = Eigen::LLT<Eigen::Matrix3d>(M).matrixL().transpose();

  residual = sqrt_info * residual;

  DLOG(INFO) << "num";
  DLOG(INFO) << residual.transpose();

  const double eps = 1e-6;
  Eigen::Matrix<double, 3, 18> num_jacobian;
  for (int k = 0; k < 18; k++) {
    Pi = Eigen::Vector3d{parameters[0][0], parameters[0][1], parameters[0][2]};
    Qi = Eigen::Quaterniond(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Pj = Eigen::Vector3d{parameters[1][0], parameters[1][1], parameters[1][2]};
    Qj = Eigen::Quaterniond(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    tlb = Eigen::Vector3d{parameters[2][0], parameters[2][1], parameters[2][2]};
    qlb = Eigen::Quaterniond(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

    int a = k / 3, b = k % 3;
    Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

    if (a == 0)
      Pi += delta;
    else if (a == 1)
      Qi = Qi * DeltaQ(delta);
    else if (a == 2)
      Pj += delta;
    else if (a == 3)
      Qj = Qj * DeltaQ(delta);
    else if (a == 4)
      tlb += delta;
    else if (a == 5)
      qlb = qlb * DeltaQ(delta);

    Qli = Qi * qlb.conjugate();
    Pli = Pi - Qli * tlb;
    Qlj = Qj * qlb.conjugate();
    Plj = Pj - Qlj * tlb;

    transform_li = Twist<double>{Qli, Pli};
    transform_lj = Twist<double>{Qlj, Plj};
    transform_lb = Twist<double>{qlb, tlb};

    transform_b_a = transform_li.inverse() * transform_lj;

    Eigen::Vector3d tmp_residual;

    tmp_residual = sqrt_info * (transform_b_a.rot * p_a + transform_b_a.pos - p_b);

    num_jacobian.col(k) = (tmp_residual - residual) / eps;
  }
  DLOG(INFO) << std::endl << num_jacobian.block<3, 6>(0, 0);
  DLOG(INFO) << std::endl << num_jacobian.block<3, 6>(0, 6);
  DLOG(INFO) << std::endl << num_jacobian.block<3, 6>(0, 12);
}

}
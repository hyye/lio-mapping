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
// Created by hyye on 3/18/18.
//

#ifndef LIO_GEOMETRY_UTILS_H_
#define LIO_GEOMETRY_UTILS_H_

#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include "3rdparty/sophus/se3.hpp"

namespace geometryutils {

/// internal geometry classes
class Vector3Intl : public Eigen::Vector4f {
 public:
  Vector3Intl(float x, float y, float z)
      : Eigen::Vector4f(x, y, z, 0) {}

  Vector3Intl(void)
      : Eigen::Vector4f(0, 0, 0, 0) {}

  template<typename OtherDerived>
  Vector3Intl(const Eigen::MatrixBase<OtherDerived> &other)
      : Eigen::Vector4f(other) {}

  Vector3Intl(const pcl::PointXYZI &p)
      : Eigen::Vector4f(p.x, p.y, p.z, 0) {}

  template<typename OtherDerived>
  Vector3Intl &operator=(const Eigen::MatrixBase<OtherDerived> &rhs) {
    this->Eigen::Vector4f::operator=(rhs);
    return *this;
  }

  Vector3Intl &operator=(const pcl::PointXYZ &rhs) {
    x() = rhs.x;
    y() = rhs.y;
    z() = rhs.z;
    return *this;
  }

  Vector3Intl &operator=(const pcl::PointXYZI &rhs) {
    x() = rhs.x;
    y() = rhs.y;
    z() = rhs.z;
    return *this;
  }

  float x() const { return (*this)(0); }

  float y() const { return (*this)(1); }

  float z() const { return (*this)(2); }

  float &x() { return (*this)(0); }

  float &y() { return (*this)(1); }

  float &z() { return (*this)(2); }

  // easy conversion
  operator pcl::PointXYZI() {
    pcl::PointXYZI dst;
    dst.x = x();
    dst.y = y();
    dst.z = z();
    dst.intensity = 0;
    return dst;
  }
};

class QuaternionIntl : public Eigen::Quaternionf {
 public:
  QuaternionIntl(float w, float x, float y, float z)
      : Eigen::Quaternionf(w, x, y, z) {}

  QuaternionIntl(void)
      : Eigen::Quaternionf(1, 0, 0, 0) {}

  template<typename OtherDerived>
  QuaternionIntl(const Eigen::QuaternionBase<OtherDerived> &other)
      : Eigen::Quaternionf(other) {}

  QuaternionIntl(const pcl::PointXYZI &p)
      : Eigen::Quaternionf(p.intensity, p.x, p.y, p.z) {}

  template<typename OtherDerived>
  QuaternionIntl &operator=(const Eigen::QuaternionBase<OtherDerived> &rhs) {
    this->Eigen::Quaternionf::operator=(rhs);
    return *this;
  }

  QuaternionIntl &operator=(const pcl::PointXYZI &rhs) {
    w() = rhs.intensity;
    x() = rhs.x;
    y() = rhs.y;
    z() = rhs.z;
    return *this;
  }

  // easy conversion
  operator pcl::PointXYZI() {
    pcl::PointXYZI dst;
    dst.x = x();
    dst.y = y();
    dst.z = z();
    dst.intensity = w();
    return dst;
  }
};

class Angle {
 public:
  Angle()
      : radian_(0.0),
        cos_(1.0),
        sin_(0.0) {}

  Angle(float rad_value)
      : radian_(rad_value),
        cos_(std::cos(rad_value)),
        sin_(std::sin(rad_value)) {}

  Angle(const Angle &other)
      : radian_(other.radian_),
        cos_(other.cos_),
        sin_(other.sin_) {}

  void operator=(const Angle &rhs) {
    radian_ = (rhs.radian_);
    cos_ = (rhs.cos_);
    sin_ = (rhs.sin_);
  }

  void operator+=(const float &rad_value) { *this = (radian_ + rad_value); }

  void operator+=(const Angle &other) { *this = (radian_ + other.radian_); }

  void operator-=(const float &rad_value) { *this = (radian_ - rad_value); }

  void operator-=(const Angle &other) { *this = (radian_ - other.radian_); }

  Angle operator-() const {
    Angle out;
    out.radian_ = -radian_;
    out.cos_ = cos_;
    out.sin_ = -(sin_);
    return out;
  }

  float rad() const { return radian_; }

  float deg() const { return radian_ * 180 / M_PI; }

  float cos() const { return cos_; }

  float sin() const { return sin_; }

 private:
  float radian_;    ///< angle value in radian
  float cos_;       ///< cosine of the angle
  float sin_;       ///< sine of the angle
};

template<typename T>
Eigen::Matrix<T, 3, 3> RightJacobian(const Eigen::Matrix<T, 3, 1> &v) {

  T v_norm = v.norm();
  T v_norm2 = v_norm * v_norm;
  T v_norm3 = v_norm2 * v_norm;
  Eigen::Matrix<T, 3, 3> v_skew = Sophus::SO3<T>::hat(v);
  Eigen::Matrix<T, 3, 3> I3x3 = Eigen::Matrix<T, 3, 3>::Identity();

  if (v_norm < Sophus::Constants<T>::epsilon()) {
    return I3x3;
  }

  Eigen::Matrix<T, 3, 3> Jr
      = I3x3 - ((1 - cos(v_norm)) / v_norm2) * v_skew + (v_norm - sin(v_norm)) / v_norm3 * v_skew * v_skew;

  return Jr;
}

template<typename T>
Eigen::Matrix<T, 3, 3> RightJacobian(const Sophus::SO3<T> &R) {
  Eigen::Matrix<T, 3, 3> Jr = RightJacobian<T>(R.log());
  return Jr;
}

template<typename T>
Eigen::Matrix<T, 3, 3> RightJacobianInverse(const Eigen::Matrix<T, 3, 1> &v) {

  T v_norm = v.norm();
  T v_norm2 = v_norm * v_norm;
  T v_norm3 = v_norm2 * v_norm;
  Eigen::Matrix<T, 3, 3> v_skew = Sophus::SO3<T>::hat(v);
  Eigen::Matrix<T, 3, 3> I3x3 = Eigen::Matrix<T, 3, 3>::Identity();

  if (v_norm < Sophus::Constants<T>::epsilon()) {
    return I3x3;
  }

  Eigen::Matrix<T, 3, 3> Jr_inv
      = I3x3 + 0.5 * v_skew + (1 / v_norm2 - (1 + cos(v_norm)) / (2 * v_norm * sin(v_norm))) * v_skew * v_skew;
  return Jr_inv;
}

template<typename T>
Eigen::Matrix<T, 3, 3> RightJacobianInverse(const Sophus::SO3<T> &R) {
  Eigen::Matrix<T, 3, 3> Jr_inv = RightJacobianInverse<T>(R.log());
  return Jr_inv;
}

template<typename T>
Eigen::Matrix<T, 3, 3> RotationVectorJacobian(const Sophus::SO3<T> &R, const Eigen::Matrix<T, 3, 1> &a) {

  Eigen::Matrix<T, 3, 3> Jr = -R.matrix() * Sophus::SO3<T>::hat(a) * RightJacobian<T>(R);

  return Jr;
}

//template<typename T>
//Eigen::Matrix<T, 3, 3> RotationTransposeVectorJacobian(const Sophus::SO3<T> &R, const Eigen::Matrix<T, 3, 1> &a) {
//
//  typename Sophus::SO3<T>::Tangent v = R.log();
//
//  Eigen::Matrix<T, 3, 3> Jr = Sophus::SO3<T>::exp(-v).matrix() * Sophus::SO3<T>::hat(a) * RightJacobian<T>(-v);
//
//  return Jr;
//}

/// these two should be equilavent
template<typename T>
Eigen::Matrix<T, 3, 3> RotationTransposeVectorJacobian(const Sophus::SO3<T> &R, const Eigen::Matrix<T, 3, 1> &a) {

  typename Sophus::SO3<T>::Tangent v = R.log();

  Eigen::Matrix<T, 3, 3> Jr = Sophus::SO3<T>::hat(R.unit_quaternion().conjugate() * a) * RightJacobian<T>(v);

  return Jr;
}

// NOTE: the delta theta in the following functions are different from the previous ones (R(th)*R(d_th) v.s. R(th+d_th))
template<typename T>
Eigen::Matrix<T, 3, 3> RotationVectorJacobian(const Eigen::Matrix<T, 3, 3> &R, const Eigen::Matrix<T, 3, 1> &a) {

  Eigen::Matrix<T, 3, 3> Jr = -R * Sophus::SO3<T>::hat(a);

  return Jr;
}

template<typename T>
Eigen::Matrix<T, 3, 3> RotationTransposeVectorJacobian(const Eigen::Matrix<T, 3, 3> &R,
                                                       const Eigen::Matrix<T, 3, 1> &a) {

  Eigen::Matrix<T, 3, 3> Jr = Sophus::SO3<T>::hat(R.transpose() * a);

  return Jr;
}

template<typename PointT>
inline void RotatePoint(const Eigen::Quaternionf &q, PointT &p) {
  Eigen::Vector3f vec, vec_out;
  vec.x() = p.x;
  vec.y() = p.y;
  vec.z() = p.z;
  vec_out = q * vec;
  p.x = vec_out.x();
  p.y = vec_out.y();
  p.z = vec_out.z();
}

template<typename PointT>
inline void TranslatePoint(const Eigen::Vector3f &t, PointT &p) {
  Eigen::Vector3f vec, vec_out;
  vec.x() = p.x;
  vec.y() = p.y;
  vec.z() = p.z;
  vec_out = vec + t;
  p.x = vec_out.x();
  p.y = vec_out.y();
  p.z = vec_out.z();
}

template<typename PointT>
inline void TransformPoint(const Eigen::Quaternionf &q, const Eigen::Vector3f &t, PointT &p) {
  Eigen::Vector3f vec, vec_out;
  RotatePoint(q, p);
  TranslatePoint(t, p);
}

} // namespace geometryutils

#endif //LIO_GEOMETRY_UTILS_H_

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

#ifndef LIO_CIRCULARBUFFER_H_
#define LIO_CIRCULARBUFFER_H_

#include <cstdlib>
#include <vector>
#include <Eigen/Eigen>

namespace lio {

/// Adapted from https://github.com/laboshinl/loam_velodyne/blob/master/include/loam_velodyne/CircularBuffer.h

/** \brief Simple circular buffer implementation for storing data history.
 *
 * @tparam T The buffer element type.
 */
template<typename T>
class CircularBuffer {
 public:
  CircularBuffer(const size_t &capacity = 200)
      : capacity_(capacity),
        size_(0),
        start_idx_(0) {
    buffer_ = new T[capacity];
  };

  ~CircularBuffer() {
    delete[] buffer_;
    buffer_ = NULL;
  }

  void Reset(size_t capacity = 1) {
    delete[] buffer_;
    buffer_ = NULL;

    capacity_ = capacity;
    size_ = 0;
    start_idx_ = 0;
    buffer_ = new T[capacity];
  }

  /** \brief Retrieve the buffer size.
   *
   * @return the buffer size
   */
  const size_t &size() {
    return size_;
  }

  /** \brief Retrieve the buffer capacity.
   *
   * @return the buffer capacity
   */
  const size_t &capacity() {
    return capacity_;
  }

  /** \brief Ensure that this buffer has at least the required capacity.
   *
   * @param req_apacity the minimum required capacity
   */
  void EnsureCapacity(const int &req_apacity) {
    if (req_apacity > 0 && capacity_ < req_apacity) {
      // create new buffer and copy (valid) entries
      T *new_buffer = new T[req_apacity];
      for (size_t i = 0; i < size_; i++) {
        new_buffer[i] = (*this)[i];
      }

      // switch buffer pointers and delete old buffer
      T *old_buffer = buffer_;
      buffer_ = new_buffer;
      start_idx_ = 0;

      delete[] old_buffer;
    }
  }

  /** \brief Check if the buffer is empty.
   *
   * @return true if the buffer is empty, false otherwise
   */
  bool empty() {
    return size_ == 0;
  }

  /** \brief Retrieve the i-th element of the buffer.
 *
 *
 * @param i the buffer index
 * @return the element at the i-th position as no-constant
 */
  T &operator[](const size_t &i) {
    return buffer_[(start_idx_ + i) % capacity_];
  }

  /** \brief Retrieve the i-th element of the buffer.
   *
   *
   * @param i the buffer index
   * @return the element at the i-th position
   */
  const T &operator[](const size_t &i) const {
    return buffer_[(start_idx_ + i) % capacity_];
  }

  /** \brief Retrieve the first (oldest) element of the buffer.
   *
   * @return the first element
   */
  const T &first() const {
    return buffer_[start_idx_];
  }

  T &first() {
    return buffer_[start_idx_];
  }

  /** \brief Retrieve the last (latest) element of the buffer.
   *
   * @return the last element
   */
  const T &last() const {
    size_t idx = size_ == 0 ? 0 : (start_idx_ + size_ - 1) % capacity_;
    return buffer_[idx];
  }

  T &last() {
    size_t idx = size_ == 0 ? 0 : (start_idx_ + size_ - 1) % capacity_;
    return buffer_[idx];
  }

  /** \brief Push a new element to the buffer.
   *
   * If the buffer reached its capacity, the oldest element is overwritten.
   *
   * @param element the element to push
   */
  void push(const T &element) {
    if (size_ < capacity_) {
      buffer_[size_] = element;
      ++size_;
    } else {
      buffer_[start_idx_] = element;
      start_idx_ = (start_idx_ + 1) % capacity_;
    }
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  size_t capacity_;   ///< buffer capacity
  size_t size_;       ///< current buffer size
  size_t start_idx_;   ///< current start index
  T *buffer_;         ///< internal element buffer
};

} // namespace lio

#endif // LIO_CIRCULARBUFFER_H_

//
// Created by hyye on 2/13/20.
//

#ifndef LIO_IMUINITIALIZERSTATIC_H_
#define LIO_IMUINITIALIZERSTATIC_H_

#include "ImuInitializer.h"

namespace lio {

class ImuInitializerStatic : public ImuInitializer {
public:
  static bool
  Initialization(CircularBuffer<PairTimeLaserTransform> &all_laser_transforms,
                 CircularBuffer<Vector3d> &Vs, CircularBuffer<Vector3d> &Bas,
                 CircularBuffer<Vector3d> &Bgs, Vector3d &g,
                 Transform &transform_lb, Matrix3d &R_WI);
};

} // namespace lio

#endif // LIO_IMUINITIALIZERSTATIC_H_

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
// Created by hyye on 5/31/18.
//

#ifndef LIO_MAPBUILDER_H_
#define LIO_MAPBUILDER_H_

#include "point_processor/PointMapping.h"

namespace lio {

using namespace std;
using namespace mathutils;

struct MapBuilderConfig {
  float corner_filter_size = 0.2;
  float surf_filter_size = 0.4;
  float map_filter_size = 0.6;

  float min_match_sq_dis = 1.0;
  float min_plane_dis = 0.2;
};

class MapBuilder : public PointMapping {
 public:
  MapBuilder() = delete;
  MapBuilder(MapBuilderConfig config);
  void SetupRos(ros::NodeHandle &nh);
  void ProcessMap();
  void OptimizeMap();
  void PublishMapBuilderResults();
  void Transform4DAssociateToMap();
  void Transform4DUpdate();

 private:
  bool system_init_ = false;
  bool enable_4d_ = true;
  int skip_count_ = 2;
  MapBuilderConfig config_;
  int odom_count_ = 0;

};

}

#endif //LIO_MAPBUILDER_H_

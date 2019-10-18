//
// Created by hyye on 10/18/19.
//

#ifndef LIO_YAMLLOADER_H_
#define LIO_YAMLLOADER_H_

#include <string>
#include <opencv2/opencv.hpp>
#include "point_processor/PointProcessor.h"
//#include "point_processor/PointOdometry.h"
//#include "point_processor/PointMapping.h"
#include "imu_processor/MeasurementManager.h"
#include "imu_processor/Estimator.h"

namespace lio {

class YamlLoader {
 public:
  YamlLoader(std::string config_file) { LoadFile(config_file); };
  bool LoadFile(std::string config_file);
  PointProcessorConfig point_processor_config;
  MeasurementManagerConfig mm_config;
  EstimatorConfig estimator_config;
};

}

#endif //LIO_YAMLLOADER_H_

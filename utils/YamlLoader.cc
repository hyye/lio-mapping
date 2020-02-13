//
// Created by hyye on 10/18/19.
//

#include "utils/YamlLoader.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace lio {

bool YamlLoader::LoadFile(std::string config_file) {
  LOG(INFO) << "config_file: " << config_file;
  std::ifstream infile(config_file);
  if (!infile.good()) {
    return false;
  }
  cv::FileStorage fs_settings(config_file, cv::FileStorage::READ);

  // FIXME: re-organize the configs
  {
    int tmp_int;
    double tmp_double;
    estimator_config.min_match_sq_dis = fs_settings["min_match_sq_dis"];
    estimator_config.min_plane_dis = fs_settings["min_plane_dis"];

    estimator_config.corner_filter_size = fs_settings["corner_filter_size"];
    estimator_config.surf_filter_size = fs_settings["surf_filter_size"];
    estimator_config.map_filter_size = fs_settings["map_filter_size"];

    tmp_int = fs_settings["window_size"];
    estimator_config.window_size = size_t(tmp_int);

    tmp_int = fs_settings["opt_window_size"];
    estimator_config.opt_window_size = size_t(tmp_int);
    estimator_config.init_window_factor = fs_settings["init_window_factor"];

    tmp_int = fs_settings["estimate_extrinsic"];
    estimator_config.estimate_extrinsic = tmp_int;

    tmp_int = fs_settings["opt_extrinsic"];
    estimator_config.opt_extrinsic = (tmp_int > 0);

    cv::Mat cv_R, cv_T;
    fs_settings["extrinsic_rotation"] >> cv_R;
    fs_settings["extrinsic_translation"] >> cv_T;
    Eigen::Matrix3d eigen_R;
    Eigen::Vector3d eigen_T;
    cv::cv2eigen(cv_R, eigen_R);
    cv::cv2eigen(cv_T, eigen_T);

    estimator_config.transform_lb = Transform{Eigen::Quaternionf(eigen_R.cast<float>()), eigen_T.cast<float>()};

    tmp_int = fs_settings["run_optimization"];
    estimator_config.run_optimization = (tmp_int > 0);
    tmp_int = fs_settings["update_laser_imu"];
    estimator_config.update_laser_imu = (tmp_int > 0);
    tmp_int = fs_settings["gravity_fix"];
    estimator_config.gravity_fix = (tmp_int > 0);
    tmp_int = fs_settings["plane_projection_factor"];
    estimator_config.plane_projection_factor = (tmp_int > 0);
    tmp_int = fs_settings["imu_factor"];
    estimator_config.imu_factor = (tmp_int > 0);
    tmp_int = fs_settings["point_distance_factor"];
    estimator_config.point_distance_factor = (tmp_int > 0);
    tmp_int = fs_settings["prior_factor"];
    estimator_config.prior_factor = (tmp_int > 0);
    tmp_int = fs_settings["marginalization_factor"];
    estimator_config.marginalization_factor = (tmp_int > 0);

    tmp_int = fs_settings["pcl_viewer"];
    estimator_config.pcl_viewer = (tmp_int > 0);

    tmp_int = fs_settings["enable_deskew"];
    estimator_config.enable_deskew = (tmp_int > 0);
    tmp_int = fs_settings["cutoff_deskew"];
    estimator_config.cutoff_deskew = (tmp_int > 0);

    tmp_int = fs_settings["keep_features"];
    estimator_config.keep_features = (tmp_int > 0);

    tmp_double = fs_settings["acc_n"];
    estimator_config.pim_config.acc_n = tmp_double;
    tmp_double = fs_settings["gyr_n"];
    estimator_config.pim_config.gyr_n = tmp_double;
    tmp_double = fs_settings["acc_w"];
    estimator_config.pim_config.acc_w = tmp_double;
    tmp_double = fs_settings["gyr_w"];
    estimator_config.pim_config.gyr_w = tmp_double;
    tmp_double = fs_settings["g_norm"];
    estimator_config.pim_config.g_norm = tmp_double;

    tmp_double = fs_settings["msg_time_delay"];
    mm_config.msg_time_delay = tmp_double;

    tmp_double = fs_settings["odom_io"];;
    mm_config.odom_io = tmp_double;

    if (!fs_settings["scan_period"].empty()) {
      tmp_double = fs_settings["scan_period"];
      mm_config.scan_period = tmp_double;
      point_processor_config.scan_period = tmp_double;
    }

    if (!fs_settings["static_init"].empty()) {
      tmp_double = fs_settings["static_init"];
      estimator_config.static_init = (tmp_double > 0);
    }
  }
}

}  // namespace lio
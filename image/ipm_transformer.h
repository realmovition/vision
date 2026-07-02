#pragma once

#include <memory>

#include <opencv2/core.hpp>

#include "calibration/camera_model.h"

namespace wheel {
namespace vision {
namespace image {

struct IpmConfig {
  double resolution{0.05};

  int bev_width{400};
  int bev_height{600};

  // Vehicle frame:
  // x: right positive
  // y: forward positive
  // z: up positive
  double x_min{-10.0};
  double y_min{3.0};

  // Ground plane height in vehicle frame.
  // Usually z = 0 if vehicle frame origin is on ground.
  double ground_z{0.0};

  // Minimum valid depth in camera frame.
  double min_camera_z{0.1};
};

class IpmTransformer {
 public:
  IpmTransformer(std::shared_ptr<calibration::CameraModel> camera_model,
                 const cv::Mat& T_vehicle_camera, const IpmConfig& config);

  void Transform(const cv::Mat& src_img, cv::Mat& dst_bev_img) const;

  const cv::Mat& map_x() const { return map_x_; }
  const cv::Mat& map_y() const { return map_y_; }

 private:
  void ValidateInputs() const;
  void PrecomputeLut();

 private:
  std::shared_ptr<calibration::CameraModel> camera_model_;

  // T_vehicle_camera: camera frame -> vehicle frame
  cv::Mat T_vehicle_camera_;

  // T_camera_vehicle: vehicle frame -> camera frame
  cv::Mat T_camera_vehicle_;

  IpmConfig config_;

  cv::Mat map_x_;
  cv::Mat map_y_;
};

}  // namespace image
}  // namespace vision
}  // namespace wheel

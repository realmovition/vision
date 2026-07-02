#include "image/ipm_transformer.h"

#include <stdexcept>

#include <Eigen/Core>
#include <opencv2/imgproc.hpp>

namespace wheel {
namespace vision {
namespace image {

IpmTransformer::IpmTransformer(
    std::shared_ptr<calibration::CameraModel> camera_model,
    const cv::Mat& T_vehicle_camera, const IpmConfig& config)
    : camera_model_(std::move(camera_model)), config_(config) {
  if (!camera_model_) {
    throw std::invalid_argument("IpmTransformer: camera_model is nullptr.");
  }

  if (T_vehicle_camera.rows != 4 || T_vehicle_camera.cols != 4) {
    throw std::invalid_argument(
        "IpmTransformer: T_vehicle_camera must be 4x4.");
  }

  T_vehicle_camera.convertTo(T_vehicle_camera_, CV_64F);
  T_camera_vehicle_ = T_vehicle_camera_.inv();

  ValidateInputs();

  map_x_.create(config_.bev_height, config_.bev_width, CV_32FC1);
  map_y_.create(config_.bev_height, config_.bev_width, CV_32FC1);

  PrecomputeLut();
}

void IpmTransformer::ValidateInputs() const {
  if (config_.resolution <= 0.0) {
    throw std::invalid_argument("IpmTransformer: resolution must be positive.");
  }

  if (config_.bev_width <= 0 || config_.bev_height <= 0) {
    throw std::invalid_argument("IpmTransformer: invalid BEV size.");
  }

  if (config_.min_camera_z <= 0.0) {
    throw std::invalid_argument(
        "IpmTransformer: min_camera_z must be positive.");
  }
}

void IpmTransformer::PrecomputeLut() {
  const cv::Mat R = T_camera_vehicle_(cv::Rect(0, 0, 3, 3));
  const cv::Mat t = T_camera_vehicle_(cv::Rect(3, 0, 1, 3));

  for (int r = 0; r < config_.bev_height; ++r) {
    for (int c = 0; c < config_.bev_width; ++c) {
      // BEV pixel -> vehicle frame.
      //
      // Vehicle frame convention:
      // x: right positive
      // y: forward positive
      // z: up positive
      const double x_v =
          config_.x_min + static_cast<double>(c) * config_.resolution;
      const double y_v =
          config_.y_min +
          static_cast<double>(config_.bev_height - 1 - r) * config_.resolution;
      const double z_v = config_.ground_z;

      const double x_c = R.at<double>(0, 0) * x_v + R.at<double>(0, 1) * y_v +
                         R.at<double>(0, 2) * z_v + t.at<double>(0, 0);

      const double y_c = R.at<double>(1, 0) * x_v + R.at<double>(1, 1) * y_v +
                         R.at<double>(1, 2) * z_v + t.at<double>(1, 0);

      const double z_c = R.at<double>(2, 0) * x_v + R.at<double>(2, 1) * y_v +
                         R.at<double>(2, 2) * z_v + t.at<double>(2, 0);

      float& map_x = map_x_.at<float>(r, c);
      float& map_y = map_y_.at<float>(r, c);

      map_x = -1.0f;
      map_y = -1.0f;

      if (z_c <= config_.min_camera_z) {
        continue;
      }

      Eigen::Vector3d pt_camera(x_c, y_c, z_c);
      Eigen::Vector2d pt_image;

      if (!camera_model_->Project(pt_camera, &pt_image)) {
        continue;
      }

      map_x = static_cast<float>(pt_image.x());
      map_y = static_cast<float>(pt_image.y());
    }
  }
}

void IpmTransformer::Transform(const cv::Mat& src_img,
                               cv::Mat& dst_bev_img) const {
  if (src_img.empty()) {
    throw std::invalid_argument("IpmTransformer::Transform: src_img is empty.");
  }

  cv::remap(src_img, dst_bev_img, map_x_, map_y_, cv::INTER_LINEAR,
            cv::BORDER_CONSTANT, cv::Scalar::all(0));
}

}  // namespace image
}  // namespace vision
}  // namespace wheel

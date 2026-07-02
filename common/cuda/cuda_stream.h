#pragma once

#include <cuda_runtime_api.h>

#include "common/cuda/cuda_types.h"

namespace wheel {
namespace vision {
namespace lidar {

constexpr CudaStream kDefaultCudaStream = nullptr;

}  // namespace lidar
}  // namespace vision
}  // namespace wheel

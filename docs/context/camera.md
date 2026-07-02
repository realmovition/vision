# Camera Module Context

This document defines the canonical package boundary for the camera library
after separating camera and lidar into standalone directories.

## Canonical package

The camera implementation now lives under:

```text
calibration/
```

Recommended public dependency and include:

```cpp
#include "calibration/camera_model.h"
```

```text
@vision//calibration:calibration
```

## Compatibility policy

There is no root-level compatibility wrapper anymore. Camera code must depend on
the canonical package directly:

- `@vision//calibration:calibration`
- `#include "calibration/camera_model.h"`

## Why split camera and lidar

The repository now has two low-level domains with different optimization goals:

1. **Camera**
   - CPU-oriented projection and back-projection math
   - stable OpenCV-compatible parameter conventions
   - lightweight model composition and testing
2. **Lidar**
   - GPU-oriented memory and kernel execution
   - explicit workspaces, buffers, and stream-aware processing
   - higher throughput requirements and different module growth path

Keeping them in separate directories makes ownership, packaging, and future API
boundaries much clearer.

## Validation commands

```bash
bazel test //calibration:all
```

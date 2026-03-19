# floam_core

![Version](https://img.shields.io/badge/version-1.0.0-blue)
![License](https://img.shields.io/badge/license-Apache--2.0-green)
![C++](https://img.shields.io/badge/C++-17-blue)

A ROS-agnostic pure C++ library implementing the core algorithm of
[F-LOAM: Fast LiDAR Odometry and Mapping](https://github.com/wh200720041/floam)
(IROS 2021, Wang Han, Nanyang Technological University).

`floam_core` is refactored from the original implementation to separate
the algorithm logic from any ROS dependency. It is designed to be consumed by
downstream packages — such as the companion [`floam`](https://github.com/Chris7462/floam)
ROS2 package — or embedded into any C++ project independently.

Key changes from the original:
- ROS completely removed — pure C++ with no ROS headers
- Upgraded to C++17
- g2o replaces the original Ceres solver
- CMake modern targets (`floam_core::floam_core`) for clean downstream integration

---

## Algorithm

The pipeline consists of three sequential stages:

**1. LiDAR Processing** — Raw point cloud is organized by scan line and segmented
into edge points (high curvature) and surface points (low curvature) using a
sector-based curvature analysis.

**2. Odometry Estimation** — Edge and surface features are matched against a local
map using nearest-neighbor search (KD-tree). A nonlinear least-squares problem
is formulated with analytic Jacobians and solved by the selected solver backend
using a Huber loss function.

**3. LiDAR Mapping** — Estimated poses are used to accumulate a voxel-filtered
3D map. A crop box filter maintains a local window around the current position
to bound memory usage.

---

## Dependencies

| Dependency | Version |
|---|---|
| CMake | ≥ 3.16 |
| C++ | 17 |
| PCL | ≥ 1.14 |
| Eigen3 | ≥ 3.4 |
| g2o | latest |
| Sophus | latest |

Install system dependencies on Ubuntu:

```bash
sudo apt-get install libpcl-all-dev libeigen3-dev libg2o libsophus-dev
```

---

## Building

`floam_core` uses `ament_cmake` and is built inside a ROS2 workspace with `colcon`:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --branch g2o https://github.com/Chris7462/floam_core.git
cd ~/ros2_ws
colcon build --packages-select floam_core
source install/setup.bash
```

---

## Related Packages

**[floam](https://github.com/Chris7462/floam)** — The companion ROS2 package that wraps `floam_core` with `rclcpp` nodes for `lidar_processing`, `odom_estimation`, and `lidar_mapping`.

---

## Reference

**[F-LOAM: Fast LiDAR Odometry and Mapping](https://github.com/wh200720041/floam)** — The original implementation by Wang Han, from which `floam_core` was refactored. Uses `catkin`, C++14, and Ceres Solver.

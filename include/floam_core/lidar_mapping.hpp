#pragma once

// pcl header
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// eigen header
#include <Eigen/Geometry>

// c++ header
#include <vector>


namespace floam_core
{

inline constexpr double LASER_CELL_WIDTH = 50.0;
inline constexpr double LASER_CELL_HEIGHT = 50.0;
inline constexpr double LASER_CELL_DEPTH = 50.0;

inline constexpr int LASER_CELL_RANGE_HORIZONTAL = 2;
inline constexpr int LASER_CELL_RANGE_VERTICAL = 2;


class LidarMapping
{
public:
  LidarMapping() = default;

  void init(double map_resolution);

  void update_current_points_to_map(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in,
    const Eigen::Isometry3d & pose_current);

  pcl::PointCloud<pcl::PointXYZI>::Ptr get_map();

private:
  int origin_in_map_x_;
  int origin_in_map_y_;
  int origin_in_map_z_;
  int map_width_;
  int map_height_;
  int map_depth_;
  std::vector<std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>>> map_;
  pcl::VoxelGrid<pcl::PointXYZI> down_size_filter_;

  void add_width_cell_negative();
  void add_width_cell_positive();
  void add_height_cell_negative();
  void add_height_cell_positive();
  void add_depth_cell_negative();
  void add_depth_cell_positive();
  void check_points(int & x, int & y, int & z);
};

}  // namespace floam_core

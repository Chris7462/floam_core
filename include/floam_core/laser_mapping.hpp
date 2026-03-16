#pragma once

// pcl header
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// eigen header
#include <Eigen/Geometry>

// c++ header
#include <vector>

#define LASER_CELL_WIDTH 50.0
#define LASER_CELL_HEIGHT 50.0
#define LASER_CELL_DEPTH 50.0

#define LASER_CELL_RANGE_HORIZONTAL 2
#define LASER_CELL_RANGE_VERTICAL 2


namespace floam_core
{

class LaserMapping
{
  public:
    LaserMapping() = default;
    void init(double map_resolution);
    void updateCurrentPointsToMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, const Eigen::Isometry3d& pose_current);
    pcl::PointCloud<pcl::PointXYZI>::Ptr getMap();

  private:
    int origin_in_map_x;
    int origin_in_map_y;
    int origin_in_map_z;
    int map_width;
    int map_height;
    int map_depth;
    std::vector<std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>>> map;
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;

    void addWidthCellNegative();
    void addWidthCellPositive();
    void addHeightCellNegative();
    void addHeightCellPositive();
    void addDepthCellNegative();
    void addDepthCellPositive();
    void checkPoints(int& x, int& y, int& z);
};

} // namespace floam_core

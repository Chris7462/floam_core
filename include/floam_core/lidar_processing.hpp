#pragma once

// pcl header
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// local header
#include "floam_core/lidar.hpp"


namespace floam_core
{

// points covariance struct
struct Double2d
{
  int id;
  double value;
  Double2d(int id_in, double value_in);
};

// points info struct
struct PointsInfo
{
  int layer;
  double time;
  PointsInfo(int layer_in, double time_in);
};

class LidarProcessing
{
public:
  LidarProcessing() = default;

  void init(Lidar lidar_param_in);

  void feature_extraction(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out_edge,
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out_surf);

  void feature_extraction_from_sector(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in,
    std::vector<Double2d> & cloud_curvature,
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out_edge,
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_out_surf);

private:
  Lidar lidar_param_;
};

} // namespace floam_core

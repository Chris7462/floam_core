#pragma once

// pcl header
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// local header
#include "floam_core/lidar.hpp"


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

class LaserProcessing
{
public:
  LaserProcessing() = default;
  void init(lidar::Lidar lidar_param_in);
  void featureExtraction(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_edge,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf);
  void featureExtractionFromSector(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in,
    std::vector<Double2d>& cloudCurvature,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_edge,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf);

private:
  lidar::Lidar lidar_param;
};

#pragma once

// ceres header
#include <ceres/ceres.h>

// pcl header
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

// eigen header
#include <Eigen/Dense>
#include <Eigen/Geometry>

// local header
#include "floam_core/lidar_optimization.hpp"


namespace floam_core
{

class OdomEstimation
{
public:
  OdomEstimation() = default;

  void init(double map_resolution);

  void init_map_with_points(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr edge_in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr surf_in);

  void update_points_to_map(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr edge_in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr surf_in);

  void get_map(pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud_map);

  Eigen::Isometry3d odom_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud_corner_map_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud_surf_map_;

private:
  // optimization parameter block: [qx, qy, qz, qw, tx, ty, tz]
  double parameters_[7] = {0, 0, 0, 1, 0, 0, 0};
  Eigen::Map<Eigen::Quaterniond> q_w_curr_{parameters_};
  Eigen::Map<Eigen::Vector3d> t_w_curr_{parameters_ + 4};

  Eigen::Isometry3d last_odom_;

  // kd-tree
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_edge_map_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_surf_map_;

  // points downsampling before add to map
  pcl::VoxelGrid<pcl::PointXYZI> down_size_filter_edge_;
  pcl::VoxelGrid<pcl::PointXYZI> down_size_filter_surf_;

  // local map
  pcl::CropBox<pcl::PointXYZI> crop_box_filter_;

  // optimization count
  int optimization_count_;

  // functions
  void add_edge_cost_factor(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr map_in,
    ceres::Problem & problem,
    ceres::LossFunction * loss_function);

  void add_surf_cost_factor(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr map_in,
    ceres::Problem & problem,
    ceres::LossFunction * loss_function);

  void add_points_to_map(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_edge_cloud,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_surf_cloud);

  void point_associate_to_map(
    pcl::PointXYZI const * const pi,
    pcl::PointXYZI * const po);

  void down_sampling_to_map(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr edge_pc_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr edge_pc_out,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr surf_pc_in,
    pcl::PointCloud<pcl::PointXYZI>::Ptr surf_pc_out);
};

}  // namespace floam_core

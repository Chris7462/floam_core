// g2o
#include <g2o/core/optimization_algorithm_gauss_newton.h>

// local header
#include "floam_core/odom_estimation.hpp"


namespace floam_core
{

void OdomEstimation::init(double map_resolution)
{
  // init local map
  lidar_cloud_corner_map = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  lidar_cloud_surf_map = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

  // downsampling size
  down_size_filter_edge.setLeafSize(map_resolution, map_resolution, map_resolution);
  down_size_filter_surf.setLeafSize(map_resolution * 2, map_resolution * 2, map_resolution * 2);

  // kd-tree
  kdtree_edge_map = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  kdtree_surf_map = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());

  odom = Eigen::Isometry3d::Identity();
  last_odom = Eigen::Isometry3d::Identity();
  optimization_count = 2;
}

void OdomEstimation::init_map_with_points(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr edge_in,
  const pcl::PointCloud<pcl::PointXYZI>::Ptr surf_in)
{
  *lidar_cloud_corner_map += *edge_in;
  *lidar_cloud_surf_map += *surf_in;
  optimization_count = 12;
}

void OdomEstimation::update_points_to_map(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr edge_in,
  const pcl::PointCloud<pcl::PointXYZI>::Ptr surf_in)
{
  if (optimization_count > 2) {
    optimization_count--;
  }

  Eigen::Isometry3d odom_prediction = odom * (last_odom.inverse() * odom);
  last_odom = odom;
  odom = odom_prediction;

  q_w_curr = Eigen::Quaterniond(odom.rotation());
  t_w_curr = odom.translation();

  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_edge_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_surf_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  down_sampling_to_map(edge_in, downsampled_edge_cloud, surf_in, downsampled_surf_cloud);

  Sophus::SE3d SE3_Rt(q_w_curr, t_w_curr);
  if (lidar_cloud_corner_map->points.size() > 10 && lidar_cloud_surf_map->points.size() > 50) {
    kdtree_edge_map->setInputCloud(lidar_cloud_corner_map);
    kdtree_surf_map->setInputCloud(lidar_cloud_surf_map);

    for (int iter_count = 0; iter_count < optimization_count; iter_count++) {
      auto solver = new g2o::OptimizationAlgorithmGaussNewton(
        std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));
      g2o::SparseOptimizer opt;
      opt.setAlgorithm(solver);
      opt.setVerbose(false);

      FloamVertex* v = new FloamVertex();
      v->setEstimate(SE3_Rt);
      v->setId(0);
      opt.addVertex(v);

      add_edge_cost_factor(downsampled_edge_cloud, lidar_cloud_corner_map, opt, v);
      add_surf_cost_factor(downsampled_surf_cloud, lidar_cloud_surf_map, opt, v);

      opt.initializeOptimization();
      opt.optimize(10);
      SE3_Rt = v->estimate();
      q_w_curr = Eigen::Quaterniond((SE3_Rt.matrix()).block<3, 3>(0, 0));
      t_w_curr = (SE3_Rt.matrix()).block<3, 1>(0, 3);
    }
  } else {
    printf("not enough points in map to associate, map error");
  }

  odom = Eigen::Isometry3d::Identity();
  odom.linear() = q_w_curr.toRotationMatrix();
  odom.translation() = t_w_curr;
  add_points_to_map(downsampled_edge_cloud, downsampled_surf_cloud);
}

void OdomEstimation::get_map(pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud_map)
{
  *lidar_cloud_map += *lidar_cloud_surf_map;
  *lidar_cloud_map += *lidar_cloud_corner_map;
}

void OdomEstimation::add_edge_cost_factor(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in,
  const pcl::PointCloud<pcl::PointXYZI>::Ptr map_in,
  g2o::SparseOptimizer& opt, FloamVertex* v)
{
  int corner_num = 0;
  for (int i = 0; i < (int)pc_in->points.size(); i++) {
    pcl::PointXYZI point_temp;
    point_associate_to_map(&(pc_in->points[i]), &point_temp);

    std::vector<int> point_search_ind;
    std::vector<float> point_search_sq_dis;
    kdtree_edge_map->nearestKSearch(point_temp, 5, point_search_ind, point_search_sq_dis);

    if (point_search_sq_dis[4] < 1.0) {
      std::vector<Eigen::Vector3d> near_corners;
      Eigen::Vector3d center(0, 0, 0);
      for (int j = 0; j < 5; j++) {
        Eigen::Vector3d tmp(map_in->points[point_search_ind[j]].x,
                            map_in->points[point_search_ind[j]].y,
                            map_in->points[point_search_ind[j]].z);
        center = center + tmp;
        near_corners.push_back(tmp);
      }
      center = center / 5.0;

      Eigen::Matrix3d cov_mat = Eigen::Matrix3d::Zero();
      for (int j = 0; j < 5; j++) {
        Eigen::Matrix<double, 3, 1> tmp_zero_mean = near_corners[j] - center;
        cov_mat = cov_mat + tmp_zero_mean * tmp_zero_mean.transpose();
      }

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cov_mat);

      Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
      Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);

      if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
        Eigen::Vector3d point_on_line = center;
        Eigen::Vector3d point_a, point_b;
        point_a = 0.1 * unit_direction + point_on_line;
        point_b = -0.1 * unit_direction + point_on_line;
        FloamEdge* edge = new FloamEdge(curr_point, point_a, point_b);
        edge->setId(i);
        edge->setVertex(0, v);
        // edge->setMeasurement(0);
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        opt.addEdge(edge);

        corner_num++;
      }
    }
  }

  if (corner_num < 20) {
    printf("not enough correct points");
  }
}

void OdomEstimation::add_surf_cost_factor(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in,
  const pcl::PointCloud<pcl::PointXYZI>::Ptr map_in,
  g2o::SparseOptimizer& opt, FloamVertex* v)
{
  int surf_num = 0;
  for (int i = 0; i < (int)pc_in->points.size(); i++) {
    pcl::PointXYZI point_temp;
    point_associate_to_map(&(pc_in->points[i]), &point_temp);
    std::vector<int> point_search_ind;
    std::vector<float> point_search_sq_dis;
    kdtree_surf_map->nearestKSearch(point_temp, 5, point_search_ind, point_search_sq_dis);

    Eigen::Matrix<double, 5, 3> mat_a0;
    Eigen::Matrix<double, 5, 1> mat_b0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();

    if (point_search_sq_dis[4] < 1.0) {
      for (int j = 0; j < 5; j++) {
        mat_a0(j, 0) = map_in->points[point_search_ind[j]].x;
        mat_a0(j, 1) = map_in->points[point_search_ind[j]].y;
        mat_a0(j, 2) = map_in->points[point_search_ind[j]].z;
      }
      // find the norm of plane
      Eigen::Vector3d norm = mat_a0.colPivHouseholderQr().solve(mat_b0);
      double negative_OA_dot_norm = 1 / norm.norm();
      norm.normalize();

      bool plane_valid = true;
      for (int j = 0; j < 5; j++) {
        if (fabs(norm(0) * map_in->points[point_search_ind[j]].x +
                 norm(1) * map_in->points[point_search_ind[j]].y +
                 norm(2) * map_in->points[point_search_ind[j]].z +
                 negative_OA_dot_norm) > 0.2) {
          plane_valid = false;
          break;
        }
      }

      Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);

      if (plane_valid) {
        FloamSurf* edge = new FloamSurf(curr_point, norm);
        edge->setId(i);
        edge->setVertex(0, v);
        edge->setMeasurement(negative_OA_dot_norm);
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        opt.addEdge(edge);

        surf_num++;
      }
    }
  }

  if (surf_num < 20) {
    printf("not enough correct points");
  }
}

void OdomEstimation::add_points_to_map(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_edge_cloud,
  const pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_surf_cloud)
{
  for (int i = 0; i < (int)downsampled_edge_cloud->points.size(); i++) {
    pcl::PointXYZI point_temp;
    point_associate_to_map(&downsampled_edge_cloud->points[i], &point_temp);
    lidar_cloud_corner_map->push_back(point_temp);
  }

  for (int i = 0; i < (int)downsampled_surf_cloud->points.size(); i++) {
    pcl::PointXYZI point_temp;
    point_associate_to_map(&downsampled_surf_cloud->points[i], &point_temp);
    lidar_cloud_surf_map->push_back(point_temp);
  }

  double x_min = +odom.translation().x() - 100;
  double y_min = +odom.translation().y() - 100;
  double z_min = +odom.translation().z() - 100;
  double x_max = +odom.translation().x() + 100;
  double y_max = +odom.translation().y() + 100;
  double z_max = +odom.translation().z() + 100;

  crop_box_filter.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
  crop_box_filter.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
  crop_box_filter.setNegative(false);

  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_corner(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_surf(new pcl::PointCloud<pcl::PointXYZI>());
  crop_box_filter.setInputCloud(lidar_cloud_corner_map);
  crop_box_filter.filter(*tmp_corner);
  crop_box_filter.setInputCloud(lidar_cloud_surf_map);
  crop_box_filter.filter(*tmp_surf);

  down_size_filter_edge.setInputCloud(tmp_corner);
  down_size_filter_edge.filter(*lidar_cloud_corner_map);
  down_size_filter_surf.setInputCloud(tmp_surf);
  down_size_filter_surf.filter(*lidar_cloud_surf_map);
}

void OdomEstimation::point_associate_to_map(pcl::PointXYZI const *const pi, pcl::PointXYZI *const po)
{
  Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
  Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
  po->x = point_w.x();
  po->y = point_w.y();
  po->z = point_w.z();
  po->intensity = pi->intensity;
}

void OdomEstimation::down_sampling_to_map(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr edge_pc_in,
  pcl::PointCloud<pcl::PointXYZI>::Ptr edge_pc_out,
  const pcl::PointCloud<pcl::PointXYZI>::Ptr surf_pc_in,
  pcl::PointCloud<pcl::PointXYZI>::Ptr surf_pc_out)
{
  down_size_filter_edge.setInputCloud(edge_pc_in);
  down_size_filter_edge.filter(*edge_pc_out);
  down_size_filter_surf.setInputCloud(surf_pc_in);
  down_size_filter_surf.filter(*surf_pc_out);
}

} // namespace floam_core

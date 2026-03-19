// local header
#include "floam_core/odom_estimation.hpp"


namespace floam_core
{

void OdomEstimation::init(double map_resolution)
{
  // init local map
  lidar_cloud_corner_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  lidar_cloud_surf_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  // downsampling size
  down_size_filter_edge_.setLeafSize(map_resolution, map_resolution, map_resolution);
  down_size_filter_surf_.setLeafSize(map_resolution * 2, map_resolution * 2, map_resolution * 2);

  // kd-tree
  kdtree_edge_map_ = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZI>>();
  kdtree_surf_map_ = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZI>>();

  odom_ = Eigen::Isometry3d::Identity();
  last_odom_ = Eigen::Isometry3d::Identity();
  optimization_count_ = 2;
}

void OdomEstimation::init_map_with_points(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr edge_in,
  const pcl::PointCloud<pcl::PointXYZI>::Ptr surf_in)
{
  *lidar_cloud_corner_map_ += *edge_in;
  *lidar_cloud_surf_map_ += *surf_in;
  optimization_count_ = 12;
}

void OdomEstimation::update_points_to_map(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr edge_in,
  const pcl::PointCloud<pcl::PointXYZI>::Ptr surf_in)
{
  if (optimization_count_ > 2) {
    optimization_count_--;
  }

  const Eigen::Isometry3d odom_prediction = odom_ * (last_odom_.inverse() * odom_);
  last_odom_ = odom_;
  odom_ = odom_prediction;

  q_w_curr_ = Eigen::Quaterniond(odom_.rotation());
  t_w_curr_ = odom_.translation();

  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_edge_cloud(
    new pcl::PointCloud<pcl::PointXYZI>());

  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_surf_cloud(
    new pcl::PointCloud<pcl::PointXYZI>());

  down_sampling_to_map(edge_in, downsampled_edge_cloud, surf_in, downsampled_surf_cloud);

  if (lidar_cloud_corner_map_->points.size() > 10 &&
    lidar_cloud_surf_map_->points.size() > 50)
  {
    kdtree_edge_map_->setInputCloud(lidar_cloud_corner_map_);
    kdtree_surf_map_->setInputCloud(lidar_cloud_surf_map_);

    for (int iter_count = 0; iter_count < optimization_count_; iter_count++) {
      ceres::LossFunction * loss_function = new ceres::HuberLoss(0.1);
      ceres::Problem problem;
      problem.AddParameterBlock(parameters_, 7, new PoseSE3Manifold());

      add_edge_cost_factor(downsampled_edge_cloud, lidar_cloud_corner_map_, problem, loss_function);
      add_surf_cost_factor(downsampled_surf_cloud, lidar_cloud_surf_map_, problem, loss_function);

      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_QR;
      options.max_num_iterations = 4;
      options.minimizer_progress_to_stdout = false;
      options.check_gradients = false;
      options.gradient_check_relative_precision = 1e-4;
      ceres::Solver::Summary summary;

      ceres::Solve(options, &problem, &summary);
    }
  } else {
    printf("not enough points in map to associate, map error");
  }

  odom_ = Eigen::Isometry3d::Identity();
  odom_.linear() = q_w_curr_.toRotationMatrix();
  odom_.translation() = t_w_curr_;
  add_points_to_map(downsampled_edge_cloud, downsampled_surf_cloud);
}

void OdomEstimation::get_map(pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud_map)
{
  *lidar_cloud_map += *lidar_cloud_surf_map_;
  *lidar_cloud_map += *lidar_cloud_corner_map_;
}

void OdomEstimation::add_edge_cost_factor(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in,
  const pcl::PointCloud<pcl::PointXYZI>::Ptr map_in,
  ceres::Problem & problem,
  ceres::LossFunction * loss_function)
{
  int corner_num = 0;
  std::vector<int> point_search_ind;
  std::vector<float> point_search_sq_dist;

  for (int i = 0; i < static_cast<int>(pc_in->points.size()); i++) {
    pcl::PointXYZI point_temp;
    point_associate_to_map(&(pc_in->points[i]), &point_temp);

    kdtree_edge_map_->nearestKSearch(point_temp, 5, point_search_ind, point_search_sq_dist);

    if (point_search_sq_dist[4] < 1.0) {
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
      const Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
      const Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);

      if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
        const Eigen::Vector3d point_on_line = center;
        const Eigen::Vector3d point_a = 0.1 * unit_direction + point_on_line;
        const Eigen::Vector3d point_b = -0.1 * unit_direction + point_on_line;

        problem.AddResidualBlock(
          new EdgeAnalyticCostFunction(curr_point, point_a, point_b),
          loss_function,
          parameters_);

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
  ceres::Problem & problem,
  ceres::LossFunction * loss_function)
{
  int surf_num = 0;
  std::vector<int> point_search_ind;
  std::vector<float> point_search_sq_dist;

  for (int i = 0; i < static_cast<int>(pc_in->points.size()); i++) {
    pcl::PointXYZI point_temp;
    point_associate_to_map(&(pc_in->points[i]), &point_temp);

    kdtree_surf_map_->nearestKSearch(point_temp, 5, point_search_ind, point_search_sq_dist);

    if (point_search_sq_dist[4] < 1.0) {
      Eigen::Matrix<double, 5, 3> mat_a0;
      Eigen::Matrix<double, 5, 1> mat_b0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();

      for (int j = 0; j < 5; j++) {
        mat_a0(j, 0) = map_in->points[point_search_ind[j]].x;
        mat_a0(j, 1) = map_in->points[point_search_ind[j]].y;
        mat_a0(j, 2) = map_in->points[point_search_ind[j]].z;
      }

      // find the norm of plane
      Eigen::Vector3d norm = mat_a0.colPivHouseholderQr().solve(mat_b0);
      const double negative_OA_dot_norm = 1 / norm.norm();
      norm.normalize();

      bool plane_valid = true;
      for (int j = 0; j < 5; j++) {
        if (std::abs(norm(0) * map_in->points[point_search_ind[j]].x +
                     norm(1) * map_in->points[point_search_ind[j]].y +
                     norm(2) * map_in->points[point_search_ind[j]].z +
                     negative_OA_dot_norm) > 0.2)
        {
          plane_valid = false;
          break;
        }
      }

      const Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);

      if (plane_valid) {
        problem.AddResidualBlock(
          new SurfNormAnalyticCostFunction(curr_point, norm, negative_OA_dot_norm),
          loss_function,
          parameters_);

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
  pcl::PointXYZI point_temp;

  for (int i = 0; i < static_cast<int>(downsampled_edge_cloud->points.size()); i++) {
    point_associate_to_map(&downsampled_edge_cloud->points[i], &point_temp);
    lidar_cloud_corner_map_->push_back(point_temp);
  }

  for (int i = 0; i < static_cast<int>(downsampled_surf_cloud->points.size()); i++) {
    point_associate_to_map(&downsampled_surf_cloud->points[i], &point_temp);
    lidar_cloud_surf_map_->push_back(point_temp);
  }

  double x_min = odom_.translation().x() - 100;
  double y_min = odom_.translation().y() - 100;
  double z_min = odom_.translation().z() - 100;
  double x_max = odom_.translation().x() + 100;
  double y_max = odom_.translation().y() + 100;
  double z_max = odom_.translation().z() + 100;

  crop_box_filter_.setMin(Eigen::Vector4f(x_min, y_min, z_min, 1.0));
  crop_box_filter_.setMax(Eigen::Vector4f(x_max, y_max, z_max, 1.0));
  crop_box_filter_.setNegative(false);

  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_corner =
    std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_surf =
    std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  crop_box_filter_.setInputCloud(lidar_cloud_corner_map_);
  crop_box_filter_.filter(*tmp_corner);
  crop_box_filter_.setInputCloud(lidar_cloud_surf_map_);
  crop_box_filter_.filter(*tmp_surf);

  down_size_filter_edge_.setInputCloud(tmp_corner);
  down_size_filter_edge_.filter(*lidar_cloud_corner_map_);
  down_size_filter_surf_.setInputCloud(tmp_surf);
  down_size_filter_surf_.filter(*lidar_cloud_surf_map_);
}

void OdomEstimation::point_associate_to_map(
  pcl::PointXYZI const * const pi,
  pcl::PointXYZI * const po)
{
  Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
  Eigen::Vector3d point_w = q_w_curr_ * point_curr + t_w_curr_;
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
  down_size_filter_edge_.setInputCloud(edge_pc_in);
  down_size_filter_edge_.filter(*edge_pc_out);
  down_size_filter_surf_.setInputCloud(surf_pc_in);
  down_size_filter_surf_.filter(*surf_pc_out);
}

}  // namespace floam_core

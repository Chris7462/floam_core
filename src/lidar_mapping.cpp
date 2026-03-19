// pcl header
#include <pcl/common/transforms.h>

// local header
#include "floam_core/lidar_mapping.hpp"


namespace floam_core
{

void LidarMapping::init(double map_resolution)
{
  // init map. init can have real object, but future added block does not need
  for (int i = 0; i < LASER_CELL_RANGE_HORIZONTAL * 2 + 1; i++) {
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> map_height_temp;
    map_height_temp.reserve(LASER_CELL_RANGE_HORIZONTAL * 2 + 1);
    for (int j = 0; j < LASER_CELL_RANGE_HORIZONTAL * 2 + 1; j++) {
      std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
      map_depth_temp.reserve(LASER_CELL_RANGE_VERTICAL * 2 + 1);
      for (int k = 0; k < LASER_CELL_RANGE_VERTICAL * 2 + 1; k++) {
        map_depth_temp.push_back(std::make_shared<pcl::PointCloud<pcl::PointXYZI>>());
      }
      map_height_temp.push_back(std::move(map_depth_temp));
    }
    map_.push_back(std::move(map_height_temp));
  }

  origin_in_map_x_ = LASER_CELL_RANGE_HORIZONTAL;
  origin_in_map_y_ = LASER_CELL_RANGE_HORIZONTAL;
  origin_in_map_z_ = LASER_CELL_RANGE_VERTICAL;
  map_width_ = LASER_CELL_RANGE_HORIZONTAL * 2 + 1;
  map_height_ = LASER_CELL_RANGE_HORIZONTAL * 2 + 1;
  map_depth_ = LASER_CELL_RANGE_VERTICAL * 2 + 1;

  // downsampling size
  down_size_filter_.setLeafSize(map_resolution, map_resolution, map_resolution);
}

// update points to map
void LidarMapping::update_current_points_to_map(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in,
  const Eigen::Isometry3d & pose_current)
{
  int current_pos_id_x = static_cast<int>(std::floor(pose_current.translation().x() /
    LASER_CELL_WIDTH + 0.5)) + origin_in_map_x_;
  int current_pos_id_y = static_cast<int>(std::floor(pose_current.translation().y() /
    LASER_CELL_HEIGHT + 0.5)) + origin_in_map_y_;
  int current_pos_id_z = static_cast<int>(std::floor(pose_current.translation().z() /
    LASER_CELL_DEPTH + 0.5)) + origin_in_map_z_;

  // check if submap is null
  check_points(current_pos_id_x, current_pos_id_y, current_pos_id_z);

  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_pc =
    std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::transformPointCloud(*pc_in, *transformed_pc, pose_current.cast<float>());

  // save points
  for (int i = 0; i < static_cast<int>(transformed_pc->points.size()); i++) {
    pcl::PointXYZI point_temp = transformed_pc->points[i];
    // for visualization only
    point_temp.intensity = std::min(1.0, std::max(pc_in->points[i].z + 2.0, 0.0) / 5);
    const int current_point_id_x = static_cast<int>(std::floor(point_temp.x / LASER_CELL_WIDTH +
      0.5)) + origin_in_map_x_;
    const int current_point_id_y = static_cast<int>(std::floor(point_temp.y / LASER_CELL_HEIGHT +
      0.5)) + origin_in_map_y_;
    const int current_point_id_z = static_cast<int>(std::floor(point_temp.z / LASER_CELL_DEPTH +
      0.5)) + origin_in_map_z_;

    map_[current_point_id_x][current_point_id_y][current_point_id_z]->push_back(point_temp);
  }

  // filtering points
  for (int i = current_pos_id_x - LASER_CELL_RANGE_HORIZONTAL;
    i < current_pos_id_x + LASER_CELL_RANGE_HORIZONTAL + 1; i++)
  {
    for (int j = current_pos_id_y - LASER_CELL_RANGE_HORIZONTAL;
      j < current_pos_id_y + LASER_CELL_RANGE_HORIZONTAL + 1; j++)
    {
      for (int k = current_pos_id_z - LASER_CELL_RANGE_VERTICAL;
        k < current_pos_id_z + LASER_CELL_RANGE_VERTICAL + 1; k++)
      {
        down_size_filter_.setInputCloud(map_[i][j][k]);
        down_size_filter_.filter(*(map_[i][j][k]));
      }
    }
  }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LidarMapping::get_map()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud_map =
    std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (int i = 0; i < map_width_; i++) {
    for (int j = 0; j < map_height_; j++) {
      for (int k = 0; k < map_depth_; k++) {
        if (map_[i][j][k] != nullptr) {
          *lidar_cloud_map += *(map_[i][j][k]);
        }
      }
    }
  }
  return lidar_cloud_map;
}

void LidarMapping::add_width_cell_negative()
{
  std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> map_height_temp;
  map_height_temp.reserve(map_height_);
  for (int j = 0; j < map_height_; j++) {
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
    map_depth_temp.reserve(map_depth_);
    for (int k = 0; k < map_depth_; k++) {
      map_depth_temp.push_back(nullptr);
    }
    map_height_temp.push_back(std::move(map_depth_temp));
  }
  map_.insert(map_.begin(), std::move(map_height_temp));

  origin_in_map_x_++;
  map_width_++;
}

void LidarMapping::add_width_cell_positive()
{
  std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> map_height_temp;
  map_height_temp.reserve(map_height_);
  for (int j = 0; j < map_height_; j++) {
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
    map_depth_temp.reserve(map_depth_);
    for (int k = 0; k < map_depth_; k++) {
      map_depth_temp.push_back(nullptr);
    }
    map_height_temp.push_back(std::move(map_depth_temp));
  }
  map_.push_back(std::move(map_height_temp));
  map_width_++;
}

void LidarMapping::add_height_cell_negative()
{
  for (int i = 0; i < map_width_; i++) {
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
    map_depth_temp.reserve(map_depth_);
    for (int k = 0; k < map_depth_; k++) {
      map_depth_temp.push_back(nullptr);
    }
    map_[i].insert(map_[i].begin(), std::move(map_depth_temp));
  }
  origin_in_map_y_++;
  map_height_++;
}

void LidarMapping::add_height_cell_positive()
{
  for (int i = 0; i < map_width_; i++) {
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
    map_depth_temp.reserve(map_depth_);
    for (int k = 0; k < map_depth_; k++) {
      map_depth_temp.push_back(nullptr);
    }
    map_[i].push_back(std::move(map_depth_temp));
  }
  map_height_++;
}

void LidarMapping::add_depth_cell_negative()
{
  for (int i = 0; i < map_width_; i++) {
    for (int j = 0; j < map_height_; j++) {
      map_[i][j].insert(map_[i][j].begin(), nullptr);
    }
  }
  origin_in_map_z_++;
  map_depth_++;
}

void LidarMapping::add_depth_cell_positive()
{
  for (int i = 0; i < map_width_; i++) {
    for (int j = 0; j < map_height_; j++) {
      map_[i][j].push_back(nullptr);
    }
  }
  map_depth_++;
}

// extend map if points exceed size
void LidarMapping::check_points(int & x, int & y, int & z)
{
  while (x + LASER_CELL_RANGE_HORIZONTAL > map_width_ - 1) {
    add_width_cell_positive();
  }
  while (x - LASER_CELL_RANGE_HORIZONTAL < 0) {
    add_width_cell_negative();
    x++;
  }
  while (y + LASER_CELL_RANGE_HORIZONTAL > map_height_ - 1) {
    add_height_cell_positive();
  }
  while (y - LASER_CELL_RANGE_HORIZONTAL < 0) {
    add_height_cell_negative();
    y++;
  }
  while (z + LASER_CELL_RANGE_VERTICAL > map_depth_ - 1) {
    add_depth_cell_positive();
  }
  while (z - LASER_CELL_RANGE_VERTICAL < 0) {
    add_depth_cell_negative();
    z++;
  }

  // initialize. create object if area is null
  for (int i = x - LASER_CELL_RANGE_HORIZONTAL; i < x + LASER_CELL_RANGE_HORIZONTAL + 1; i++) {
    for (int j = y - LASER_CELL_RANGE_HORIZONTAL; j < y + LASER_CELL_RANGE_HORIZONTAL + 1; j++) {
      for (int k = z - LASER_CELL_RANGE_VERTICAL; k < z + LASER_CELL_RANGE_VERTICAL + 1; k++) {
        if (map_[i][j][k] == nullptr) {
          map_[i][j][k] = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        }
      }
    }
  }
}

}  // namespace floam_core

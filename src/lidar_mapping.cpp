// pcl header
#include <pcl/common/transforms.h>

// local header
#include "floam_core/lidar_mapping.hpp"


namespace floam_core
{

void LidarMapping::init(double map_resolution)
{
  //init map. init can have real object, but future added block does not need
  for (int i = 0; i < LASER_CELL_RANGE_HORIZONTAL * 2 + 1; i++) {
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> map_height_temp;
    for (int j = 0; j < LASER_CELL_RANGE_HORIZONTAL * 2 + 1; j++) {
      std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
      for (int k = 0; k < LASER_CELL_RANGE_VERTICAL * 2 + 1; k++) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp(new pcl::PointCloud<pcl::PointXYZI>);
        map_depth_temp.push_back(point_cloud_temp);
      }
      map_height_temp.push_back(map_depth_temp);
    }
    map.push_back(map_height_temp);
  }

  origin_in_map_x = LASER_CELL_RANGE_HORIZONTAL;
  origin_in_map_y = LASER_CELL_RANGE_HORIZONTAL;
  origin_in_map_z = LASER_CELL_RANGE_VERTICAL;
  map_width = LASER_CELL_RANGE_HORIZONTAL*2+1;
  map_height = LASER_CELL_RANGE_HORIZONTAL*2+1;
  map_depth = LASER_CELL_RANGE_HORIZONTAL*2+1;

  // downsampling size
  down_size_filter.setLeafSize(map_resolution, map_resolution, map_resolution);
}

// update points to map
void LidarMapping::update_current_points_to_map(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr pc_in,
  const Eigen::Isometry3d& pose_current)
{
  int current_pos_id_x = int(std::floor(pose_current.translation().x() / LASER_CELL_WIDTH + 0.5)) + origin_in_map_x;
  int current_pos_id_y = int(std::floor(pose_current.translation().y() / LASER_CELL_HEIGHT + 0.5)) + origin_in_map_y;
  int current_pos_id_z = int(std::floor(pose_current.translation().z() / LASER_CELL_DEPTH + 0.5)) + origin_in_map_z;

  // check is submap is null
  check_points(current_pos_id_x, current_pos_id_y, current_pos_id_z);

  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_pc(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*pc_in, *transformed_pc, pose_current.cast<float>());

  // save points
  for (int i = 0; i < (int)transformed_pc->points.size(); i++) {
    pcl::PointXYZI point_temp = transformed_pc->points[i];
    // for visualization only
    point_temp.intensity = std::min(1.0 , std::max(pc_in->points[i].z+2.0, 0.0) / 5);
    int current_point_id_x = int(std::floor(point_temp.x / LASER_CELL_WIDTH + 0.5)) + origin_in_map_x;
    int current_point_id_y = int(std::floor(point_temp.y / LASER_CELL_HEIGHT + 0.5)) + origin_in_map_y;
    int current_point_id_z = int(std::floor(point_temp.z / LASER_CELL_DEPTH + 0.5)) + origin_in_map_z;

    map[current_point_id_x][current_point_id_y][current_point_id_z]->push_back(point_temp);
  }

  // filtering points
  for (int i = current_pos_id_x - LASER_CELL_RANGE_HORIZONTAL; i < current_pos_id_x + LASER_CELL_RANGE_HORIZONTAL + 1; i++) {
    for (int j = current_pos_id_y - LASER_CELL_RANGE_HORIZONTAL; j < current_pos_id_y + LASER_CELL_RANGE_HORIZONTAL + 1; j++) {
      for (int k = current_pos_id_z - LASER_CELL_RANGE_VERTICAL; k < current_pos_id_z + LASER_CELL_RANGE_VERTICAL + 1; k++) {
        down_size_filter.setInputCloud(map[i][j][k]);
        down_size_filter.filter(*(map[i][j][k]));
      }
    }
  }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LidarMapping::get_map()
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr lidar_cloud_map = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
  for (int i = 0; i < map_width; i++) {
    for (int j = 0; j < map_height; j++) {
      for (int k = 0; k < map_depth; k++) {
        if (map[i][j][k] != nullptr) {
          *lidar_cloud_map += *(map[i][j][k]);
        }
      }
    }
  }
  return lidar_cloud_map;
}

void LidarMapping::add_width_cell_negative()
{
  std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> map_height_temp;
  for (int j = 0; j < map_height; j++) {
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
    for(int k = 0; k < map_depth; k++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
      map_depth_temp.push_back(point_cloud_temp);
    }
    map_height_temp.push_back(map_depth_temp);
  }
  map.insert(map.begin(), map_height_temp);

  origin_in_map_x++;
  map_width++;
}

void LidarMapping::add_width_cell_positive()
{
  std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> map_height_temp;
  for (int j = 0; j < map_height; j++) {
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
    for (int k = 0; k < map_depth; k++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
      map_depth_temp.push_back(point_cloud_temp);
    }
    map_height_temp.push_back(map_depth_temp);
  }
  map.push_back(map_height_temp);
  map_width++;
}

void LidarMapping::add_height_cell_negative()
{
  for (int i = 0; i < map_width; i++) {
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
    for (int k=0; k < map_depth; k++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
      map_depth_temp.push_back(point_cloud_temp);
    }
    map[i].insert(map[i].begin(), map_depth_temp);
  }
  origin_in_map_y++;
  map_height++;
}

void LidarMapping::add_height_cell_positive()
{
  for (int i = 0; i < map_width; i++) {
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> map_depth_temp;
    for (int k = 0; k < map_depth; k++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
      map_depth_temp.push_back(point_cloud_temp);
    }
    map[i].push_back(map_depth_temp);
  }
  map_height++;
}

void LidarMapping::add_depth_cell_negative()
{
  for (int i = 0; i < map_width; i++) {
    for (int j = 0; j < map_height; j++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
      map[i][j].insert(map[i][j].begin(), point_cloud_temp);
    }
  }
  origin_in_map_z++;
  map_depth++;
}

void LidarMapping::add_depth_cell_positive()
{
  for (int i = 0; i < map_width; i++) {
    for (int j = 0; j < map_height; j++) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp;
      map[i][j].push_back(point_cloud_temp);
    }
  }
  map_depth++;
}

// extend map if points exceed size
void LidarMapping::check_points(int& x, int& y, int& z)
{
  while (x + LASER_CELL_RANGE_HORIZONTAL > map_width - 1) {
    add_width_cell_positive();
  }
  while (x - LASER_CELL_RANGE_HORIZONTAL < 0) {
    add_width_cell_negative();
    x++;
  }
  while (y + LASER_CELL_RANGE_HORIZONTAL > map_height - 1) {
    add_height_cell_positive();
  }
  while (y - LASER_CELL_RANGE_HORIZONTAL < 0) {
    add_height_cell_negative();
    y++;
  }
  while (z + LASER_CELL_RANGE_VERTICAL > map_depth - 1) {
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
        if (map[i][j][k] == nullptr) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_temp(new pcl::PointCloud<pcl::PointXYZI>());
          map[i][j][k] = point_cloud_temp;
        }
      }
    }
  }
}

} // namespace floam_core

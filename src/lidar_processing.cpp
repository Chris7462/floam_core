// c++ header
#include <vector>
#include <cmath>

// pcl header
#include <pcl/filters/filter.h>

// local header
#include "floam_core/lidar_processing.hpp"


namespace floam_core
{

Double2d::Double2d(int id_in, double value_in)
{
  id = id_in;
  value = value_in;
}

PointsInfo::PointsInfo(int layer_in, double time_in)
{
  layer = layer_in;
  time = time_in;
}

void LidarProcessing::init(Lidar lidar_param_in)
{
  lidar_param = lidar_param_in;
}

void LidarProcessing::feature_extraction(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in,
  pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_edge,
  pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf)
{
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*pc_in, *pc_in, indices);

  int N_SCANS = lidar_param.num_lines;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> lidar_cloud_scans;

  for (int i = 0; i < N_SCANS; ++i) {
    lidar_cloud_scans.push_back(pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>()));
  }

  for (int i = 0; i < (int)pc_in->points.size(); i++) {
    int scanID = 0;
    double distance = sqrt(pc_in->points[i].x * pc_in->points[i].x + pc_in->points[i].y * pc_in->points[i].y);
    if (distance < lidar_param.min_distance || distance > lidar_param.max_distance) {
      continue;
    }

    double angle = atan(pc_in->points[i].z / distance) * 180 / M_PI;

    if (N_SCANS == 16) {
      scanID = int((angle + 15) / 2 + 0.5);
      if (scanID > (N_SCANS - 1) || scanID < 0) {
        continue;
      }
    } else if (N_SCANS == 32) {
      scanID = int((angle + 92.0/3.0) * 3.0 / 4.0);
      if (scanID > (N_SCANS - 1) || scanID < 0) {
        continue;
      }
    } else if (N_SCANS == 64) {
      if (angle >= -8.83) {
        scanID = int((2 - angle) * 3.0 + 0.5);
      } else {
        scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);
      }

      if (angle > 2 || angle < -24.33 || scanID > 63 || scanID < 0) {
        continue;
      }
    } else {
      printf("wrong scan number\n");
    }

    lidar_cloud_scans[scanID]->push_back(pc_in->points[i]);
  }

  for (int i = 0; i < N_SCANS; i++) {
    if (lidar_cloud_scans[i]->points.size() < 131) {
      continue;
    }

    std::vector<Double2d> cloud_curvature;
    int total_points = lidar_cloud_scans[i]->points.size() - 10;
    for (int j = 5; j < (int)lidar_cloud_scans[i]->points.size() - 5; ++j) {
      double diffX = lidar_cloud_scans[i]->points[j - 5].x + lidar_cloud_scans[i]->points[j - 4].x + lidar_cloud_scans[i]->points[j - 3].x + lidar_cloud_scans[i]->points[j - 2].x + lidar_cloud_scans[i]->points[j - 1].x - 10 * lidar_cloud_scans[i]->points[j].x + lidar_cloud_scans[i]->points[j + 1].x + lidar_cloud_scans[i]->points[j + 2].x + lidar_cloud_scans[i]->points[j + 3].x + lidar_cloud_scans[i]->points[j + 4].x + lidar_cloud_scans[i]->points[j + 5].x;
      double diffY = lidar_cloud_scans[i]->points[j - 5].y + lidar_cloud_scans[i]->points[j - 4].y + lidar_cloud_scans[i]->points[j - 3].y + lidar_cloud_scans[i]->points[j - 2].y + lidar_cloud_scans[i]->points[j - 1].y - 10 * lidar_cloud_scans[i]->points[j].y + lidar_cloud_scans[i]->points[j + 1].y + lidar_cloud_scans[i]->points[j + 2].y + lidar_cloud_scans[i]->points[j + 3].y + lidar_cloud_scans[i]->points[j + 4].y + lidar_cloud_scans[i]->points[j + 5].y;
      double diffZ = lidar_cloud_scans[i]->points[j - 5].z + lidar_cloud_scans[i]->points[j - 4].z + lidar_cloud_scans[i]->points[j - 3].z + lidar_cloud_scans[i]->points[j - 2].z + lidar_cloud_scans[i]->points[j - 1].z - 10 * lidar_cloud_scans[i]->points[j].z + lidar_cloud_scans[i]->points[j + 1].z + lidar_cloud_scans[i]->points[j + 2].z + lidar_cloud_scans[i]->points[j + 3].z + lidar_cloud_scans[i]->points[j + 4].z + lidar_cloud_scans[i]->points[j + 5].z;
      Double2d distance(j, diffX * diffX + diffY * diffY + diffZ * diffZ);
      cloud_curvature.push_back(distance);
    }

    for (int j = 0; j < 6; ++j) {
      int sector_length = (int)(total_points / 6);
      int sector_start = sector_length * j;
      int sector_end = sector_length * (j+1) - 1;
      if (j==5) {
          sector_end = total_points - 1;
      }
      std::vector<Double2d> sub_cloud_curvature(cloud_curvature.begin()+sector_start,cloud_curvature.begin()+sector_end);

      feature_extraction_from_sector(lidar_cloud_scans[i], sub_cloud_curvature, pc_out_edge, pc_out_surf);
    }
  }
}

void LidarProcessing::feature_extraction_from_sector(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in,
  std::vector<Double2d>& cloud_curvature,
  pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_edge,
  pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf)
{
  std::sort(cloud_curvature.begin(), cloud_curvature.end(), [](const Double2d & a, const Double2d & b) {
    return a.value < b.value;
  });

  int largest_picked_num = 0;
  std::vector<int> picked_points;
  int point_info_count = 0;
  for (int i = cloud_curvature.size()-1; i >= 0; i--) {
    int ind = cloud_curvature[i].id;
    if (std::find(picked_points.begin(), picked_points.end(), ind)==picked_points.end()) {
      if (cloud_curvature[i].value <= 0.1) {
        break;
      }

      largest_picked_num++;
      picked_points.push_back(ind);

      if (largest_picked_num <= 20) {
        pc_out_edge->push_back(pc_in->points[ind]);
        point_info_count++;
      } else {
        break;
      }

      for (int k=1; k<=5; k++) {
        double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k - 1].x;
        double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k - 1].y;
        double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k - 1].z;
        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
          break;
        }
        picked_points.push_back(ind+k);
      }
      for (int k=-1; k>=-5; k--) {
        double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k + 1].x;
        double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k + 1].y;
        double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k + 1].z;
        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
          break;
        }
        picked_points.push_back(ind+k);
      }
    }
  }

  for (int i = 0; i < (int)cloud_curvature.size(); i++) {
    int ind = cloud_curvature[i].id;
    if (std::find(picked_points.begin(), picked_points.end(), ind) == picked_points.end()) {
      pc_out_surf->push_back(pc_in->points[ind]);
    }
  }
}

} // namespace floam_core

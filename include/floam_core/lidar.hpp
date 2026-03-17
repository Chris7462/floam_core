#pragma once

namespace floam_core
{

struct Lidar
{
  double max_distance;
  double min_distance;
  int num_lines;
  double scan_period;
  int points_per_line;
  double horizontal_angle_resolution;
  double horizontal_angle;
  double vertical_angle_resolution;
  double vertical_angle;
};

} // namespace floam_core

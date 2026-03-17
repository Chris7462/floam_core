#pragma once

namespace floam_core
{

class Lidar
{
public:
  Lidar() = default;

  void set_scan_period(double scan_period_in);
  void set_lines(int num_lines_in);
  void set_vertical_angle(double vertical_angle_in);
  void set_vertical_resolution(double vertical_angle_resolution_in);
  void set_max_distance(double max_distance_in);
  void set_min_distance(double min_distance_in);

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

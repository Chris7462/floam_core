// local header
#include "floam_core/lidar.hpp"


namespace floam_core
{

void Lidar::set_lines(int num_lines_in)
{
  num_lines = num_lines_in;
}

void Lidar::set_vertical_angle(double vertical_angle_in)
{
  vertical_angle = vertical_angle_in;
}

void Lidar::set_vertical_resolution(double vertical_angle_resolution_in)
{
  vertical_angle_resolution = vertical_angle_resolution_in;
}

void Lidar::set_scan_period(double scan_period_in)
{
  scan_period = scan_period_in;
}

void Lidar::set_max_distance(double max_distance_in)
{
  max_distance = max_distance_in;
}

void Lidar::set_min_distance(double min_distance_in)
{
  min_distance = min_distance_in;
}

} // namespace floam_core

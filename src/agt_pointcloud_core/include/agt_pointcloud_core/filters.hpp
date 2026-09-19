#pragma once

#include <algorithm>
#include <cmath>
#include <string>

namespace agt_pointcloud_core
{

struct PointXYZ
{
  double x{};
  double y{};
  double z{};
};

struct PointContext
{
  PointXYZ source;
  PointXYZ robot;
};

inline double normalize_angle(double value)
{
  constexpr double kPi = 3.14159265358979323846;
  while (value > kPi) value -= 2.0 * kPi;
  while (value < -kPi) value += 2.0 * kPi;
  return value;
}

class RangeFilter
{
public:
  RangeFilter(double min_m, double max_m) : min_m_(min_m), max_m_(max_m) {}

  bool removes(const PointContext & p) const
  {
    const auto & q = p.source;
    const double r = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
    return r < min_m_ || r > max_m_;
  }

private:
  double min_m_;
  double max_m_;
};

class BoxFilter
{
public:
  BoxFilter(
    double x_min, double x_max, double y_min, double y_max,
    double z_min, double z_max)
  : x_min_(x_min), x_max_(x_max), y_min_(y_min), y_max_(y_max),
    z_min_(z_min), z_max_(z_max) {}

  bool valid() const
  {
    return x_min_ < x_max_ && y_min_ < y_max_ && z_min_ < z_max_;
  }

  bool removes(const PointContext & p) const
  {
    const auto & q = p.robot;
    return q.x >= x_min_ && q.x <= x_max_ &&
           q.y >= y_min_ && q.y <= y_max_ &&
           q.z >= z_min_ && q.z <= z_max_;
  }

private:
  double x_min_, x_max_, y_min_, y_max_, z_min_, z_max_;
};

class SectorFilter
{
public:
  SectorFilter(
    double center_rad, double half_width_rad,
    double min_range_m, double max_range_m,
    double z_min_m, double z_max_m)
  : center_rad_(center_rad), half_width_rad_(half_width_rad),
    min_range_m_(min_range_m), max_range_m_(max_range_m),
    z_min_m_(z_min_m), z_max_m_(z_max_m) {}

  bool removes(const PointContext & p) const
  {
    const auto & q = p.robot;
    const double r = std::hypot(q.x, q.y);
    if (r < min_range_m_ || r > max_range_m_ || q.z < z_min_m_ || q.z > z_max_m_) {
      return false;
    }
    const double bearing = std::atan2(q.y, q.x);
    return std::abs(normalize_angle(bearing - center_rad_)) <= half_width_rad_;
  }

private:
  double center_rad_, half_width_rad_;
  double min_range_m_, max_range_m_;
  double z_min_m_, z_max_m_;
};

}  // namespace agt_pointcloud_core

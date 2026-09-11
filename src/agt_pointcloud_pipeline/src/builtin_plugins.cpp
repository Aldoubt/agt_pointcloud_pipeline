#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "agt_pointcloud_core/filters.hpp"
#include "agt_pointcloud_pipeline/filter_plugin.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace agt_pointcloud_pipeline
{

class RangeFilterPlugin final : public FilterPlugin
{
public:
  void configure(rclcpp::Node & node, const std::string & prefix) override
  {
    const double min_m = node.declare_parameter<double>(prefix + ".min_m", 0.5);
    const double max_m = node.declare_parameter<double>(prefix + ".max_m", 120.0);
    if (min_m < 0.0 || max_m <= min_m) {
      throw std::runtime_error("invalid range filter limits for " + prefix);
    }
    filter_ = std::make_unique<agt_pointcloud_core::RangeFilter>(min_m, max_m);
  }

  bool removes(const agt_pointcloud_core::PointContext & point) const override
  {
    return filter_ && filter_->removes(point);
  }

private:
  std::unique_ptr<agt_pointcloud_core::RangeFilter> filter_;
};

class BoxFilterPlugin final : public FilterPlugin
{
public:
  void configure(rclcpp::Node & node, const std::string & prefix) override
  {
    const auto x_min = node.declare_parameter<double>(prefix + ".x_min", -1.0);
    const auto x_max = node.declare_parameter<double>(prefix + ".x_max", 1.0);
    const auto y_min = node.declare_parameter<double>(prefix + ".y_min", -1.0);
    const auto y_max = node.declare_parameter<double>(prefix + ".y_max", 1.0);
    const auto z_min = node.declare_parameter<double>(prefix + ".z_min", -1.0);
    const auto z_max = node.declare_parameter<double>(prefix + ".z_max", 1.0);
    filter_ = std::make_unique<agt_pointcloud_core::BoxFilter>(
      x_min, x_max, y_min, y_max, z_min, z_max);
    if (!filter_->valid()) {
      throw std::runtime_error("invalid box limits for " + prefix);
    }
  }

  bool removes(const agt_pointcloud_core::PointContext & point) const override
  {
    return filter_ && filter_->removes(point);
  }

private:
  std::unique_ptr<agt_pointcloud_core::BoxFilter> filter_;
};

class SectorFilterPlugin final : public FilterPlugin
{
public:
  void configure(rclcpp::Node & node, const std::string & prefix) override
  {
    constexpr double pi = 3.14159265358979323846;
    const auto center_deg = node.declare_parameter<double>(prefix + ".center_deg", 180.0);
    const auto width_deg = node.declare_parameter<double>(prefix + ".width_deg", 10.0);
    const auto min_range = node.declare_parameter<double>(prefix + ".min_range_m", 0.5);
    const auto max_range = node.declare_parameter<double>(prefix + ".max_range_m", 2.0);
    const auto z_min = node.declare_parameter<double>(prefix + ".z_min_m", -1.0);
    const auto z_max = node.declare_parameter<double>(prefix + ".z_max_m", 2.0);
    if (width_deg <= 0.0 || width_deg > 360.0 || min_range < 0.0 ||
      max_range <= min_range || z_max <= z_min)
    {
      throw std::runtime_error("invalid sector limits for " + prefix);
    }
    filter_ = std::make_unique<agt_pointcloud_core::SectorFilter>(
      center_deg * pi / 180.0, 0.5 * width_deg * pi / 180.0,
      min_range, max_range, z_min, z_max);
  }

  bool removes(const agt_pointcloud_core::PointContext & point) const override
  {
    return filter_ && filter_->removes(point);
  }

private:
  std::unique_ptr<agt_pointcloud_core::SectorFilter> filter_;
};

}  // namespace agt_pointcloud_pipeline

PLUGINLIB_EXPORT_CLASS(
  agt_pointcloud_pipeline::RangeFilterPlugin,
  agt_pointcloud_pipeline::FilterPlugin)
PLUGINLIB_EXPORT_CLASS(
  agt_pointcloud_pipeline::BoxFilterPlugin,
  agt_pointcloud_pipeline::FilterPlugin)
PLUGINLIB_EXPORT_CLASS(
  agt_pointcloud_pipeline::SectorFilterPlugin,
  agt_pointcloud_pipeline::FilterPlugin)

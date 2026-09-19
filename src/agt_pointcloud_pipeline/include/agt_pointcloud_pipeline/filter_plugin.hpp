#pragma once

#include <string>
#include "agt_pointcloud_core/filters.hpp"
#include "rclcpp/rclcpp.hpp"

namespace agt_pointcloud_pipeline
{

class FilterPlugin
{
public:
  virtual ~FilterPlugin() = default;

  virtual void configure(rclcpp::Node & node, const std::string & prefix) = 0;
  virtual bool removes(const agt_pointcloud_core::PointContext & point) const = 0;
};

}  // namespace agt_pointcloud_pipeline

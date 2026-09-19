#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "agt_pointcloud_core/filters.hpp"
#include "agt_pointcloud_pipeline/filter_plugin.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/header.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

bool finite_xyz(float x, float y, float z)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

sensor_msgs::msg::PointCloud2 make_xyz_cloud(
  const std_msgs::msg::Header & header,
  const std::vector<std::array<float, 3>> & points)
{
  sensor_msgs::msg::PointCloud2 output;
  output.header = header;
  output.height = 1;
  output.width = static_cast<std::uint32_t>(points.size());
  output.is_dense = false;
  sensor_msgs::PointCloud2Modifier modifier(output);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> x(output, "x");
  sensor_msgs::PointCloud2Iterator<float> y(output, "y");
  sensor_msgs::PointCloud2Iterator<float> z(output, "z");
  for (const auto & p : points) {
    *x = p[0]; *y = p[1]; *z = p[2];
    ++x; ++y; ++z;
  }
  return output;
}

}  // namespace

class FilterNode final : public rclcpp::Node
{
public:
  FilterNode()
  : Node("agt_pointcloud_filter"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    loader_("agt_pointcloud_pipeline", "agt_pointcloud_pipeline::FilterPlugin")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/agt/livox/points");
    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    raw_topic_ = declare_parameter<std::string>("raw_topic", "/agt/pointcloud/raw");
    filtered_topic_ = declare_parameter<std::string>("filtered_topic", "/agt/pointcloud/filtered");
    rejected_topic_ = declare_parameter<std::string>("rejected_topic", "/agt/pointcloud/rejected");
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.05);
    publish_raw_ = declare_parameter<bool>("publish_raw", true);
    publish_rejected_ = declare_parameter<bool>("publish_rejected", true);
    statistics_period_sec_ = declare_parameter<double>("statistics_period_sec", 2.0);

    const auto filter_names = declare_parameter<std::vector<std::string>>(
      "filter_chain", {"range", "self_box", "rear_sector"});
    for (const auto & name : filter_names) {
      const std::string prefix = "filters." + name;
      const auto type = declare_parameter<std::string>(prefix + ".type", "");
      if (type.empty()) {
        throw std::runtime_error("filter type is empty for " + name);
      }
      auto plugin = loader_.createSharedInstance(type);
      plugin->configure(*this, prefix);
      filters_.push_back({name, std::move(plugin)});
      removed_by_filter_[name] = 0;
    }

    raw_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(raw_topic_, rclcpp::SensorDataQoS());
    filtered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      filtered_topic_, rclcpp::SensorDataQoS());
    rejected_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      rejected_topic_, rclcpp::SensorDataQoS());

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FilterNode::on_cloud, this, std::placeholders::_1));

    if (statistics_period_sec_ > 0.0) {
      const auto period_ms = std::chrono::milliseconds(
        std::max(100, static_cast<int>(statistics_period_sec_ * 1000.0)));
      stats_timer_ = create_wall_timer(period_ms, std::bind(&FilterNode::log_statistics, this));
    }

    RCLCPP_INFO(
      get_logger(), "Point cloud pipeline: %s -> %s (%zu filters, target_frame=%s)",
      input_topic_.c_str(), filtered_topic_.c_str(), filters_.size(), target_frame_.c_str());
  }

private:
  struct NamedPlugin
  {
    std::string name;
    std::shared_ptr<agt_pointcloud_pipeline::FilterPlugin> plugin;
  };

  bool lookup_robot_from_source(
    const sensor_msgs::msg::PointCloud2 & cloud,
    tf2::Transform & robot_from_source)
  {
    if (cloud.header.frame_id == target_frame_) {
      robot_from_source.setIdentity();
      return true;
    }
    try {
      const auto stamped = tf_buffer_.lookupTransform(
        target_frame_, cloud.header.frame_id, rclcpp::Time(cloud.header.stamp),
        rclcpp::Duration::from_seconds(tf_timeout_sec_));
      const auto & t = stamped.transform.translation;
      const auto & r = stamped.transform.rotation;
      tf2::Quaternion q(r.x, r.y, r.z, r.w);
      if (q.length2() < 1e-12) {
        return false;
      }
      q.normalize();
      robot_from_source = tf2::Transform(q, tf2::Vector3(t.x, t.y, t.z));
      return true;
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "TF %s <- %s unavailable: %s",
        target_frame_.c_str(), cloud.header.frame_id.c_str(), ex.what());
      return false;
    }
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    if (cloud->header.frame_id.empty()) {
      return;
    }

    tf2::Transform robot_from_source;
    if (!lookup_robot_from_source(*cloud, robot_from_source)) {
      return;
    }

    if (publish_raw_) {
      raw_pub_->publish(*cloud);
    }

    std::vector<std::array<float, 3>> kept;
    std::vector<std::array<float, 3>> rejected;
    const std::size_t count = static_cast<std::size_t>(cloud->width) * cloud->height;
    input_points_ += count;
    kept.reserve(count);
    rejected.reserve(count / 8 + 1);

    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!finite_xyz(*x, *y, *z)) {
          ++invalid_removed_;
          continue;
        }

        const tf2::Vector3 robot = robot_from_source * tf2::Vector3(*x, *y, *z);
        const agt_pointcloud_core::PointContext ctx{
          {*x, *y, *z}, {robot.x(), robot.y(), robot.z()}};

        bool remove = false;
        for (const auto & entry : filters_) {
          if (entry.plugin->removes(ctx)) {
            ++removed_by_filter_[entry.name];
            remove = true;
            break;
          }
        }

        if (remove) {
          ++rejected_points_;
          if (publish_rejected_) rejected.push_back({*x, *y, *z});
        } else {
          kept.push_back({*x, *y, *z});
        }
      }
    } catch (const std::runtime_error & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "PointCloud2 requires float x/y/z: %s", ex.what());
      return;
    }

    output_points_ += kept.size();
    ++clouds_;
    filtered_pub_->publish(make_xyz_cloud(cloud->header, kept));
    if (publish_rejected_) {
      rejected_pub_->publish(make_xyz_cloud(cloud->header, rejected));
    }
  }

  void log_statistics()
  {
    if (input_points_ == 0) {
      return;
    }
    const double rejected_pct = 100.0 * static_cast<double>(rejected_points_) /
      static_cast<double>(input_points_);
    RCLCPP_INFO(
      get_logger(),
      "filter stats: clouds=%llu input=%llu output=%llu rejected=%llu (%.2f%%) invalid=%llu",
      static_cast<unsigned long long>(clouds_),
      static_cast<unsigned long long>(input_points_),
      static_cast<unsigned long long>(output_points_),
      static_cast<unsigned long long>(rejected_points_),
      rejected_pct,
      static_cast<unsigned long long>(invalid_removed_));
    for (const auto & entry : filters_) {
      RCLCPP_INFO(
        get_logger(), "  removed[%s]=%llu", entry.name.c_str(),
        static_cast<unsigned long long>(removed_by_filter_[entry.name]));
    }
  }

  std::string input_topic_, target_frame_, raw_topic_, filtered_topic_, rejected_topic_;
  double tf_timeout_sec_{};
  double statistics_period_sec_{};
  bool publish_raw_{};
  bool publish_rejected_{};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  pluginlib::ClassLoader<agt_pointcloud_pipeline::FilterPlugin> loader_;
  std::vector<NamedPlugin> filters_;
  std::unordered_map<std::string, std::uint64_t> removed_by_filter_;
  std::uint64_t clouds_{};
  std::uint64_t input_points_{};
  std::uint64_t output_points_{};
  std::uint64_t rejected_points_{};
  std::uint64_t invalid_removed_{};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rejected_pub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FilterNode>());
  rclcpp::shutdown();
  return 0;
}

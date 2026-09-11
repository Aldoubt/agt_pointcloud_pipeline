#include <gtest/gtest.h>
#include "agt_pointcloud_core/filters.hpp"

using agt_pointcloud_core::BoxFilter;
using agt_pointcloud_core::PointContext;
using agt_pointcloud_core::PointXYZ;
using agt_pointcloud_core::RangeFilter;
using agt_pointcloud_core::SectorFilter;

TEST(Filters, RangeUsesSourceFrame)
{
  RangeFilter f(0.5, 10.0);
  PointContext p{{0.1, 0.0, 0.0}, {5.0, 0.0, 0.0}};
  EXPECT_TRUE(f.removes(p));
}

TEST(Filters, BoxUsesRobotFrame)
{
  BoxFilter f(-1.0, 0.0, -0.5, 0.5, -0.2, 1.0);
  ASSERT_TRUE(f.valid());
  PointContext p{{5.0, 5.0, 5.0}, {-0.5, 0.0, 0.2}};
  EXPECT_TRUE(f.removes(p));
}

TEST(Filters, SectorIsRangeAndHeightBounded)
{
  constexpr double pi = 3.14159265358979323846;
  SectorFilter f(pi, 10.0 * pi / 180.0, 0.5, 2.0, -0.5, 1.0);
  EXPECT_TRUE(f.removes(PointContext{{}, {-1.0, 0.0, 0.0}}));
  EXPECT_FALSE(f.removes(PointContext{{}, {-3.0, 0.0, 0.0}}));
  EXPECT_FALSE(f.removes(PointContext{{}, {-1.0, 0.0, 2.0}}));
}

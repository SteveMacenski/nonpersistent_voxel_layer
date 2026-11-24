// Copyright (c) 2026 PAL Robotics S.L.
// Licensed under the BSD License, see the LICENSE file of this package.

// Resetting only the cells written in the previous update must produce exactly the same layer as
// resetting the whole maps at every update, and leave no stale obstacle in the master costmap.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nonpersistent_voxel_layer/nonpersistent_voxel_layer.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/buffer.h"

namespace
{

class TestLayer : public nav2_costmap_2d::NonPersistentVoxelLayer
{
public:
  explicit TestLayer(bool full_reset)
  : full_reset_(full_reset) {}

  // The reference layer resets the whole maps at every update, as the layer used to
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override
  {
    if (full_reset_) {
      resetMaps();
    }
    NonPersistentVoxelLayer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
  }

  // Replace the marking observations of the next update
  void setObservation(nav2_costmap_2d::Observation & obs)
  {
    static_marking_observations_.clear();
    static_marking_observations_.push_back(obs);
  }

private:
  bool full_reset_;
};

// mark: false puts all the points above max_obstacle_height, so nothing is marked
nav2_costmap_2d::Observation makeObservation(
  double rx, double ry, std::mt19937 & rng, bool mark)
{
  const unsigned int n_points = 3000;
  sensor_msgs::msg::PointCloud2 cloud;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(n_points);
  std::uniform_real_distribution<float> xy(-3.5f, 3.5f), z(-0.2f, 2.5f);
  sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x"), it_y(cloud, "y"), it_z(cloud, "z");
  for (unsigned int i = 0; i < n_points; ++i, ++it_x, ++it_y, ++it_z) {
    *it_x = static_cast<float>(rx) + xy(rng);
    *it_y = static_cast<float>(ry) + xy(rng);
    *it_z = mark ? z(rng) : 3.0f;
  }
  geometry_msgs::msg::Point origin;
  origin.x = rx;
  origin.y = ry;
  origin.z = 0.5;
  // obstacle range 3 m: points farther away are ignored, as on the robot
  return nav2_costmap_2d::Observation(origin, cloud, 3.0, 0.0, 6.0, 0.0);
}

struct Costmap
{
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> layered;
  std::shared_ptr<TestLayer> layer;
};

Costmap makeCostmap(
  const nav2_util::LifecycleNode::SharedPtr & node, tf2_ros::Buffer & tf,
  const std::string & name, bool rolling, bool full_reset)
{
  Costmap c;
  c.layered = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", rolling, true);
  c.layer = std::make_shared<TestLayer>(full_reset);
  c.layered->addPlugin(c.layer);
  c.layer->initialize(c.layered.get(), name, &tf, node, nullptr);
  // a "global" costmap much larger than the sensor range
  c.layered->resizeMap(800, 600, 0.05, -20.0, -15.0);
  c.layered->setFootprint(nav2_costmap_2d::makeFootprintFromRadius(0.2));
  return c;
}

void runEquivalence(bool rolling)
{
  const std::string prefix = rolling ? "rolling_" : "static_";
  std::vector<rclcpp::Parameter> overrides;
  for (const auto & name : {prefix + "full", prefix + "partial"}) {
    overrides.emplace_back(name + ".z_voxels", 10);
    overrides.emplace_back(name + ".z_resolution", 0.2);
    overrides.emplace_back(name + ".origin_z", 0.0);
    overrides.emplace_back(name + ".max_obstacle_height", 2.0);
    overrides.emplace_back(name + ".mark_threshold", 0);
  }
  auto node = std::make_shared<nav2_util::LifecycleNode>(
    prefix + "test_node", "", rclcpp::NodeOptions().parameter_overrides(overrides));
  // declared by Costmap2DROS in a real costmap, read by ObstacleLayer
  node->declare_parameter("track_unknown_space", true);
  tf2_ros::Buffer tf(node->get_clock());

  Costmap full = makeCostmap(node, tf, prefix + "full", rolling, true);
  Costmap partial = makeCostmap(node, tf, prefix + "partial", rolling, false);
  const unsigned int size = full.layer->getSizeInCellsX() * full.layer->getSizeInCellsY();

  std::mt19937 rng(42);
  for (int step = 0; step < 200; ++step) {
    // the robot drives back and forth, turning, so old obstacles must be cleared
    const double t = step * 0.2;
    const double rx = 12.0 * std::sin(t * 0.1), ry = 5.0 * std::sin(t * 0.23);
    const double yaw = t * 0.3;
    // every 25 updates there is nothing to mark: everything must be cleared
    auto obs = makeObservation(rx, ry, rng, step % 25 != 24);
    full.layer->setObservation(obs);
    partial.layer->setObservation(obs);
    full.layered->updateMap(rx, ry, yaw);
    partial.layered->updateMap(rx, ry, yaw);

    const unsigned char * a = full.layer->getCharMap();
    const unsigned char * b = partial.layer->getCharMap();
    unsigned int differences = 0, lethal = 0;
    for (unsigned int i = 0; i < size; ++i) {
      differences += a[i] != b[i];
      lethal += b[i] == nav2_costmap_2d::LETHAL_OBSTACLE;
    }
    ASSERT_EQ(differences, 0u) << "layer differs from the full reset one at step " << step;
    if (step % 25 != 24) {
      ASSERT_GT(lethal, 0u) << "nothing marked at step " << step;
    }

    // Every lethal cell of the master costmap comes from the current observation
    const auto * master = partial.layered->getCostmap();
    unsigned int stale = 0;
    for (unsigned int my = 0; my < master->getSizeInCellsY(); ++my) {
      for (unsigned int mx = 0; mx < master->getSizeInCellsX(); ++mx) {
        if (master->getCost(mx, my) != nav2_costmap_2d::LETHAL_OBSTACLE) {
          continue;
        }
        double wx, wy;
        unsigned int lx, ly;
        master->mapToWorld(mx, my, wx, wy);
        if (!partial.layer->worldToMap(wx, wy, lx, ly) ||
          partial.layer->getCost(lx, ly) != nav2_costmap_2d::LETHAL_OBSTACLE)
        {
          ++stale;
        }
      }
    }
    ASSERT_EQ(stale, 0u) << "stale obstacles in the master costmap at step " << step;
  }
}

}  // namespace

TEST(NonPersistentVoxelLayerPartialReset, SameAsFullResetStaticCostmap)
{
  runEquivalence(false);
}

TEST(NonPersistentVoxelLayerPartialReset, SameAsFullResetRollingCostmap)
{
  runEquivalence(true);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}

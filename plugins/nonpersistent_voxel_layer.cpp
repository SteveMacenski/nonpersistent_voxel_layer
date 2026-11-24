/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Steve Macenski
 *********************************************************************/

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <algorithm>
#include <limits>
#include <vector>
#include "nonpersistent_voxel_layer/nonpersistent_voxel_layer.hpp"

#define VOXEL_BITS 16

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

using nav2_costmap_2d::ObservationBuffer;
using nav2_costmap_2d::Observation;

namespace nav2_costmap_2d
{

void NonPersistentVoxelLayer::onInitialize()
{
  auto node = node_.lock();
  clock_ = node->get_clock();
  ObstacleLayer::onInitialize();
  footprint_clearing_enabled_ = node->get_parameter(
    name_ + ".footprint_clearing_enabled").as_bool();
  enabled_ = node->get_parameter(name_ + ".enabled").as_bool();
  max_obstacle_height_ = node->get_parameter(
    name_ + ".max_obstacle_height").as_double();
  combination_method_ = node->get_parameter(
    name_ + ".combination_method").as_int();

  size_z_ = node->declare_parameter(name_ + ".z_voxels", 16);
  origin_z_ = node->declare_parameter(name_ + ".origin_z", 16.0);
  z_resolution_ = node->declare_parameter(
    name_ + ".z_resolution", 0.05);
  unknown_threshold_ = node->declare_parameter(
    name_ + ".unknown_threshold", 15) + (VOXEL_BITS - size_z_);
  mark_threshold_ = node->declare_parameter(
    name_ + ".mark_threshold", 0);
  publish_voxel_ = node->declare_parameter(
    name_ + ".publish_voxel_map", false);

  if (publish_voxel_) {
    voxel_pub_ =
      node->create_publisher<nav2_msgs::msg::VoxelGrid>(
      "voxel_grid", rclcpp::QoS(1));
  }

  matchSize();
}

NonPersistentVoxelLayer::~NonPersistentVoxelLayer()
{
}

void NonPersistentVoxelLayer::updateFootprint(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!footprint_clearing_enabled_) {
    return;
  }

  transformFootprint(robot_x, robot_y, robot_yaw,
    getFootprint(), transformed_footprint_);

  for (unsigned int i = 0; i < transformed_footprint_.size(); i++) {
    touch(transformed_footprint_[i].x, transformed_footprint_[i].y,
      min_x, min_y, max_x, max_y);
  }

  if (!transformed_footprint_.empty()) {
    // The footprint cells are set free here and again by ObstacleLayer::updateCosts
    int x0 = std::numeric_limits<int>::max(), y0 = std::numeric_limits<int>::max();
    int x1 = std::numeric_limits<int>::min(), y1 = std::numeric_limits<int>::min();
    for (const auto & p : transformed_footprint_) {
      int mx, my;
      worldToMapEnforceBounds(p.x, p.y, mx, my);
      x0 = std::min(x0, mx);
      y0 = std::min(y0, my);
      x1 = std::max(x1, mx);
      y1 = std::max(y1, my);
    }
    markUpdated(x0, y0, x1, y1);
  }

  setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
}


void NonPersistentVoxelLayer::matchSize()
{
  ObstacleLayer::matchSize();
  voxel_grid_.resize(size_x_, size_y_, size_z_);
}

void NonPersistentVoxelLayer::reset()
{
  ObstacleLayer::reset();
  resetMaps();
}

void NonPersistentVoxelLayer::resetMaps()
{
  ObstacleLayer::resetMaps();
  voxel_grid_.reset();
  has_updated_area_ = false;
}

void NonPersistentVoxelLayer::markUpdated(
  unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1)
{
  if (!has_updated_area_) {
    updated_min_x_ = x0;
    updated_min_y_ = y0;
    updated_max_x_ = x1;
    updated_max_y_ = y1;
    has_updated_area_ = true;
    return;
  }
  updated_min_x_ = std::min(updated_min_x_, x0);
  updated_min_y_ = std::min(updated_min_y_, y0);
  updated_max_x_ = std::max(updated_max_x_, x1);
  updated_max_y_ = std::max(updated_max_y_, y1);
}

void NonPersistentVoxelLayer::resetUpdatedArea(
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!has_updated_area_) {
    return;
  }
  // The master costmap must also be updated where the obstacles of the last update were, or they
  // stay there. Done before a rolling window moves the origin, which these cells were written with
  double wx, wy;
  mapToWorld(updated_min_x_, updated_min_y_, wx, wy);
  touch(wx - resolution_ / 2.0, wy - resolution_ / 2.0, min_x, min_y, max_x, max_y);
  mapToWorld(updated_max_x_, updated_max_y_, wx, wy);
  touch(wx + resolution_ / 2.0, wy + resolution_ / 2.0, min_x, min_y, max_x, max_y);

  std::unique_lock<Costmap2D::mutex_t> lock(*getMutex());
  uint32_t * voxels = voxel_grid_.getData();
  // value of a column after VoxelGrid::reset(): all voxels unknown
  const uint32_t voxel_reset_value = ~static_cast<uint32_t>(0) >> 16;
  const unsigned int width = updated_max_x_ - updated_min_x_ + 1;
  for (unsigned int y = updated_min_y_; y <= updated_max_y_; ++y) {
    const unsigned int index = getIndex(updated_min_x_, y);
    std::fill_n(costmap_ + index, width, default_value_);
    std::fill_n(voxels + index, width, voxel_reset_value);
  }
  has_updated_area_ = false;
}

void NonPersistentVoxelLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  // reset what the last update wrote, instead of the whole maps
  resetUpdatedArea(min_x, min_y, max_x, max_y);

  // update origin information for rolling costmap publication
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2,
      robot_y - getSizeInMetersY() / 2);
  }

  // if not enabled, stop here
  if (!enabled_) {
    return;
  }

  // get the maximum sized window required to operate
  useExtraBounds(min_x, min_y, max_x, max_y);

  // get the marking observations
  bool current = true;
  std::vector<Observation> observations;
  current = getMarkingObservations(observations) && current;

  // update the global current status
  current_ = current;

  // place the new obstacles into a priority queue... each with a priority of zero to begin with
  for (std::vector<Observation>::const_iterator it = observations.begin();
    it != observations.end(); ++it)
  {
    const Observation & obs = *it;

    // the height filter of the observation buffer can leave a cloud without points, on which an
    // iterator can not be created (it takes data.front())
    if (obs.cloud_->height == 0 || obs.cloud_->width == 0) {
      continue;
    }

    double sq_obstacle_range = obs.obstacle_max_range_ * obs.obstacle_max_range_;

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*obs.cloud_, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*obs.cloud_, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*obs.cloud_, "z");
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z)
    {
      // if the obstacle is too high or too far away from the robot we won't add it
      if (*it_z > max_obstacle_height_) {
        continue;
      }

      // compute the squared distance from the hitpoint to the pointcloud's origin
      double sq_dist = (*it_x - obs.origin_.x) * (*it_x - obs.origin_.x) +
        (*it_y - obs.origin_.y) * (*it_y - obs.origin_.y) +
        (*it_z - obs.origin_.z) * (*it_z - obs.origin_.z);

      // if the point is far enough away... we won't consider it
      if (sq_dist >= sq_obstacle_range) {
        continue;
      }

      // now we need to compute the map coordinates for the observation
      unsigned int mx, my, mz;
      if (*it_z < origin_z_) {
        if (!worldToMap3D(*it_x, *it_y, origin_z_, mx, my, mz)) {
          continue;
        }
      } else if (!worldToMap3D(*it_x, *it_y, *it_z, mx, my, mz)) {
        continue;
      }

      // mark the cell in the voxel grid and check if we should also mark it in the costmap
      markUpdated(mx, my, mx, my);
      if (voxel_grid_.markVoxelInMap(mx, my, mz, mark_threshold_)) {
        unsigned int index = getIndex(mx, my);

        costmap_[index] = LETHAL_OBSTACLE;
        touch(static_cast<double>(*it_x),
          static_cast<double>(*it_y), min_x, min_y, max_x, max_y);
      }
    }
  }

  if (publish_voxel_) {
    nav2_msgs::msg::VoxelGrid grid_msg;
    unsigned int size = voxel_grid_.sizeX() * voxel_grid_.sizeY();
    grid_msg.size_x = voxel_grid_.sizeX();
    grid_msg.size_y = voxel_grid_.sizeY();
    grid_msg.size_z = voxel_grid_.sizeZ();
    grid_msg.data.resize(size);
    memcpy(&grid_msg.data[0], voxel_grid_.getData(), size * sizeof(unsigned int));

    grid_msg.origin.x = origin_x_;
    grid_msg.origin.y = origin_y_;
    grid_msg.origin.z = origin_z_;

    grid_msg.resolutions.x = resolution_;
    grid_msg.resolutions.y = resolution_;
    grid_msg.resolutions.z = z_resolution_;
    grid_msg.header.frame_id = global_frame_;
    grid_msg.header.stamp = clock_->now();
    voxel_pub_->publish(grid_msg);
  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void NonPersistentVoxelLayer::updateOrigin(
  double new_origin_x, double new_origin_y)
{
  // project the new origin into the grid
  int cell_ox, cell_oy;
  cell_ox = static_cast<int>((new_origin_x - origin_x_) / resolution_);
  cell_oy = static_cast<int>((new_origin_y - origin_y_) / resolution_);

  // update the origin with the appropriate world coordinates
  origin_x_ = origin_x_ + cell_ox * resolution_;
  origin_y_ = origin_y_ + cell_oy * resolution_;
}

}  // namespace nav2_costmap_2d

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::NonPersistentVoxelLayer,
  nav2_costmap_2d::Layer)

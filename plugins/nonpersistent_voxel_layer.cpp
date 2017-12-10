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
#include <nonpersistent_voxel_layer/nonpersistent_voxel_layer.hpp>
#include <pluginlib/class_list_macros.h>
#include <pcl_conversions/pcl_conversions.h>

#define VOXEL_BITS 16
PLUGINLIB_EXPORT_CLASS(costmap_2d::NonPersistentVoxelLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

using costmap_2d::ObservationBuffer;
using costmap_2d::Observation;

namespace costmap_2d
{

void NonPersistentVoxelLayer::onInitialize()
{
  ObstacleLayer::onInitialize();
  ros::NodeHandle private_nh("~/" + name_);

  private_nh.param("publish_voxel_map", publish_voxel_, false);
  if (publish_voxel_)
    voxel_pub_ = private_nh.advertise < costmap_2d::VoxelGrid > ("voxel_grid", 1);
}

void NonPersistentVoxelLayer::setupDynamicReconfigure(ros::NodeHandle& nh)
{
  voxel_dsrv_ = new dynamic_reconfigure::Server<costmap_2d::NonPersistentVoxelPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::NonPersistentVoxelPluginConfig>::CallbackType cb = boost::bind(
      &NonPersistentVoxelLayer::reconfigureCB, this, _1, _2);
  voxel_dsrv_->setCallback(cb);
}

NonPersistentVoxelLayer::~NonPersistentVoxelLayer()
{
  if (voxel_dsrv_)
    delete voxel_dsrv_;
}

void NonPersistentVoxelLayer::reconfigureCB(costmap_2d::NonPersistentVoxelPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
  max_obstacle_height_ = config.max_obstacle_height;
  size_z_ = config.z_voxels;
  origin_z_ = config.origin_z;
  z_resolution_ = config.z_resolution;
  unknown_threshold_ = config.unknown_threshold + (VOXEL_BITS - size_z_);
  mark_threshold_ = config.mark_threshold;
  combination_method_ = config.combination_method;
  matchSize();
}

void NonPersistentVoxelLayer::matchSize()
{
  ObstacleLayer::matchSize();
  voxel_grid_.resize(size_x_, size_y_, size_z_);
  ROS_ASSERT(voxel_grid_.sizeX() == size_x_ && voxel_grid_.sizeY() == size_y_);
}

void NonPersistentVoxelLayer::reset()
{
  deactivate();
  resetMaps();
  voxel_grid_.reset();
  activate();
}

void NonPersistentVoxelLayer::resetMaps()
{
  Costmap2D::resetMaps();
  voxel_grid_.reset();
}

void NonPersistentVoxelLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                       double* min_y, double* max_x, double* max_y)
{
  // update origin information for rolling costmap publication
  if (rolling_window_)
  {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  // reset maps each iteration
  resetMaps();

  // if not enabled, stop here
  if (!enabled_)
    return;

  // get the maximum sized window required to operate
  useExtraBounds(min_x, min_y, max_x, max_y);

  // get the marking observations
  bool current = true;
  std::vector<Observation> observations;
  current = getMarkingObservations(observations) && current;

  // update the global current status
  current_ = current;

  // place the new obstacles into a priority queue... each with a priority of zero to begin with
  for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it)
  {
    const Observation& obs = *it;

    const pcl::PointCloud<pcl::PointXYZ>& cloud = *(obs.cloud_);

    double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;

    for (unsigned int i = 0; i < cloud.points.size(); ++i)
    {
      // if the obstacle is too high or too far away from the robot we won't add it
      if (cloud.points[i].z > max_obstacle_height_)
        continue;

      // compute the squared distance from the hitpoint to the pointcloud's origin
      double sq_dist = (cloud.points[i].x - obs.origin_.x) * (cloud.points[i].x - obs.origin_.x)
          + (cloud.points[i].y - obs.origin_.y) * (cloud.points[i].y - obs.origin_.y)
          + (cloud.points[i].z - obs.origin_.z) * (cloud.points[i].z - obs.origin_.z);

      // if the point is far enough away... we won't consider it
      if (sq_dist >= sq_obstacle_range)
        continue;

      // now we need to compute the map coordinates for the observation
      unsigned int mx, my, mz;
      if (cloud.points[i].z < origin_z_)
      {
        if (!worldToMap3D(cloud.points[i].x, cloud.points[i].y, origin_z_, mx, my, mz))
          continue;
      }
      else if (!worldToMap3D(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z, mx, my, mz))
      {
        continue;
      }

      // mark the cell in the voxel grid and check if we should also mark it in the costmap
      if (voxel_grid_.markVoxelInMap(mx, my, mz, mark_threshold_))
      {
        unsigned int index = getIndex(mx, my);

        costmap_[index] = LETHAL_OBSTACLE;
        touch((double)cloud.points[i].x, (double)cloud.points[i].y, min_x, min_y, max_x, max_y);
      }
    }
  }

  if (publish_voxel_)
  {
    costmap_2d::VoxelGrid grid_msg;
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
    grid_msg.header.stamp = ros::Time::now();
    voxel_pub_.publish(grid_msg);
  }

  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void NonPersistentVoxelLayer::updateOrigin(double new_origin_x, double new_origin_y)
{
  // project the new origin into the grid
  int cell_ox, cell_oy;
  cell_ox = int((new_origin_x - origin_x_) / resolution_);
  cell_oy = int((new_origin_y - origin_y_) / resolution_);

  // update the origin with the appropriate world coordinates
  origin_x_ = origin_x_ + cell_ox * resolution_;
  origin_y_ = origin_y_ + cell_oy * resolution_;
}

} // namespace costmap_2d
# nonpersistent_voxel_layer [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__navigation__ubuntu_xenial_amd64__binary)](http://build.ros.org/job/Kdev__nonpersistent_voxel_layer__ubuntu_xenial_amd64/2/)
ROS drop in replacement to the voxel layer which does not persist readings through iterations when ray tracing or map maintenance is undesirable.

Created in response to need for a rolling local costmap layer to not persist readings due to a specific sensor being used. After looking through the community, it seems several people on ros answers have asked for a similar tool. This is to make that possible. This also helps with sensors like sonars, blob marking, radars, etc that aren't dense enough to clear effectively.  

Build and tested on ROS1 Kinetic, verified working in Indigo and Melodic

## Example Use

### in costmap commons
```
nonpersisting_obstacle_layer:
  enabled:              true
  track_unknown_space:  true
  max_obstacle_height:  1.8
  unknown_threshold:    15
  mark_threshold:       2
  combination_method:   1
  obstacle_range: 3.0
  origin_z: 0.
  z_resolution: 0.05
  z_voxels: 16
  publish_voxel_map: true
  observation_sources: rgbd
  rgbd:
    data_type: PointCloud2
    topic: camera/depth/points
    marking: true
    min_obstacle_height: 0.7
    max_obstacle_height: 1.7
```
### in list of plugins for local/global
```
plugins:
 - {name: nonpersisting_obstacle_layer, type: "costmap_2d/NonpersistentVoxelLayer"}
```

## parameters 
Parameters as the same as found in the voxel layer, except the clearing bits. See the Voxel Layer API. The above example is a good minimum working example

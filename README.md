# nonpersistent_voxel_layer

<a href="https://www.buymeacoffee.com/stevemacenski" target="_blank"><img src="https://www.buymeacoffee.com/assets/img/custom_images/orange_img.png" alt="Buy Me A Coffee" style="height: 41px !important;width: 174px !important;box-shadow: 0px 3px 2px 0px rgba(190, 190, 190, 0.5) !important;-webkit-box-shadow: 0px 3px 2px 0px rgba(190, 190, 190, 0.5) !important;" ></a>

ROS2 drop in replacement to the voxel layer which does not persist readings through iterations when ray tracing or map maintenance is undesirable.

Bloom released, install via 

```
sudo apt-get update && sudo apt-get install ros-eloquent-nonpersistent-voxel-layer
```

Created in response to need for a rolling local costmap layer to not persist readings due to a specific sensor being used. After looking through the community, it seems several people on ros answers have asked for a similar tool. This is to make that possible. This also helps with sensors like sonars, blob marking, radars, etc that aren't dense enough to clear effectively.  

http://wiki.ros.org/nonpersistent_voxel_grid

## Example Use

### in costmap commons
```
    global_costmap:
      global_costmap:
        ros__parameters:
          nonpersisting_obstacle_layer:
            plugin: nav2_costmap_2d/NonPersistentVoxelLayer
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
    global_costmap:
      global_costmap:
        ros__parameters:
          use_sim_time: True
          plugin_names: ["static_layer", "nonpersisting_obstacle_layer"]
```

## parameters 
Parameters as the same as found in the voxel layer, except the clearing bits. See the Voxel Layer API. The above example is a good minimum working example

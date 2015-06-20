# fbet
This is a quick start for 3D-FBET

1.compile:

copy fbet to catkin_ws/src/

then 

$catkin_make

2.input:

you need sensor_msgs/PointCloud2 data from topic /cloud_in

Incoming 3D point cloud for scan integration. 

You need to remap this topic to your sensor data and provide a tf transform between the sensor data and the static map frame.

that is to say, you need a slam

3.launch:

better to make a launch file like bellow:
<launch>

  <master auto="start"/>
  
  <include file="$(find openni2_launch)/launch/openni2.launch"/>
  
  <include file="$(find slam)/launch/slam.launch" > 
  
  </include>

  <node name="fbet_node" pkg="fbet" type="fbet_node">
  
    <remap from="/cloud_in"       to="/voxel_cloud"/>
    
    <param name="resolution" type="double" value="0.25"/>
    
  </node>

</launch>

p.s: replace the "slam" part by your own

4.output:

3D-fbet publish topics:

type: Marker; name: frontier_goal_marker

type: MarkerArray;

name: occupied_cells_vis_array

      free_cells_vis_array
      
      frontier_cells_vis_array

use rviz to visualize

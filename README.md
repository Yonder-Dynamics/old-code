# 2018_URC

## RGBD SLAM
RGBDSLAM
http://wiki.ros.org/rgbdslam

## Realsense R200 to Octomap
src/realSense_to_Octo.py

### Description
Takes in PointCloud2 data received from topic, ```/camera/depth/points```, and publishes that same data to topic, ```cloud_in```. In order to send PointCloud2 data from realsense_camera node to octomap_server node.

### Software Dependencies
```
sudo apt-get install ros-kinetic-realsense-camera ros-kinetic-octomap-server ros-kinetic-octomap-rviz-plugins
```
[ros-kinetic-realsense-camera](http://wiki.ros.org/realsense_camera)     -  Get PointCloud2 from depth camera  
[ros-kinetic-octomap-server](http://wiki.ros.org/octomap_server)       -  Generating octomap from PointCloud2? ('?' means I'm unsure)  
ros-kinetic-octomap-rviz-plugins -  Rviz plugins to visualize octomap?  

### Execution

#### Visualizing PointCloud
In terminal 1:
```
roslaunch realsense_camera r200_nodelet_rgbd.launch 
```
In terminal 2:
```
rviz
```

In rviz:
1. Set 'Fixed Frame' to depth_camera_ (Note: unsure of exact name)
2. Click 'Add'
3. Click on the 'By topic'
4. Choose '/camera/depth/points' and PointCloud2

After this a 3-D point cloud should appear showing what the realsense camera sees.

#### Visualizing Octomap
In terminal 1:
```bash
roslaunch realsense_camera r200_nodelet_rgbd.launch 
```

In terminal 2:
```
python src/realSense_to_Octo.py
```

In terminal 3 (I'm not sure if this works or if I am doing this right):
```
roslaunch octomap_server octomap_mapping.launch 
```
In terminal 4:
```
rviz
```

In rviz:
??? (I don't know exactly how to get this to work.)

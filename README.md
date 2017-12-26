# 3D_Object_Modeling

## Installation

### package

```
$ sudo apt-get install ros-kinetic-pcl-ros
$ sudo apt-get install ros-kinetic-pcl-conversions
$ sudo apt-get install pcl-tools 
```

Note:
do realsense tutorial first

## Function

### pre-work

```
$ cd 3D_Object_Modeling/catkin_ws
$ source environment.sh
$ source set_ros_master.sh
```
Note:
Do it everytime as you execute any file


### Open Camera

```
$ roslaunch realsense_camera sr300_nodelet_rgbd.launch 
```

### Detect Apriltags

```
$ roslaunch demos apriltags_demos.launch
```

### Save PCD files

```
$ rosrun pcd create_pcd input:= [pcl_topic]
```

### View The PointCloud in PCL_viewer

```
$ pcl_viewer [PCD_file_name]
```

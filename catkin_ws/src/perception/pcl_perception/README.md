# Velodyne Perception

In charge: David Chen

## How to run pointcloud perception

### Without pointcloud classification
```
$ roslaunch velodyne_perception cluster_no_preprocess.launch classify:=false

- It will open "cluster_no_preprocess" & "object_map.py"
```
Publish:

|Topic | Classification | Coordinate |
|-------			|--------	|--------	|
|/obj_list			|No			|velodyne	|
|/obj_list/map		|No			|odom		|

### With pointcloud classification
Remember to follow README.md in "catkin_ws/src/classification/object_classification/" first

https://github.com/RobotX-NCTU/robotx_nctu/blob/master/catkin_ws/src/classification/object_classification/README.md
```
$ roslaunch velodyne_perception cluster_no_preprocess.launch classify:=true

- You also need to open another terminal for operating pcl classification

$ rosrun object_classification classify_rot.py
```
Publish:

|Topic | Classification | Coordinate |
|-------			|--------	|--------	|
|/obj_list			|No			|velodyne	|
|/obj_list/classify	|Yes		|velodyne	|
|/obj_list/map		|Yes		|odom		|
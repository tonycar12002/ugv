# UGV
The repo stores the code, algorithms...etc except the gazebo part

## Docker
- ROS kinetic 
- Ubuntu 16.04
- PCL (Point Cloud Library)
- OpenCV 3.3.1
- ZED SDK 2.7.1
- CUDA 9.0

```
$ docker pull argnctu/subt_16 
$ ./docker_run -u
or 
open same docker
$ ./docker_run -u -s
```

## Hardware

|Name | Type |
|-------		|--------					|
|Vehicle		|Husky						|
|IMU			|Sbg						|
|GPS			|Vector	V103 GPS			|
|LIDAR 			|Velodyne VLP-16			|
|RGBD Camera	|Realsense D435				|
|Stereo Camera	|ZED and ZED-Mini			|

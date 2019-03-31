# UGV
Husky waypoint navagation and avoid obstacles
![husky](https://github.com/tonycar12002/ugv/tree/master/images/husky.jpg =500x400)
## Docker
- ROS kinetic 
- Ubuntu 16.04
- PCL (Point Cloud Library)
- OpenCV 3.3.1
- ZED SDK 2.7.1
- CUDA 9.0

```
$ docker pull argnctu/subt_16 or docker build -t ugv -f Dockerfile .
$ ./docker_run -u
or 
open same docker
$ ./docker_run -u -s
```
## How to run
* [Husky Joystick Control](https://github.com/tonycar12002/ugv/wiki/Husky_Joystick_Control)
* [Husky Waypoint Navagation](https://github.com/tonycar12002/ugv/wiki/Waypoint_Navagation)

## Videos
[waypt navagation](https://www.youtube.com/watch?v=ns-YryCcy-s)

## Hardware

|Name | Type |
|-------		|--------					|
|Vehicle		|Husky						|
|IMU			|Sbg						|
|GPS			|Vector	V103 GPS			|
|LIDAR 			|Velodyne VLP-16			|
|RGBD Camera	|Realsense D435				|
|Stereo Camera	|ZED and ZED-Mini			|

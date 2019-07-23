# LIO-mapping
## A Tightly Coupled 3D Lidar and Inertial Odometry and Mapping Approach
#### Authors: Haoyang Ye, Yuying Chen, and Ming Liu from [RAM-LAB](https://ram-lab.com/).
[[Paper]](https://arxiv.org/abs/1904.06993), [[Project]](https://sites.google.com/view/lio-mapping), [[Bib]](https://ram-lab.com/papers/2019/icra_2019_ye.bib). ICRA 2019.
## Table of Contents
* [Demo Results](#demo-results)
* [Prerequisites](#prerequisites)
* [Build](#build)
* [Examples](#examples)
* [Docker](#docker)
* [Credits](#credits)
* [Licence](#licence)

## Demo Results
<a href="https://youtu.be/4bGR_QJZZFw" target="_blank"><img src="https://ram-lab.com/file/hyye/lio-mapping-gt.gif" alt="lio-mapping-gt" height="200"></a>
<a href="https://youtu.be/4bGR_QJZZFw" target="_blank"><img src="https://ram-lab.com/file/hyye/lio-mapping-indoor.png" alt="lio-mapping-indoor" height="200"></a>

Video: [[More indoor and outdoor tests]](https://ram-lab.com/file/hyye/lio-mapping.mp4).

## Prerequisites
See [Dockerfile](docker/Dockerfile) as a reference:
1. [ROS](http://wiki.ros.org/melodic/Installation) with Ubuntu 18.04 or Ubuntu 16.04.
2. [Ceres-solver](http://ceres-solver.org/installation.html#linux).
3. [PCL](http://www.pointclouds.org/downloads/), the default version accompanying by ROS.
4. [OpenCV](https://docs.opencv.org/master/d7/d9f/tutorial_linux_install.html), the default version accompanying by ROS.

## Build
1. `git clone git@github.com:hyye/lio-mapping.git` into the `src` folder of your catkin workspace.
2. `catkin build -DCMAKE_BUILD_TYPE=Release lio` or `catkin_make -DCMAKE_BUILD_TYPE=Release`.

## Examples
Some [sample data](https://drive.google.com/drive/folders/1dPy667dAnJy9wgXmlnRgQZxQF_ESuve3).
1. `source devel/setup.zsh`, or `setup.bash` if your prefer `bash`.
2. `roslaunch lio test_indoor.launch &`.
3. `roslaunch lio map_4D_indoor.launch &`.
4. `rosbag play fast1.bag`.

## Docker
Try it out using docker:
1. Run `docker/build_docker.sh`.
2. Run `docker/run_docker.sh`.
3. Run `rosbag play fast1.bag`, in your host machine or in the running container.

Note: Visualization (rviz) can run in the running container with [nvidia-docker](https://github.com/NVIDIA/nvidia-docker). The [Dockerfile](docker/Dockerfile) is compatible with [nvidia-docker 2.0](https://github.com/nvidia/nvidia-docker/wiki/Installation-(version-2.0)); [1.Dockerfile](docker/1.Dockerfile) with [nvidia-docker 1.0](https://github.com/nvidia/nvidia-docker/wiki/Installation-(version-1.0)).

## Credits
The feature extraction, lidar-only odometry and baseline implemented were heavily derived or taken from the original [LOAM](http://wiki.ros.org/loam_velodyne) and its [modified version](https://github.com/laboshinl/loam_velodyne) (the point_processor in our project), and one of the initialization methods and the optimization pipeline from [VINS-mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono). The copyright headers are retained for the relevant files.

## Licence
The source code is released under [GPL-3.0](https://www.gnu.org/licenses/).

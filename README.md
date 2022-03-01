# visual_livox
This is a repository for Visual-Livox LiDARS mapping, which is a reproduced version of VLOAM described in the following paper:
 **Zhang J , Singh S . Visual-lidar odometry and mapping: low-drift, robust, and fast ICRA, 2015**
 In this version, livox LIDAR is utilized, but it can be replaced by LOAM or any other LiDAR SLAM framework. 
 **Debugging, and more code is coming**

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2. **PCL && Eigen && openCV**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).
Follow [Eigen Installation](http://eigen.tuxfamily.org/index.php?title=Main_Page).
Follow [openCV Installation](https://opencv.org/releases/).

### 1.3. **livox_sdk**
Follow [Livox_SDK Installation](https://github.com/Livox-SDK/Livox-SDK).

### 1.4. **livox_ros_driver**
Follow [livox_ros_driver Installation](https://github.com/Livox-SDK/livox_ros_driver).

## 2. Build
Clone the repository and catkin_make:

```
    cd ~/catkin_ws/src
    git clone https://github.com/bobocode/visual-livox.git
    cd ..
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```
## 3. Test Package
Run the launch file:

```
```
## 5.Acknowledgments
Thanks for livox_mapping, [livox_mapping](https://github.com/Livox-SDK/livox_mapping) and VLOAM.

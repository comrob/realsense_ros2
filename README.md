[//]: # (Image References)

[image1]: imgs/rs-viewer.png "rs-viewer"
[image2]: imgs/rviz.gif "rviz"
[image3]: imgs/cartographer.png "cartographer"
[image4]: imgs/cartographer.gif "2D"
[image5]: imgs/rtabmap.gif "rtabmap"
[image6]: imgs/rtabmap.png "rtabmap"





# realsense_ros2
ROS 2 wrapper targeted for the Intel RealSense T265 tracking camera on ROS 2 Jazzy.

This wrapper's implementation is specially developed with the objective of running it in Nvidia's Jetson Nano, however it should also work on any other platform running Ubuntu 18.04 and 20.04.

By running this wrapper you would be able to obtain:

* Pose data from the realsense t265 tracking camera

Legacy/optional support in this repository also includes D435 depth camera nodes.

**Tested on Jetson Nano:
L4T 32.4.3 [ JetPack 4.4 ]
   Ubuntu 18.04.4 LTS
   Kernel Version: 4.9.140-tegra 
   ROS 2 Eloquent**

**Tested on Ubuntu 20.04 amd64:
ROS2 Foxy**

## Requirements
* ROS2 jazzy
* Ubuntu 24.04

## Installation
1. Install librealsense2 of the version 2.50.0 from sources:

    ```bash
    # dependencies
    sudo apt update && sudo apt upgrade
    sudo apt install git wget cmake build-essential libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

    # download librealsense2 version 2.50.0
    git clone https://github.com/realsenseai/librealsense
    cd librealsense
    git checkout v2.50.0 

    # build and install
    mkdir build && cd build
    cmake .. -DFORCE_RSUSB_BACKEND=ON -DBUILD_EXAMPLES=true -DBUILD_WITH_CUDA=OFF
    make -j$(nproc)
    sudo make install
    ```

2. Connect your cameras and check they are working propperly by openning a new terminal and typing:

    ```bash
    realsense-viewer
    ```
    You should be able to add the t265 and d435 cameras and view their data streams simultaneously.

    ![rs-viewer][image1]

3. Create a ROS 2 workspace, clone this repository inside and build: 
    ```bash
    mkdir -p dev_ws/src
    cd dev_ws/src
    git clone https://github.com/jdgalviss/realsense_ros2.git
    cd ..
    colcon build
    ```
4. Source your workspace
    ```bash
    . install/setup.bash
    ```

## Run
### T265 tracking camera (primary target)
Run the node with the following command:

```bash
ros2 run realsense_ros2 rs_t265_node
```

To also publish fisheye images from T265, run:

```bash
ros2 run realsense_ros2 rs_t265_node --ros-args -p publish_fisheye:=true
```

When `publish_fisheye` is enabled, static transforms from `t265_frame` to
`t265_fisheye1_frame` and `t265_fisheye2_frame` are also published using
RealSense API extrinsics.

### D435 depth camera only
Run the node with the following command:

```bash
ros2 run realsense_ros2 rs_d435_node --ros-args -p is_color:=true -p publish_depth:=true -p fps:=30
```

* When the *is_color* parameter is set to true, the color image from the depth camera is aligned with the pointcloud.
* When the *publish_depth* parameter is set to true, the depth image is published.
* When the *publish_pointcloud* parameter is set to true, the 3D pointcloud is published.
* The *fps* parameter is used to modify the rate at which the node publishes the pointcloud and the depth image.


### T265 tracking and D435 depth cameras simultaneously
In one terminal, launch the two cameras:
```bash
ros2 launch realsense_ros2 realsense_launch.py
```
![rviz][image2]

### T265 tracking and D435 depth cameras simultaneously with SLAM-TOOLBOX 2D SLAM
In one terminal, launch the 2 cameras:
```bash
ros2 launch realsense_ros2 realsense_2d_slam_launch.py
```

In another terminal launch slam-toolbox:
```bash
ros2 launch realsense_ros2 online_async_launch.py
```

This requires to have ROS Slam-toolbox installed (apt-get install ros-<distro>-slam-toolbox)

![cartographer][image4]

### T265 tracking and D435 depth cameras simultaneously with rtabmap 3D SLAM
First, Download and Install rtabmap and rtabmap_ros following these [instructions](https://github.com/introlab/rtabmap_ros/tree/ros2#rtabmap_ros) in the branch **ros2**.

In one terminal, launch the two cameras and rtabmap ros (make sure that you source the workspace where rtabmap_ros was built):
```bash
ros2 launch realsense_ros2 slam_rtabmap_launch.py
```
![rtabmap][image5]
![rtabmap2][image6]


## Published topics

### rs_t265_node
* rs_t265/odom [nav_msgs/Odometry]: Pose and speeds of the t265 tracking camera.
* rs_t265/imu [sensor_msgs/Imu]: Imu data.
* rs_t265/fisheye1/image_raw [sensor_msgs/Image]: Left fisheye image (only published if *publish_fisheye* is set to true).
* rs_t265/fisheye2/image_raw [sensor_msgs/Image]: Right fisheye image (only published if *publish_fisheye* is set to true).
* rs_t265/fisheye1/camera_info [sensor_msgs/CameraInfo]: Calibration/intrinsics for left fisheye (only published if *publish_fisheye* is set to true).
* rs_t265/fisheye2/camera_info [sensor_msgs/CameraInfo]: Calibration/intrinsics for right fisheye (only published if *publish_fisheye* is set to true).
* tf


### rs_d435_node

* rs_d435/point_cloud [sensor_msgs/PointCloud2]: Pointcloud from d435 depth camera.
* rs_d435/aligned_depth/image_raw [sensor_msgs/Image]: Depth Image from d435 depth camera (only published if *publish_depth* parameter is set to true ).
* rs_d435/image_raw [sensor_msgs/Image]: Raw Color images


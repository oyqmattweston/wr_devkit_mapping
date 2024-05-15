# WR Devkit Mapping

![default workflow](https://github.com/westonrobot/wr_devkit_mapping/actions/workflows/default.yml/badge.svg?branch=main)

This repository provides **a sample setup** for using LIO-SAM to do 3D mapping with the mobile robot development kit from Weston Robot. It mainly serves as a reference to help you get started with your application development as quickly as possible. If you have questions regarding the LIO-SAM algorithm itself, please refer to the original [GitHub repository by the author](https://github.com/TixiaoShan/LIO-SAM) and [relevant research papers](https://github.com/TixiaoShan/LIO-SAM?tab=readme-ov-file#paper). 

## Requirements

The following hardware configurations are supported: 

* WR Devkit V1.0
  * With Livox Mid-360 Lidar

The onboard computer with the devkit should have been configured with the following software environment:

* Ubuntu 22.04 
* ROS Humble

## Installation

Please refer to the [CI build script](.github/workflows/default.yml) for the most complete and up-to-date installation steps.

* Install the Weston Robot Platform SDK (wrp-sdk)

    ```bash
    $ sudo install -m 0755 -d /etc/apt/keyrings
    $ curl -fsSL http://deb.westonrobot.net/signing.key | sudo gpg --dearmor -o /etc/apt/keyrings/weston-robot.gpg
    $ sudo chmod a+r /etc/apt/keyrings/weston-robot.gpg

    $ echo \
        "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/weston-robot.gpg] http://deb.westonrobot.net/$(lsb_release -cs) $(lsb_release -cs) main" | \
        sudo tee /etc/apt/sources.list.d/weston-robot.list > /dev/null
    $ sudo apt-get update

    $ sudo apt-get install wrp-sdk
    ```

    Please refer to [this page](https://docs.westonrobot.net/software/installation_guide.html) for more details of the installation steps.

* Install Livox SDK2 (if you have the devkit variant with the Livox Mid-360 Lidar)

    ```bash
    $ cd ~
    $ git clone https://github.com/Livox-SDK/Livox-SDK2.git
    $ cd Livox-SDK2
    $ mkdir build && cd build && cmake .. && make
    $ sudo make install
    ```

    Note: you can build and install the Livox-SDK2 at your preferred places other than "~/Livox-SDK2". And you can optionally remove the "Livox-SDK2" folder after installation.

* Install vcstool **(Make sure you have ros2 installed first)**
    ```bash
    $ sudo apt install python3-vcstool
    ```

* Install GTSAM

    ```bash
    $ sudo add-apt-repository ppa:borglab/gtsam-release-4.1
    $ sudo apt install libgtsam-dev libgtsam-unstable-dev
    ```

* Install additional ROS packages

    ```bash
    $ source /opt/ros/humble/setup.bash
    $ sudo apt install ros-$ROS_DISTRO-perception-pcl \
  	   ros-$ROS_DISTRO-pcl-msgs \
  	   ros-$ROS_DISTRO-vision-opencv \
  	   ros-$ROS_DISTRO-xacro
    ```

* Import the ROS packages into the workspace and build

    ```bash
    $ cd <your-workspace>
    $ git clone https://github.com/westonrobot/wr_devkit_mapping.git
    $ cd wr_devkit_mapping
    # for Livox Mid-360 setup
    $ vcs import src < ./livox_mapping.repos

    $ source /opt/ros/humble/setup.bash
    $ colcon build --symlink-install
    ```

    The build process should finish without any errors.

## Start a mapping session

On the robot, launch the mapping nodes:

```bash
$ cd <your-workspace>/wr_devkit_mapping
$ source /opt/ros/humble.bash
$ source install/setup.bash
$ ros2 launch sample_launch livox_mapping.launch.py
```

If you want to check the mapping result, you can run rviz on your computer (make sure your computer is connected to the robot network first)

```bash
$ ros2 run rviz2 rviz2
```

You can clone this repository to your computer and load the rviz configuration located at "src/sample_launch/config/rviz2.rviz".

## Notes
1. Depending on the specific hardware configurations, you may need to modify the sample launch files and configuration files to adapt to your setup.  
   Take note of the below in particluar:
   1. IP addresses of the lidar and the data ports it uses. (Configuration files used can be inferred from the [launch file](./src/sample_launch/launch/livox_mapping.launch.py)).
   2. Device path of the IMU in the [launch file](./src/sample_launch/launch/livox_mapping.launch.py).
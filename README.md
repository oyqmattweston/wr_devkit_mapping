# WR Devkit Mapping

![default workflow](https://github.com/westonrobot/wr_devkit_mapping/actions/workflows/default.yml/badge.svg?branch=main)

This repository provides a reference setup for using LIO-SAM to do 3D mapping with the mobile robot development kit from Weston Robot.

## Requirements

The following hardware configurations are supported: 

* WR Devkit V1.0
  * With Livox Mid-360 Lidar

The onboard computer with the devkit should have been configured with the following software environment:

* Ubuntu 22.04 
* ROS Humble

## Installation

* Install the Weston Robot Platform SDK (wrp-sdk)

    ```bash
    $ sudo install -m 0755 -d /etc/apt/keyrings
    $ curl -fsSL http://deb.westonrobot.net/signing.key | sudo gpg --dearmor -o /etc/apt/keyrings/weston-robot.gpg
    $ sudo chmod a+r /etc/apt/keyrings/weston-robot.gpg

    $ echo \
        "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/weston-robot.gpg] http://deb.westonrobot.net/$(lsb_release -cs) $(lsb_release -cs) main" | \
        sudo tee /etc/apt/sources.list.d/weston-robot.list > /dev/null
    $ sudo apt-get update

    $ sudo apt-get install wrp_sdk
    ```

    Please refer to [this page](https://docs.westonrobot.net/software/installation_guide.html) for more details of the installation steps.

* Install Livox SDK (if you have the devkit variant with the Livox Mid-360 Lidar)

    ```bash
    $ cd ~
    $ git clone https://github.com/Livox-SDK/Livox-SDK.git
    $ cd Livox-SDK
    $ cd build && cmake .. && make
    $ sudo make install
    ```

    Note: you can build and install the Livox-SDK at your preferred places other than "~/Livox-SDK". And you can optionally remove the "Livox-SDK" folder after installation.

* Import the ROS packages into the workspace and build

    ```bash
    $ cd <your-workspace>/wr_devkit_mapping
    # for Livox Mid-360 setup
    $ vcs import src < ./livox_mapping.repos

    $ source /opt/ros/humble/setup.bash
    $ colcon build --symlink-install
    ```

    The build process should finish without any errors.
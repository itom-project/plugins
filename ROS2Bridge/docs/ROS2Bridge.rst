===================
 PCOCamera
===================

=============== ========================================================================================================
**Summary**:    :pluginsummary:`PCOCamera`
**Type**:       :plugintype:`PCOCamera`
**License**:    :pluginlicense:`PCOCamera`
**Platforms**:  Windows
**Devices**:    PCO Cameras supported by the pco.sdk
**Author**:     :pluginauthor:`PCOCamera`
=============== ========================================================================================================

Overview
========

The PCOCamera is a plugin to access PCO.XXXX, e.g. PCO.1300 or PCO.2000, with itom. It uses the SDK pco.sdk from PCO AG, Germany.
The plugin has mainly been developed and tested using the cameras PCO.1200s, PCO.1300, PCO.2000 and PCO.edge USB3.

The camera is always operated in a software trigger mode with a standard image size (no extended image size).

Initialization
==============

The following parameters are mandatory or optional for initializing an instance of this plugin:

    .. plugininitparams::
        :plugin: PCOCamera

Parameters
==========

These parameters are available and can be used to configure the **PCOCamera** instance. Many of them are directly initialized by the
parameters of the constructor. During the runtime of an instance, the value of these parameters is obtained by the method *getParam*, writeable
parameters can be changed using *setParam*.

**name**: {str}, read-only
    name of the plugin (*PCOCamera*)
**camera_name**: {str}, read-only
    device name of camera
**interface**: {str}, read-only
    name of the interface (eithernet, USB, cameralink...)
**x0**, **x1**: {int}
    first and last column (zero-based) of a software region of interest. The real camera image can be cropped to a region of interest, the
    corresponding horizontal boundaries are given by **x0** and **x1**, whereas the current width is obtained by **sizex**.
**y0**, **y1**: {int}
    the same than above but first and last row (related to the height of the ROI).
**sizex**, **sizey**: {int}, read-only
    width and height of the region of interest or the full camera size (default)
**bpp**: {int}
    bit depth, bits per pixel (usually not adjustable)
**temperatures**: {double}, read-only
    list containing the current CCD, camera and power supply temperatures in degree celsius
**coolingSetPointTemperature**: {int}
    set point for the CCD cooling control in degree celsius (only available if supported with this camera)
**IRSensitivity**: {bool} [0,1]
    enables (True, 1) or disables (False, 0) the IR sensitivity of the image sensor, parameter is set to read-only if not available for the specific camera
**pixelrate**: {int}
    Transfer pixelrate for data from the camera in MHz.
**conversionFactor**: {double}
    conversion factor in electrons/count
**binning**: {int}
    Horizontal and vertical binning. The value is obtained by *horizontal * 100 + vertical*. Therefore, no binning corresponds to 101. Some cameras accepts a linear range of binning values [1,2,...max], others only allow a binary range [1,2,4,8,..max]. If the binning is changed, the region of interest is adapted as well to a suitable value.
**gain**: {double}, read-only
    not available
**offset**: {double}, read-only
    not available

Most parameters not only have a minimum and maximum value but also a step size.

.. note::

    Please consider that the parameters defining the region of interest may change if the binning is changed, since an increased binning value decreases the available image size.

Compilation
============

Please be aware that this plugin is currently under development and used and tested on Ubuntu 24.04 Noble Numbat exclusively.
TO install ROS2 - Jazzy Jalisco, please follow the installation instructions from:
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html or follow the instructions listed below:

1. ROS2 Installation:

set locale:
    locale  # check for UTF-8
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    locale  # verify settings

Enable required repositories:
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

Install development tools:
    sudo apt update && sudo apt install ros-dev-tools

Install ROS 2:
    sudo apt update
    sudo apt upgrade
    sudo apt install ros-jazzy-desktop
    sudo apt install ros-jazzy-ros-base

2. Install rosapi
    sudo apt install ros-jazzy-rosapi

3. Setup environment

    source /opt/ros/jazzy/setup.bash

    Please be aware that the setup environment is not defined permanently and
    only valid for the command shell that calls the bash script. A more permanent
    solution is under investigation.


Changelog
==========

* itom setup 5.0.0: This plugin has been compiled using ROS2 - Jazzy Jalisco Distribution along with Ubuntu 24.04 Noble Numbat

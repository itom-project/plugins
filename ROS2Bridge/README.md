# ROS2Bridge

ROS2Bridge is a plugin designed to explore data exchange between ITOM and ROS2. It enables ITOM to create ROS2 nodes and interact with external ROS2 programs. This plugin currently validates data exchanges between ITOM and external ROS2 Programs and demonstrates how to exchange 2D Image Data between ITOM and external ROS2 Program.


## Table of Contents

- [Requirements](#requirements)
- [Installation](#installation)
  - [Extract ROS2 Packages](#extract-ros2-packages)
  - [Create a ROS2 Workspace](#create-a-ros2-workspace)
  - [Configuration and Build Instructions](#configuration-and-build-instructions)
- [ROS2Bridge Overview](#ros2bridge-overview)
- [image_exchange_pkg Overview](#image_exchange_pkg-overview)
  - [Client Creation](#client-creation)
  - [Image Request](#image-request)
  - [Service Server Response (ROS2Bridge)](#service-server-response-ros2bridge)
  - [Client Processing](#client-processing)
  - [ITOM Data Conversion](#itom-data-conversion)
- [Usage](#usage)
  - [Prepare the ROS2 Workspace](#1-prepare-the-ros2-workspace)
  - [Run the ROS2 Image Exchange Client](#2-run-the-ros2-image-exchange-client)
  - [Execute ITOM Commands](#3-execute-itom-commands)
- [Extending ROS2Bridge](#extending-ros2bridge)




## Requirements

Before using ROS2Bridge, ensure that the following packages and dependencies (as listed under "Locate the components required for ROS 2") are installed on your computer. Additionally, you must have ROS2 Humble installed.

Refer to the installation guide here:  
[ROS 2 Humble Installation (Ubuntu)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

The plugin depends on the following ROS2 packages:

- *rclcpp*: Core package to create ROS2 nodes.
- *rosapi_msgs*
- *std_msgs*
- *std_srvs*
- *example_interfaces*
- *sensor_msgs*
- *itom_ros2_test*: Used for message definitions.
- *cv_bridge*: Required to test 2D image data exchange.




## Installation

### Extract ROS2 Packages

In the *test* folder of ROS2Bridge, there are two ROS2 packages:

- **image_exchange_pkg**: Validates 2D data exchange between ITOM and external ROS2 programs.
- **itom_ros2_test**: Contains message definitions for 1D, 2D, ND, and point cloud data. This package is used by ROS2Bridge and can be extended for further development with ND and point cloud data.

### Create a ROS2 Workspace

Follow the ROS2 workspace creation instructions here:  
[Creating a ROS2 Workspace](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)  

This guide explains how to set up a new workspace and compile the extracted packages.

---

### Configuration and Build Instructions

To allow your plugin to interact with the ROS2 system, add the following lines in your **CMakeLists.txt**:

```cmake
find_package(rclcpp REQUIRED)
find_package(rosapi_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(itom_ros2_test REQUIRED HINTS)
find_package(cv_bridge REQUIRED)
```
include these packages in the *ament_target_dependencies* section:
```cmake
ament_target_dependencies(${target_name}
  rclcpp
  rosapi_msgs
  std_msgs
  example_interfaces
  itom_ros2_test
  sensor_msgs
  cv_bridge
)
```


## ROS2Bridge Overview

ROS2Bridge is a plugin integrated into ITOM that facilitates data exchange between ITOM and external ROS2 programs. Key features include:

### Data Exchange
- Allows bidirectional data exchange between ITOM and ROS2 (both sending and receiving data).
- Supports conversion between ROS2 Image messages and ITOM's `ito::DataObject` (via OpenCV and cv_bridge).

### Node and Service Server
Upon creation of an instance, ROS2Bridge:
- Creates a ROS2 node named `ros2_itom_bridge`.
- Sets up a service server called `image_exchange` to handle service requests.



## image_exchange_pkg Overview

The `image_exchange_pkg` is a simple ROS2 package used to validate data exchange with ITOM. Its workflow is as follows:

### Client Creation
The package creates a ROS2 client for the `image_exchange` service provided by ROS2Bridge. It waits until the ROS2Bridge service server is active.

### Image Request
In the callback function `send_image_request`, the package reads the image file `Software_solution.jpg` from the package's `data` directory.  
It converts the image into a `cv::Mat` object, then uses `cv_bridge` to convert the `cv::Mat` to a ROS2 `Image` message and sends the request to ROS2Bridge.

### Service Server Response (ROS2Bridge)
The service server callback `handle_image_request` in ROS2Bridge receives the ROS2 `Image` message.  
It converts the ROS2 `Image` message to a `cv::Mat`, rotates the image by **90 degrees** using OpenCV, converts the rotated image back to a ROS2 `Image` message, and sends it back as a reply.

### Client Processing
Once the client in `image_exchange_pkg` receives the response, it converts the ROS2 `Image` message back to a `cv::Mat` and saves the image as `response_image.jpg`.

### ITOM Data Conversion
Finally, when ROS2Bridge's `GetVal()` function is called, it uses `setCvMatToDataObject` to convert the image (received from `image_exchange_pkg`) into ITOM's `ito::DataObject`.



## Usage

### 1. Prepare the ROS2 Workspace
Ensure that you have extracted the `image_exchange_pkg` and `itom_ros2_test` packages, created a ROS2 workspace, and built the workspace.

### 2. Run the ROS2 Image Exchange Client
Open a terminal in the `image_exchange_pkg` workspace and run:

```bash
source install/setup.bash
ros2 run image_exchange_pkg image_exchange_client
```

### 3.  Execute ITOM Commands

In ITOM, input the following commands in sequence:

```ypagon
test_conversion = dataIO("ROS2Bridge", ServiceServerName = "whatever")
data = dataObject()
test_conversion.getVal(data)
```

 You should see logs in both the ITOM terminal and the terminal running the `image_exchange_pkg` client.  In ITOM, click on `Global Variables` on the top-right, and you will see the image displayed as an ITOM `ito::DataObject`.


## Extending ROS2Bridge

Users can extend ROS2Bridge to interact with other ROS2 packages. For example, to add MoveIt! support:

- Add `find_package(moveit_core REQUIRED)` in your **CMakeLists.txt**.
- Append `moveit_core` to the `ament_target_dependencies` list.

This modular approach allows you to enhance the functionality of ROS2Bridge for various ROS2-based applications.



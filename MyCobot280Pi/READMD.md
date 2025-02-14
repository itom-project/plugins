# MyCobot280Pi Plugin

MyCobot280Pi is a plugin integrated into ITOM that provides a **UI-based control panel (Widget)** for managing the **MyCobot280Pi** robotic arm. The plugin communicates with an external **ROS2 program** via **Socket** to send movement commands. However, it currently acts as a **frontend**, transmitting UI commands to the external ROS2 program, which then controls the MyCobot280Pi.

The **backend control programs** are located in the `test` folder, specifically:
- `remote_control_server_test.py`
- `arm_control_service.py`

To **fully integrate** the backend functionality into the MyCobot280Pi plugin, follow the approach outlined below.

---

## Prerequisites

Before integrating the backend into the plugin, ensure the following components are installed:

1. **Install `mycobot_ros`** on your system by following this repository:  
   [mycobot_ros GitHub](https://github.com/elephantrobotics/mycobot_ros)

2. **Install MoveIt 2** for ROS2:  
   [MoveIt 2 Installation Guide](https://moveit.ai/install-moveit2/binary/)

3. **Convert Backend Scripts to C++**:  
   - Convert `arm_control_service.py` and `remote_control_server_test.py` into **C++ equivalents**.
   - Identify the required ROS2 packages, such as:
     - `rclcpp`
     - `moveit_core`
     - `moveit_ros_planning_interface`
     - `moveit_ros_planning`
     - `mycobot_280pi_moveit.srv`
     - `sensor_msgs.msg`
   - Modify `CMakeLists.txt` accordingly, referencing the structure in the **ROS2Bridge** `README.md`.

---

## Plugin Modification Guidelines

To improve the MyCobot280Pi plugin and integrate backend functionality, follow these steps:

### 1. **Remove Socket Connection**
- **Remove** `connectToSocket()` from `MyCobot280Pi.cpp`.
- **Delete** the **Connect Socket Server** component from `dockWidgetMyCobot280Pi.ui`.

### 2. **Replace `sendSocketData` Function**
- Modify the `sendSocketData` function in `MyCobot280Pi.cpp`.
- **Replace** its logic using **remote_control_server_test.py** as a reference.

### 3. **Implement MoveIt-based Control**
- Create new **C++ functions** in `MyCobot280Pi.cpp` using **MoveIt API**.
- Use `arm_control_service.py` as a reference for motion control implementation.

---

## Future Work
- Extend the plugin’s **UI** to reflect real-time **robot status**.
- Enhance **error handling** and communication reliability between ITOM and ROS2.
- Consider adding support for **custom trajectories and external sensor integration**.

---




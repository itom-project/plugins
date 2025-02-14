#!/usr/bin/env python3
import rospy
from mycobot_280pi_moveit.srv import ControlArm, GetJointStates
import socket
import os
import threading

class RobotControlServer:
    def __init__(self):
        rospy.init_node('robot_control_server')

        # Set up the socket server
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind(('0.0.0.0', 9999))
        self.sock.listen(5)
        self.sock.settimeout(1.0)
        rospy.loginfo('Socket server started, waiting for connection...')

        # Services are initially not connected
        self.control_arm_client = None
        self.get_joint_states_client = None

        # Handle client connections in a separate thread
        self.handle_client_connection()

        # Register shutdown hook
        rospy.on_shutdown(self.shutdown_hook)

    def handle_client_connection(self):
        while not rospy.is_shutdown():
            try:
                # Accept new connection with timeout
                client_socket, address = self.sock.accept()
                rospy.loginfo(f"Accepted connection from {address}")

                while not rospy.is_shutdown():
                    data = client_socket.recv(1024).decode('utf-8')
                    if not data:
                        break
                    rospy.loginfo(f"Received command: {data}")
                    response = self.process_command(data)
                    client_socket.sendall(response.encode('utf-8'))

                client_socket.close()
            except socket.timeout:
                # Timeout is fine. Just continue and check for shutdown signal
                continue

    def process_command(self, command):
        if command == "start_slider_control":
            threading.Thread(target=self.start_slider_control).start()
            return "Slider control started."

        elif command == "get_joint_states":
            if not self.get_joint_states_client:
                return "Error: Services are not yet connected. Use 'connect_services' to connect first."
            return self.get_joint_states()

        elif command == "start_arm_control_service":
            if not self.control_arm_client:
                return "Error: Services are not yet connected. Use 'connect_services' to connect first."
            threading.Thread(target=self.start_arm_control_service).start()
            return "Arm control service started."

        elif command == "connect_services":
            return self.connect_services()

        else:
            try:
                if command.startswith("control_arm"):
                    if not self.control_arm_client:
                        return "Error: Services are not yet connected. Use 'connect_services' to connect first."
                    
                    parts = command.split()[1:]  # Split off the "control_arm" part
                    joint_names = []
                    target_values = []

                    for part in parts:
                        joint_name, target_value = part.split(":")
                        joint_names.append(joint_name)
                        target_values.append(float(target_value))

                    return self.control_arm(joint_names, target_values)
            except Exception as e:
                rospy.logerr(f"Failed to process command: {e}")
                return f"Error processing command: {e}"

        return "Unknown command"

    def connect_services(self):
        rospy.loginfo("Attempting to connect to services...")

        # Try to connect to control_arm service
        try:
            rospy.wait_for_service('control_arm', timeout=5)
            self.control_arm_client = rospy.ServiceProxy('control_arm', ControlArm)
            rospy.loginfo("Connected to control_arm service.")
        except rospy.ROSException:
            return "Failed to connect to control_arm service."

        # Try to connect to get_joint_states service
        try:
            rospy.wait_for_service('get_joint_states', timeout=5)
            self.get_joint_states_client = rospy.ServiceProxy('get_joint_states', GetJointStates)
            rospy.loginfo("Connected to get_joint_states service.")
        except rospy.ROSException:
            return "Failed to connect to get_joint_states service."

        return "Services successfully connected."

    def start_slider_control(self):
        rospy.loginfo("Starting slider control...")
        os.system("roslaunch mycobot_280_moveit demo.launch")

    def start_arm_control_service(self):
        rospy.loginfo("Starting arm control service...")
        os.system("rosrun mycobot_280pi_moveit arm_control_service.py")

    def control_arm(self, joint_names, target_values):
        rospy.loginfo(f"Controlling joints {joint_names} to positions {target_values}")
        try:
            response = self.control_arm_client(False, joint_names, target_values)
            if response.success:
                rospy.loginfo(f"Motion executed successfully: {response.message}")
                return f"Motion executed successfully: {response.message}"
            else:
                rospy.logerr(f"Motion failed: {response.message}")
                return f"Motion failed: {response.message}"
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return f"Service call failed: {e}"

    def get_joint_states(self):
        rospy.loginfo("Retrieving joint states...")
        try:
            response = self.get_joint_states_client()
            if response.success:
                # Format the joint names and positions for the client
                joint_info = "\n".join([f"{name}: {position:.2f}" for name, position in zip(response.joint_names, response.joint_positions)])
                rospy.loginfo(f"Joint states retrieved:\n{joint_info}")
                return f"Joint states:\n{joint_info}"
            else:
                rospy.logerr(f"Failed to retrieve joint states: {response.message}")
                return f"Failed to retrieve joint states: {response.message}"
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return f"Service call failed: {e}"

    def shutdown_hook(self):
        rospy.loginfo("Shutting down server...")
        self.sock.close()

if __name__ == '__main__':
    server = RobotControlServer()

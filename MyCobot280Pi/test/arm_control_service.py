#!/usr/bin/env python3

import rospy
import sys
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize, roscpp_shutdown
from sensor_msgs.msg import JointState
from mycobot_280pi_moveit.srv import ControlArm, ControlArmResponse, GetJointStates, GetJointStatesResponse

class ArmControlService:
    def __init__(self):
        # Initialize the move_group API
        roscpp_initialize(sys.argv)
        rospy.init_node('arm_control_service', anonymous=True)

        # Initialize the MoveGroupCommander for your robot's arm group
        self.arm_group = MoveGroupCommander("arm_group")
        self.robot = RobotCommander()

        # Service to control arm movement or query joints
        self.control_service = rospy.Service('control_arm', ControlArm, self.handle_control_arm)
        
        # New service to get joint states
        self.get_joint_states_service = rospy.Service('get_joint_states', GetJointStates, self.handle_get_joint_states)

        # Variable to store the most recent joint states
        self.joint_states = None

        # Subscribe to /joint_states topic
        self.joint_state_subscriber = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        rospy.loginfo("Arm Control Service is ready.")

    def joint_state_callback(self, msg):
        # Save the latest joint states from the /joint_states topic
        self.joint_states = msg

    def get_joint_names(self):
        # Returns the names of all the joints in the arm_group
        joint_names = self.arm_group.get_active_joints()
        return joint_names

    def handle_control_arm(self, req):
        # If the request is to query available joints
        if req.query_joints:
            joint_names = self.get_joint_names()
            return ControlArmResponse(success=True, message="Available joints retrieved.", available_joints=joint_names)

        # Otherwise, proceed with controlling the joints
        joint_names = self.get_joint_names()
        for joint_name in req.joint_names:
            if joint_name not in joint_names:
                return ControlArmResponse(success=False, message=f"Joint '{joint_name}' not found.")

        # Set the target for each joint
        try:
            for joint_name, target_value in zip(req.joint_names, req.target_values):
                self.arm_group.set_joint_value_target(joint_name, target_value)

            # Plan and execute the motion
            self.arm_group.go(wait=True)
            self.arm_group.stop()  # Ensure no residual movement

            return ControlArmResponse(success=True, message="Motion executed successfully.")
        except Exception as e:
            return ControlArmResponse(success=False, message=str(e))

    def handle_get_joint_states(self, req):
        # Check if joint_states data has been received
        if self.joint_states is None:
            return GetJointStatesResponse(success=False, message="No joint states received yet.", joint_names=[], joint_positions=[])

        # Format the joint names and positions in the desired format
        joint_info = "\n".join([f"{name}: {position:.2f}" for name, position in zip(self.joint_states.name, self.joint_states.position)])

        return GetJointStatesResponse(
            success=True,
            message=f"Joint states retrieved successfully:\n{joint_info}",
            joint_names=self.joint_states.name,
            joint_positions=self.joint_states.position
        )

    def spin(self):
        rospy.spin()
        roscpp_shutdown()

if __name__ == "__main__":
    arm_control_service = ArmControlService()
    arm_control_service.spin()

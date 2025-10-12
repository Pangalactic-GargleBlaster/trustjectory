try:
    from Quanser.p_QArm import QArm
    print("Found the QArm software")
except ModuleNotFoundError:
    print("Mocking the QArm")
    class QArm:
        def __init__(self):
            self.measJointPosition = np.array([0,0,0,0],dtype=np.float64)
        def read_write_std(self, phiCMD=None, grpCMD=None, baseLED=None):
            if phiCMD is not None:
                self.measJointPosition = phiCMD
            if grpCMD is not None:
                self.measGripperPosition = grpCMD
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from trajectory_msgs.msg import JointTrajectoryPoint
import numpy as np 


arm = QArm()
phiCMD = np.array([0,0,0,0],dtype=np.float64)
gripCMD = np.float64(0)
ledCMD = np.array([0,1,0],dtype=np.float64)
arm.read_write_std(phiCMD=phiCMD, grpCMD=gripCMD, baseLED=ledCMD) # initialize at home position, green LED
ledCMD = np.array([0,0,1],dtype=np.float64) # from the first command onwards keep LED blue
def send_command_to_arm(joint_trajectory_point: JointTrajectoryPoint):
    phiCMD[0:4] = joint_trajectory_point.positions[0:4]
    gripCMD = np.float64(joint_trajectory_point.positions[4])
    arm.read_write_std(phiCMD=phiCMD, grpCMD=gripCMD, baseLED=ledCMD)

rclpy.init()
node = Node("QArm")
qos_profile = QoSProfile(**{
    'reliability': QoSReliabilityPolicy.RELIABLE,
    'durability': QoSDurabilityPolicy.VOLATILE,
    'depth': 1,
})
publisher = node.create_publisher(JointTrajectoryPoint, '/robot_state', qos_profile)
print("Created the publisher")
def publish_arm_state():
    arm.read_write_std()
    ros_message = JointTrajectoryPoint(**{
        'positions': np.append(arm.measJointPosition, arm.measGripperPosition),
    })
    publisher.publish(ros_message)
node.create_timer(0.005, publish_arm_state)
node.create_subscription(JointTrajectoryPoint, '/robot_commands', send_command_to_arm, qos_profile)
print("Created the subscriber")
rclpy.spin(node)
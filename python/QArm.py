import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from trajectory_msgs.msg import JointTrajectoryPoint
import numpy as np 
from Quanser.p_QArm import QArm
import time

arm = QArm()
phiCMD = np.array([0,0,0,0],dtype=np.float64)
gripCMD = np.array([0],dtype=np.float64)
ledCMD = np.array([0,1,0],dtype=np.float64)
arm.read_write_std(phiCMD=phiCMD, grpCMD=gripCMD, baseLED=ledCMD) # initialize at home position, green LED
ledCMD = np.array([0,0,1],dtype=np.float64) # from the first command onwards keep LED blue
global last_message_time
last_message_time = time.time()
def send_command_to_arm(joint_trajectory_point: JointTrajectoryPoint):
    global last_message_time
    now = time.time()
    delta_t = now - last_message_time
    last_message_time = now
    print(f"delta_t: {delta_t}, count: {joint_trajectory_point.positions[0]}")

    phiCMD[0:4] = joint_trajectory_point.positions[0:4]
    gripCMD = np.array([joint_trajectory_point.positions[4]], dtype=np.float64)
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
    arm.read_std()
    ros_message = JointTrajectoryPoint(**{
        'positions': arm.measJointPosition,
    })
    publisher.publish(ros_message)
node.create_timer(0.016, publish_arm_state)
node.create_subscription(JointTrajectoryPoint, '/robot_commands', send_command_to_arm, qos_profile)
print("Created the subscriber")
rclpy.spin(node)
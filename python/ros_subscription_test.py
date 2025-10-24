import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from trajectory_msgs.msg import JointTrajectoryPoint
import time
rclpy.init()
node = Node("QArm")
qos_profile = QoSProfile(**{
    'reliability': QoSReliabilityPolicy.RELIABLE,
    'durability': QoSDurabilityPolicy.VOLATILE,
    'depth': 1,
})
global last_message_time
last_message_time = time.time()
def log_stats(message: JointTrajectoryPoint):
    now = time.time()
    global last_message_time
    print(f"Delay: {now - last_message_time}, index: {message.positions[0]}")
    last_message_time = now
    for i in range(1000000000):
        if i%100000000 == 0:
            print(f"tick {i//100000000}")
node.create_subscription(JointTrajectoryPoint, '/robot_state', log_stats, qos_profile)
rclpy.spin(node)
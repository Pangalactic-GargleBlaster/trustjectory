import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from builtin_interfaces.msg import Duration
import numpy as np 
from Quanser.p_QArm import QArm
import time

arm = QArm()
home_pose = np.array([0,0,0,0,0.5])
armLED = np.array([0,1,0], dtype=np.float64) # initialize at home position, green LED
def send_command_to_arm(joint_position: np.array):
    arm.read_write_std(
        phiCMD=joint_position[0:4],
        grpCMD=joint_position[4:5],
        baseLED=armLED
    )
send_command_to_arm(home_pose)
armLED = np.array([0,0,1],dtype=np.float64) # from the first command onwards keep LED blue

rclpy.init()
node = Node("QArm")
qos_profile = QoSProfile(**{
    'reliability': QoSReliabilityPolicy.RELIABLE,
    'durability': QoSDurabilityPolicy.VOLATILE,
    'depth': 5,
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

def duration_in_seconds(ros_duration: Duration) -> float:
    return ros_duration.sec + ros_duration.nanosec/1000000000

def lerp(a,b,t):
    print(f"lerp: {a*(1-t)+b*t}")
    return a*(1-t)+b*t

def run_trajectory(joint_trajectory: JointTrajectory):
    trajectory_length = joint_trajectory.points.__len__()
    times_array = np.zeros(trajectory_length)
    positions_array = np.zeros((trajectory_length,5))
    for index, point in enumerate(joint_trajectory.points):
        times_array[index] = duration_in_seconds(point.time_from_start)
        positions_array[index] = point.positions
    trajectory_duration = times_array[-1]
    if trajectory_length > 2:
        print(f"Running trajectory with {trajectory_length} points. It will take {trajectory_duration}s.")
    current_trajectory_index = 0
    start_time = time.time()
    while (time_from_start := time.time() - start_time) < trajectory_duration:
        while(times_array[current_trajectory_index+1] < time_from_start):
            current_trajectory_index += 1
        send_command_to_arm(lerp(
            positions_array[current_trajectory_index],
            positions_array[current_trajectory_index+1],
            (time_from_start - times_array[current_trajectory_index]) / (times_array[current_trajectory_index+1] - times_array[current_trajectory_index])
        ))
        publish_arm_state()
        time.sleep(0.016)
    send_command_to_arm(positions_array[-1])
    if trajectory_length > 2:
        print("Trajectory complete.")

node.create_subscription(JointTrajectory, '/robot_commands', run_trajectory, qos_profile)
print("Created the subscriber")
rclpy.spin(node)
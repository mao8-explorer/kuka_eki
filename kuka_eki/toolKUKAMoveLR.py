import rospy
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from EkiConfig.ekiclients import EkiMotionClient
from EkiConfig.krl_struct import Axis, Pos
import signal
import sys
import numpy as np
import math
from EkiConfig import *
import time

# Global variables
target_index = 0
sigma = 50.0  # 距离阈值

# Define the target set to avoid redundancy
common_orientation = (-125.0, 0.0, 180.0)
target_set = [
    Pos(800.0, 1900.0, 1280.0, *common_orientation),
    Pos(1800.0, 1900.0, 1280.0, *common_orientation)
]
TCP_target_pos = Pos(1300.0, 1900.0, 1280.0, *common_orientation)

max_buffersize_limit = 5
E1_max_buffersize_limit = 1


def sigint_handler(sig, frame):
    global eki_motion_client
    rospy.logwarn("SIGINT received, closing client...")
    eki_motion_client.stop()
    eki_motion_client.close()
    eki_motion_client = None  # Release reference to allow garbage collection
    sys.exit(0)

def sigterm_handler(sig, frame):
    global eki_motion_client
    rospy.logwarn("SIGTERM received, closing client...")
    eki_motion_client.stop()
    eki_motion_client.close()
    eki_motion_client = None  # Release reference to allow garbage collection
    sys.exit(0)

signal.signal(signal.SIGINT, sigint_handler)
signal.signal(signal.SIGTERM, sigterm_handler)

def robot_state_callback(msg):
    global target_index, target_set, sigma, max_buffersize_limit

    current_pos = pose_to_xyzabc_in_mm_deg(msg.pose)[0:3] # return np.array([x, y, z, a, b, c])
    target_pos = [target_set[target_index].x, target_set[target_index].y, target_set[target_index].z]
    distance = np.linalg.norm(np.array(current_pos) - np.array(target_pos))

    if distance < sigma:
        cur_buffersize = eki_motion_client.ReadBufferSize()
        if cur_buffersize is not None and cur_buffersize < max_buffersize_limit:
            target_index = (target_index + 1) % len(target_set)
            rospy.loginfo(f"Moving to next target: {target_set[target_index]}, Current Buffer: {cur_buffersize}")
            move_to_target(target_set[target_index])

def move_to_target(target):
    global eki_motion_client
    eki_motion_client.ptp(target, 80)



if __name__ == "__main__":

    eki_motion_client = EkiMotionClient("172.31.1.147", 54605)
    eki_motion_client.connect()

    E1_eki_motion_client = EkiMotionClient("172.31.1.147", 54607)
    E1_eki_motion_client.connect()

    speedControl_client = EkiMotionClient("172.31.1.147", 54608)
    speedControl_client.connect()

    rospy.init_node('PoseCommand_Client')
    rate = rospy.Rate(100)  # 10 Hz
    PoseCommand_pub = rospy.Publisher('PoseCommandClient', PoseStamped, queue_size=10)
    rospy.Subscriber("kuka_eki_state_publisher/robot_state", PoseStamped, robot_state_callback)
    CommandVisual = PoseStamped()
    CommandVisual.header.frame_id = 'world'  # 参考坐标系



    init_el_value = 35.0
    el_handle = AxisE1(e1=init_el_value)

    eki_motion_client.ptp(TCP_target_pos, 50)
    E1_eki_motion_client.asyptp(el_handle)

    for i in range(5):
        eki_motion_client.ptp(TCP_target_pos, 50)
    while eki_motion_client._is_running:
        rate.sleep()
        cur_buffersize = eki_motion_client.ReadBufferSize()
        if cur_buffersize == 0:
            break

    e1_value = 30.0
    e1_step = 5.0
    e1_direction = 1.0
    e1_lower_limit = 20.0
    e1_upper_limit = 50.0
    vel_scale = 10

    try:
        while not rospy.is_shutdown() and eki_motion_client._is_running and E1_eki_motion_client._is_running:
            speed_cur_buffersize = speedControl_client.ReadBufferSize()

            if speed_cur_buffersize is not None and speed_cur_buffersize < max_buffersize_limit:
                speedControl_client.speedSend(vel_scale)

            E1_cur_buffersize = E1_eki_motion_client.ReadBufferSize()
            if E1_cur_buffersize is not None and E1_cur_buffersize < E1_max_buffersize_limit:
                el_handle.e1 = e1_value
                E1_eki_motion_client.asyptp(el_handle)

                if e1_value > e1_upper_limit:
                    e1_direction = -1
                if e1_value < e1_lower_limit:
                    e1_direction = 1

                e1_value += e1_step * e1_direction
            
            rospy.spinOnce()  # 确保回调函数正常执行
            rate.sleep()

    except Exception as e:
        rospy.loginfo(f"An error occurred: {e}")
        eki_motion_client.close()
        E1_eki_motion_client.close()

    finally:
        eki_motion_client.close()
        E1_eki_motion_client.close()

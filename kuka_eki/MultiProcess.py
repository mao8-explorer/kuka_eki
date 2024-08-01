import rospy
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

from EkiConfig.ekiclients import EkiMotionClient
from EkiConfig.krl_struct import Axis,Pos
import signal
import sys
import numpy as np
import math
from EkiConfig import *
import multiprocessing
import time

def sigint_handler(sig, frame):
    global eki_motion_client
    # rospy.logwarn("Current buffer: {}, E1 buffer: {}".format(cur_buffersize, E1_cur_buffersize))("SIGINT received, closing client...")
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
    
# 注册SIGINT和SIGTERM信号处理函数
signal.signal(signal.SIGINT, sigint_handler)
signal.signal(signal.SIGTERM, sigterm_handler)



# Define the client function for E1_eki_motion_client
def e1_client_process(e1_eki_motion_client):

    E1_max_buffersize_limit = 2 # by default, limit command buffer to 5 (size of advance run in KRL)
    rate = rospy.Rate(30)  # Adjust the rate as needed

    e1_value = 30.0
    e1_step = 5.0
    e1_direction = 1.0
    e1_lower_limit = 20.0
    e1_upper_limit = 50.0

    try:
        while e1_eki_motion_client._is_running:
        
            E1_cur_buffersize = e1_eki_motion_client.ReadBufferSize()
            if E1_cur_buffersize is not None and E1_cur_buffersize < E1_max_buffersize_limit:
                el_handle.e1 = e1_value
                e1_eki_motion_client.asyptp(el_handle)

                if e1_value > e1_upper_limit:
                    e1_direction = -1
                if e1_value < e1_lower_limit:
                    e1_direction = 1

                e1_value += e1_step * e1_direction
            
            rospy.loginfo(f"Current E1 buffer: {E1_cur_buffersize}")
            rate.sleep()

    except Exception as e:
        rospy.loginfo(f"E1 An error occurred: {e}")
        E1_eki_motion_client.close()

    finally:
        # Close the motion client
        E1_eki_motion_client.close()




# Define the client function for eki_motion_client
def eki_client_process(eki_motion_client, PoseCommand_pub):

    max_buffersize_limit = 5 # by default, limit command buffer to 5 (size of advance run in KRL)
    rate = rospy.Rate(100)  # Adjust the rate as needed

    # Define circle parameters
    circle_radius = 500  # radius of the circle
    circle_center = (1300.0, 1900.0)  # center coordinates of the circle
    num_points = 100 # number of points to divide the circle
    angle_increment = 2 * math.pi / num_points  # angle increment for each point
    current_angle = 0
    rx = 10.0

    CommandVisual = PoseStamped()
    CommandVisual.header.frame_id = 'world'  # 参考坐标系
    
    try:
        while eki_motion_client._is_running:
            cur_buffersize = eki_motion_client.ReadBufferSize()

            if cur_buffersize is not None and cur_buffersize < max_buffersize_limit:
                x = circle_center[0] + circle_radius * math.cos(current_angle)
                y = circle_center[1] + circle_radius * math.sin(current_angle)

                TCP_target_pos = Pos(x, y, 1280.0, -125.0, 0.0, 180.0)
                eki_motion_client.ptp(TCP_target_pos, 80)

                TCP_pose = xyzabc_in_mm_deg_to_pose([*TCP_target_pos])
                CommandVisual.header.stamp = rospy.Time.now()
                CommandVisual.pose = TCP_pose
                PoseCommand_pub.publish(CommandVisual)

                current_angle += angle_increment
                if current_angle >= 2 * math.pi:
                    current_angle -= 2 * math.pi

                if rx == 10.0:
                    rx = -10.0
                else:
                    rx = 10.0

            rospy.loginfo(f"Current buffer: {cur_buffersize}")
            rate.sleep()

    except Exception as e:
        rospy.loginfo(f"Arm An error occurred: {e}")
        eki_motion_client.close()

    finally:
        # Close the motion client
        eki_motion_client.close()


if __name__ == "__main__":

    eki_motion_client = EkiMotionClient("172.31.1.147", 54605)
    eki_motion_client.connect()

    E1_eki_motion_client = EkiMotionClient("172.31.1.147", 54607)
    E1_eki_motion_client.connect()
    
    rospy.init_node('multi_client_node')
    PoseCommand_pub = rospy.Publisher('PoseCommandClient', PoseStamped, queue_size=10)


    # 初始位置确定
    init_el_value = 35.0 # ee_tools相对于 ee_pose的初始外部轴offset
    el_handle = AxisE1(e1 = init_el_value)
    TCP_target_pos = Pos(1300.0, 1900.0, 1280.0, -125.0, 0.0, 180.0)
    eki_motion_client.ptp(TCP_target_pos, 50)
    E1_eki_motion_client.asyptp(el_handle)
    
    # 定初始位置 确定末端TCP (约向下10cm)
    for i in range(5):
        eki_motion_client.ptp(TCP_target_pos, 50)
    while eki_motion_client._is_running:
        time.sleep(0.01)
        cur_buffersize = eki_motion_client.ReadBufferSize()
        if cur_buffersize == 0:
            break


    # Start the processes
    e1_process = multiprocessing.Process(target=e1_client_process, args=(E1_eki_motion_client,))
    eki_process = multiprocessing.Process(target=eki_client_process, args=(eki_motion_client,PoseCommand_pub))
    
    e1_process.start()
    eki_process.start()

    e1_process.join()
    eki_process.join()


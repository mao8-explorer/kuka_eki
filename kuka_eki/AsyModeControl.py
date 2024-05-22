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

def sigint_handler(sig, frame):
    global eki_motion_client
    rospy.logwarn("Current buffer: {}, E1 buffer: {}".format(cur_buffersize, E1_cur_buffersize))("SIGINT received, closing client...")
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




def tf_to_pose():
    rospy.init_node('tf_to_point')
    tf_listener = tf.TransformListener()
    # pose_pub = rospy.Publisher('/tf_pose', PoseStamped, queue_size=10)
    rate = rospy.Rate(100.0)

    pose_received = False
    while not rospy.is_shutdown() and not pose_received:
        try:
            (trans, rot) = tf_listener.lookupTransform('/joint_a7_Link', '/joint_a6_Link', rospy.Time(0))

            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'joint_a7_Link'
            pose.pose.position.x = trans[0]
            pose.pose.position.y = trans[1]
            pose.pose.position.z = trans[2]
            pose.pose.orientation.x = rot[0]
            pose.pose.orientation.y = rot[1]
            pose.pose.orientation.z = rot[2]
            pose.pose.orientation.w = rot[3]
            # pose_pub.publish(pose)

            # 设置标志为True，表示成功获取姿态信息
            pose_received = True

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()

    """将Pose转换为4x4变换矩阵 - 工具坐标系 TCP"""
    T_a6_to_a7 = pose_to_matrix(pose.pose)
    return T_a6_to_a7


"异步模式下的串行程序测试 - PTP 与 ASYPTP"


if __name__ == "__main__":

    eki_motion_client = EkiMotionClient("172.31.1.147", 54605)
    eki_motion_client.connect()

    E1_eki_motion_client = EkiMotionClient("172.31.1.147", 54607)
    E1_eki_motion_client.connect()
    

    
    max_buffersize_limit = 10 # by default, limit command buffer to 5 (size of advance run in KRL)
    E1_max_buffersize_limit = 3 # by default, limit command buffer to 5 (size of advance run in KRL)

    # 确定ee_tools 与 ee_pose的固定坐标变换关系
    init_el_value = 35.0 # ee_tools相对于 ee_pose的初始外部轴offset
    el_handle = AxisE1(e1 = init_el_value)
    target_pos = Pos(10.0, 1765.0, 1910.0, -40.0, 90.0, -93.0)
    eki_motion_client.ptp(target_pos, 50)
    E1_eki_motion_client.asyptp(el_handle)
    
    # 定初始位置 确定末端TCP (约向下10cm)
    for i in range(5):
        eki_motion_client.ptp(target_pos, 50)
    while eki_motion_client._is_running  and eki_motion_client.ReadBufferSize() != 0: pass

    T_a6_to_a7 = tf_to_pose() # TCP工具坐标系 get from rviz - based on URDF (Trust!)
    rospy.loginfo("EE TCP Config Successfully!")


    TCP_target_pos = Pos(190.0, 2000.0, 1134.0, 90.0, 180.0, 0.0)
    ##---------------->>>
    TCP_pose = xyzabc_in_mm_deg_to_pose([*TCP_target_pos])

    # 计算从工具坐标系->机械臂末端->基坐标系的变换矩阵
    T_a7_to_base = pose_to_matrix(TCP_pose)
    T_a6_to_base = np.dot(T_a7_to_base, T_a6_to_a7)

    # 转换回 Pose 对象并转换为 xyzabc 格式
    a6_pose = matrix_to_pose(T_a6_to_base)
    xyzabc = pose_to_xyzabc_in_mm_deg(a6_pose) # shape = (6,)
    ## <<<----------------

    # 调用运动控制客户端
    eki_motion_client.ptp(Pos(*xyzabc),50)
    E1_eki_motion_client.asyptp(el_handle)

    # Define circle parameters
    circle_radius = 500  # radius of the circle
    circle_center = (190.0, 2000.0)  # center coordinates of the circle
    num_points = 20  # number of points to divide the circle
    angle_increment = 2 * math.pi / num_points  # angle increment for each point

    # Initialize angle and initial e1_value
    current_angle = 0
    e1_value = 30.0  # initial value for e1 axis

    # e1 parameters
    e1_step = 10.0
    e1_direction = 1.0
    e1_lower_limit = 20.0
    e1_upper_limit = 50.0

    rx = 10.0


    try:
        while eki_motion_client._is_running and E1_eki_motion_client._is_running:
            cur_buffersize = eki_motion_client.ReadBufferSize()
            E1_cur_buffersize = E1_eki_motion_client.ReadBufferSize()

            # E1 移动
            if E1_cur_buffersize < E1_max_buffersize_limit:

                el_handle.e1 = e1_value
                E1_eki_motion_client.asyptp(el_handle)

                if(e1_value > e1_upper_limit):
                    e1_direction = -1
                if(e1_value < e1_lower_limit):
                    e1_direction = 1
    
                e1_value +=  e1_step * e1_direction
            
            rospy.loginfo("Current buffer: {}, E1 buffer: {}".format(cur_buffersize, E1_cur_buffersize))

            # Axis 1~6 + (X Y Z A B C)
            if cur_buffersize < max_buffersize_limit: 
                # Calculate target position based on current angle
                x = circle_center[0] + circle_radius * math.cos(current_angle)
                y = circle_center[1] + circle_radius * math.sin(current_angle)

                # Create Pos object with constant z-value, calculated x, y values, and e1 axis value
                # target_pos = Pos(x,y, 1910.0, -40.0, 90.0, -93.0, e1=e1_value)
                # # Perform point-to-point motion to target position
                # eki_motion_client.ptp(target_pos, 50)

                TCP_target_pos = Pos(x, 2000.0, 1134.0, 90.0, 180.0, 0.0)
                # 坐标系转换接口！！
                ##---------------->>>
                TCP_pose = xyzabc_in_mm_deg_to_pose([*TCP_target_pos])

                # 计算从工具坐标系->机械臂末端->基坐标系的变换矩阵
                T_a7_to_base = pose_to_matrix(TCP_pose)
                T_a6_to_base = np.dot(T_a7_to_base, T_a6_to_a7)

                # 转换回 Pose 对象并转换为 xyzabc 格式
                a6_pose = matrix_to_pose(T_a6_to_base)
                xyzabc = pose_to_xyzabc_in_mm_deg(a6_pose) # shape = (6,)
                ## <<<----------------
                # 调用运动控制客户端
                eki_motion_client.ptp(Pos(*xyzabc),50)

                # Increment angle for next iteration
                current_angle += angle_increment
                if current_angle >= 2 * math.pi:
                    current_angle -= 2 * math.pi

                if (rx == 10.0):
                    rx =-10.0
                else:
                    rx = 10.0



    except Exception as e:
        rospy.loginfo(f"An error occurred: {e}")
        eki_motion_client.close()
        E1_eki_motion_client.close()

    finally:
        # Close the motion client
        eki_motion_client.close()
        E1_eki_motion_client.close()


import signal
import sys
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import numpy as np
from EkiConfig import EkiStateClient
from EkiConfig import xyzabc_in_mm_deg_to_pose
import socket

def sigint_handler(sig, frame):
    global eki_state_client
    print("SIGINT received, closing client...")
    eki_state_client.close()
    eki_state_client = None  # Release reference to allow garbage collection
    sys.exit(0)

def sigterm_handler(sig, frame):
    global eki_state_client
    print("SIGTERM received, closing client...")
    eki_state_client.close()
    eki_state_client = None  # Release reference to allow garbage collection
    sys.exit(0)

# 注册SIGINT和SIGTERM信号处理函数
signal.signal(signal.SIGINT, sigint_handler)
signal.signal(signal.SIGTERM, sigterm_handler)


if __name__ == "__main__":

    eki_state_client = EkiStateClient("172.31.1.147",54606)
    try:
        eki_state_client.connect()
    except socket.timeout:
        print("Connection timed out. Exiting.")
        sys.exit(1)

    # ros-related 
    name = 'kuka_eki_state_publisher'
    rospy.init_node(name)  # 添加 anonymous=True 参数
    joint_state_pub = rospy.Publisher('~joint_states', JointState, queue_size=10)
    robot_state_pub = rospy.Publisher('~robot_state', PoseStamped, queue_size=10)

    joint_state = JointState()
    joint_state.name = [
        'joint_a1',
        'joint_a2',
        'joint_a3',
        'joint_a4',
        'joint_a5',
        'joint_a6',
        'joint_a7']
    
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = 'base_link'

    while eki_state_client._is_running and not rospy.is_shutdown():
        # time.sleep(1.0)
        state = eki_state_client.state()
        if state:
            now = rospy.Time.now()
            joint_state.header.stamp = now

            # 将前六个轴的角度值从度转换为弧度
            angles_in_radians = np.deg2rad([
                float(state.axis.a1),
                float(state.axis.a2),
                float(state.axis.a3),
                float(state.axis.a4),
                float(state.axis.a5),
                float(state.axis.a6),
            ])

            # 计算第七个轴的值并保留小数点后6位
            e1_value = round((float(state.axis.e1) - 10.0) * 3.25 / 1000.0, 6)

            # 将前六个值和第七个值合并成一个包含7个浮点数的数组
            joint_state.position = np.append(angles_in_radians, e1_value)

            # print(state.pos)
            # print(state.axis)
            joint_state_pub.publish(joint_state)


            # 真机末端距离发布 EE_pose
            pose_stamped.header.stamp = now
            pose_stamped.pose = xyzabc_in_mm_deg_to_pose([
                float(state.pos.x),
                float(state.pos.y),
                float(state.pos.z),
                float(state.pos.a),
                float(state.pos.b),
                float(state.pos.c)])

            robot_state_pub.publish(pose_stamped)



    eki_state_client.close()

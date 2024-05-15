from typing import Tuple
import socket

from dataclasses import dataclass
import xml.etree.ElementTree as ET
import time

import signal
import sys
import rospy
from sensor_msgs.msg import JointState
import numpy as np


# 状态接收指令
Address = Tuple[str, int]
class TcpClient:
    def __init__(self, address: Address, timeout: int = 5):
        self._address: Address = address
        self._timeout = timeout
        self._socket: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def connect(self) -> None:
        self._socket.settimeout(self._timeout) 
        self._socket.connect(self._address)

    def sendall(self, data: bytes) -> None:
        self._socket.sendall(data)

    def recv(self, bufsize: int = 1024) -> bytes:
        return self._socket.recv(bufsize)
    
    def close(self):
        self._socket.close()



@dataclass
class Axis:
    a1: float = 0.0
    a2: float = 0.0
    a3: float = 0.0
    a4: float = 0.0
    a5: float = 0.0
    a6: float = 0.0
    e1: float = 0.0

    def to_xml(self, root: ET.Element) -> ET.Element:
        element = ET.SubElement(
            root,
            "Axis",
            {
                "A1": str(self.a1),
                "A2": str(self.a2),
                "A3": str(self.a3),
                "A4": str(self.a4),
                "A5": str(self.a5),
                "A6": str(self.a6),
                "E1": str(self.e1),
            },
        )
        return element


@dataclass
class Pos:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    a: float = 0.0
    b: float = 0.0
    c: float = 0.0
    s: int = 0
    t: int = 0

    def to_xml(self, root: ET.Element) -> ET.Element:
        element = ET.SubElement(
            root,
            "Cart",
            {
                "X": str(self.x),
                "Y": str(self.y),
                "Z": str(self.z),
                "A": str(self.a),
                "B": str(self.b),
                "C": str(self.c),
                "S": str(self.s),
                "T": str(self.t),
            },
        )
        return element

@dataclass
class RobotState:
    axis: Axis = Axis()
    pos: Pos = Pos()

    @classmethod
    def from_xml(cls, xml: bytes):
        root: ET.Element = ET.fromstring(xml)
        attrib: dict[str, str] = root.find("Pos").attrib
        pos = Pos(
            attrib["X"],
            attrib["Y"],
            attrib["Z"],
            attrib["A"],
            attrib["B"],
            attrib["C"],
            attrib["S"],
            attrib["T"],
        )
        attrib = root.find("Axis").attrib
        axis = Axis(
            attrib["A1"],
            attrib["A2"],
            attrib["A3"],
            attrib["A4"],
            attrib["A5"],
            attrib["A6"],
            attrib["E1"],
        )
        return RobotState(axis, pos)

    def to_xml(self) -> bytes:
        root: ET.Element = ET.Element("RobotState")
        self.axis.to_xml(root)
        self.pos.to_xml(root)
        return ET.tostring(root)


class EkiStateClient:
    def __init__(self, ip_address: str, STATE_PORT: int = 54606, connect_timeout: int = 5) -> None:
        self._tcp_client: TcpClient = TcpClient((ip_address, STATE_PORT), timeout=connect_timeout)
        self._is_running = True

    def connect(self) -> None:
        self._tcp_client.connect()

    def state(self) -> RobotState:
        try:
            data: bytes = self._tcp_client.recv(1024)
            if data is None:
                raise RuntimeError("No data received")
            # data是批量的数据，选定一个就可以。
            data_str = data.decode()
            start_index = data_str.find('<RobotState>')
            end_index = data_str.find('</RobotState>', start_index)
            if start_index != -1 and end_index != -1:
                first_robot_state = data_str[start_index:end_index+len('</RobotState>')]
                return RobotState.from_xml(first_robot_state)
            else:
                raise ValueError("Incomplete RobotState found in data")
        except (socket.error, ET.ParseError, RuntimeError, ValueError) as e:
            print(f"Error: {e}. Closing client.")
            self._is_running = False
            return None

    def close(self) -> None:
        self._tcp_client.close()



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
    rospy.init_node(name)
    joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    joint_state = JointState()
    joint_state.name = [
        'joint_a1',
        'joint_a2',
        'joint_a3',
        'joint_a4',
        'joint_a5',
        'joint_a6',
        'joint_a7']
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
            print(state.axis)
            joint_state_pub.publish(joint_state)


    eki_state_client.close()

import signal
import sys
import time
from typing import Tuple, Union
from enum import IntEnum
import xml.etree.ElementTree as ET
from dataclasses import dataclass
import socket


import math

from traitlets import directional_link
# 运动控制指令

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
    # s: int = 0
    # t: int = 0
    e1: float = 0.0

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
                # "S": str(self.s),
                # "T": str(self.t),
                "E1": str(self.e1),
            },
        )
        return element


class CommandType(IntEnum):
    PTP_AXIS = 1
    PTP_CART = 2


@dataclass
class CommandBuffersize:
    buffersize: int = 0
    @classmethod
    def from_xml(cls, xml: bytes):
        root: ET.Element = ET.fromstring(xml)
        attrib: dict[str, str] = root.find("RobotCommand").attrib
        buffersize = int(attrib["Size"])
        return buffersize

@dataclass
class RobotCommand:
    command_type: CommandType
    target: Union[Axis, Pos]
    velocity_scaling: int

    def to_xml(self) -> bytes:
        root = ET.Element("RobotCommand")
        ET.SubElement(root, "Type").text = str(self.command_type.value)
        self.target.to_xml(root)
        Pos().to_xml(root) if isinstance(self.target, Axis) else Axis().to_xml(root)
        ET.SubElement(root, "Velocity").text = str(self.velocity_scaling)
        return ET.tostring(root)


class EkiMotionClient:
    # def __init__(self, ip_address: str, MOTION_PORT: int = 54610) -> None:
    #     self._tcp_client = TcpClient((ip_address, MOTION_PORT))
    #     self._is_running = True

    def __init__(self, ip_address: str, MOTION_PORT: int = 54610) -> None:
        self._tcp_client = TcpClient((ip_address, MOTION_PORT))
        self._is_running = True  # 添加这个属性
        # ... [其他代码] ...

        # 添加一个方法来改变循环标志

    def stop(self) -> None:
        self._is_running = False

    def connect(self) -> None:
        try:
            self._tcp_client.connect()
        except socket.error as e:
            print(f"Error connecting to server: {e}")
            self._is_running = False

    def ptp(self, target: Union[Axis, Pos], max_velocity_scaling=20) -> None:
        if not self._is_running:
            print("Client is not connected. Cannot send command.")
            return
        command_type: CommandType
        if isinstance(target, Axis):
            command_type = CommandType.PTP_AXIS
        elif isinstance(target, Pos):
            command_type = CommandType.PTP_CART
        else:
            raise TypeError("Expected argument of type Axis or Pos")
        command: RobotCommand = RobotCommand(command_type, target, max_velocity_scaling)
        self._tcp_client.sendall(command.to_xml())

    def ReadBufferSize(self):
        try:
            data: bytes = self._tcp_client.recv(1024)
            if data is None:
                return 0
            # data是批量的数据，选定一个就可以。
            data_str = data.decode()
            start_index = data_str.find('<RobotState>')
            end_index = data_str.find('</RobotState>', start_index)
            if start_index != -1 and end_index != -1:
                first_robot_state = data_str[start_index:end_index+len('</RobotState>')]
                return CommandBuffersize.from_xml(first_robot_state)
            else:
                raise ValueError("Incomplete RobotState found in data")
        except (socket.error, ET.ParseError, RuntimeError, ValueError) as e:
            print(f"Error: {e}. Closing client.")
            self._is_running = False
            return None
        
    def close(self) -> None:
        self._tcp_client.close()





def sigint_handler(sig, frame):
    global eki_motion_client
    print("SIGINT received, closing client...")
    eki_motion_client.stop()
    eki_motion_client.close()
    eki_motion_client = None  # Release reference to allow garbage collection
    sys.exit(0)

def sigterm_handler(sig, frame):
    global eki_motion_client
    print("SIGTERM received, closing client...")
    eki_motion_client.stop()
    eki_motion_client.close()
    eki_motion_client = None  # Release reference to allow garbage collection
    sys.exit(0)

# 注册SIGINT和SIGTERM信号处理函数
signal.signal(signal.SIGINT, sigint_handler)
signal.signal(signal.SIGTERM, sigterm_handler)




if __name__ == "__main__":

    eki_motion_client = EkiMotionClient("172.31.1.147", 54605)
    eki_motion_client.connect()
    
    
    max_buffersize_limit = 10 # by default, limit command buffer to 5 (size of advance run in KRL)
    target_axis = Axis(a1=-90, a2=-90.0, a3=90, a4=0.0, a5=0.0, a6=0.0, e1 = 20)
    target_pos = Pos(1090.0, 1387.0, 1910.0, -40.0, 90.0, -93.0, e1 = 20)

    eki_motion_client.ptp(target_pos, 50)

    # Define circle parameters
    circle_radius = 500  # radius of the circle
    circle_center = (1090.0, 1387.0)  # center coordinates of the circle
    num_points = 30  # number of points to divide the circle
    angle_increment = 2 * math.pi / num_points  # angle increment for each point

    # Initialize angle and initial e1_value
    current_angle = 0
    e1_value = 20  # initial value for e1 axis

    # e1 parameters
    e1_step = 8.0
    e1_direction = 1.0
    e1_lower_limit = 20.0
    e1_upper_limit = 50.0

    try:
        while eki_motion_client._is_running:
            cur_buffersize = eki_motion_client.ReadBufferSize()
            if cur_buffersize < max_buffersize_limit: 
                print("current buffer size: ", cur_buffersize)
                # Calculate target position based on current angle
                x = circle_center[0] + circle_radius * math.cos(current_angle)
                y = circle_center[1] + circle_radius * math.sin(current_angle)

                # Create Pos object with constant z-value, calculated x, y values, and e1 axis value
                target_pos = Pos(x, y, 1910.0, -40.0, 90.0, -93.0, e1=e1_value)
                # Perform point-to-point motion to target position
                eki_motion_client.ptp(target_pos, 50)

                # Increment angle for next iteration
                current_angle += angle_increment
                if current_angle >= 2 * math.pi:
                    current_angle -= 2 * math.pi

                if(e1_value > e1_upper_limit):
                    e1_direction = -1
                if(e1_value < e1_lower_limit):
                    e1_direction = 1
    
                e1_value +=  e1_step * e1_direction



    except Exception as e:
        print(f"An error occurred: {e}")
        eki_motion_client.close()

    finally:
        # Close the motion client
        eki_motion_client.close()


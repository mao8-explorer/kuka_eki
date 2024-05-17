from .krl_struct import Axis, Pos
from .krl_struct import RobotState
from .krl_struct import CommandType, CommandBuffersize, RobotCommand
from .tcpclient import TcpClient
import socket
from typing import Tuple, Union
import xml.etree.ElementTree as ET

class EkiMotionClient:
    # def __init__(self, ip_address: str, MOTION_PORT: int = 54610) -> None:
    #     self._tcp_client = TcpClient((ip_address, MOTION_PORT))
    #     self._is_running = True

    def __init__(self, ip_address: str, MOTION_PORT: int = 54610) -> None:
        self._tcp_client = TcpClient((ip_address, MOTION_PORT))
        self._is_running = True  # 添加这个属性

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


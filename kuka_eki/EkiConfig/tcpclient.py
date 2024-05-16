from typing import Tuple
import socket

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
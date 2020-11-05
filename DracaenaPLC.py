import socket
import enum


class DracaenaCodes(enum.Enum):
    Acknowledge = b'\x00\x01'
    VisionStart = b'\x00\x02'
    GripperClose = b'\x00\x03'
    GripperOpen = b'\x00\x04'
    RotatePlant = b'\x00\05'
    NoDetections = b'\x00\x08'
    Shutdown = b'\x00\xFE'
    EmergencyStop = b'\x00\xFF'


class DracaenaPLC:
    """
    Python interface to Dracaena PLC
    """

    def __init__(self, host="192.168.0.50", port=2000):
        self.host = host
        self.port = port
        self.plc_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def connect_to_plc_socket(self):
        print("Connecting to PLC...")
        self.plc_socket.connect((self.host, self.port))

    def send(self, data: bytes):
        try:
            self.plc_socket.send(data)
        except socket.error:
            print('Socket error!')

    def receive(self, length):
        return self.plc_socket.recv(length)

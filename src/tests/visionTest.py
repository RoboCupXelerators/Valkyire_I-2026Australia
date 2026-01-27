import socket
import pickle
import time
from Vision import UnitedVision

# UDP socket setup
UDP_IP = "127.0.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

vision = UnitedVision()

while True:
    ball_pos = vision.GetBallPos()
    chassis_pos = vision.GetChassisPos()
    print(f"Ball Position: {ball_pos}, Chassis Position: {chassis_pos}")
    time.sleep(0.01)  # Adjust sleep time as needed for performance
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
    data = {"ball_pos": ball_pos, "chassis_pos": chassis_pos}
    pickled_data = pickle.dumps(data)
    sock.sendto(pickled_data, (UDP_IP, UDP_PORT))
    time.sleep(0.01)  # Adjust sleep time as needed for performance
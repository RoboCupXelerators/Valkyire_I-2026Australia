import socket
import pickle
import time
from ReasonData import Settings
from Sensor import Lidar

cfg = Settings

# Initialize Bits based on config
if cfg.RoboInfo.Bit == "AB":
    from utils.ArisuBits import ArisBit
    Bits = ArisBit()
elif cfg.RoboInfo.Bit == "IL":
    from utils.IceLoongBits import IceLoongBits
    Bits = IceLoongBits(port=Settings.Ports.IceLoongPort)
else:
    raise RuntimeError("Unsupported Bit type")

lidar = Lidar(Bits.GetYaw)

# UDP socket setup
UDP_IP = "127.0.0.1"
UDP_PORT = 5006
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

while True:
    dists = lidar.GetDists()
    data = {"dists": dists}
    pickled_data = pickle.dumps(data)
    sock.sendto(pickled_data, (UDP_IP, UDP_PORT))
    time.sleep(0.01)
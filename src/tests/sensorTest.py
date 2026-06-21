from ReasonData import logger, Settings
from LunaPre.sensor import Lidar
from LunaPre.motion.chassis import Car, Peripherals, Key # Universal-Movement-Standard

from utils.IceLoongBits import IceLoongBits

Bits = IceLoongBits(port=Settings.Ports.IceLoongPort)
lidar = Lidar(Bits.GetYaw)
peripheral = Peripherals(Bits.VirtualPorts) #TODO
chassis = Car(Bits.SetWheelSpeed,Bits.GetYaw)
compass = Bits.GetYaw

try:
    while(1):
        (lidar.GetDists())
except KeyboardInterrupt:
    lidar.stop()

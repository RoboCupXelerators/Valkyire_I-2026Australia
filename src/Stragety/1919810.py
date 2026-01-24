#### HEADER ####
from ReasonData import QkJson, Positions
from kits import *
from OD import *
import signal
from Defence_link import *
from Defence_Follow_panning import*
from Defence_Follow_ratio import*

#### HEADER ####
logger.info("Starting Xelerator")


class Key:
    def __init__(self,GetKeyFunc):
        self.KeyCount = 0
        self.LastPressed = False
        
        self.GetKey = GetKeyFunc

        self.KeyEventThread = threading.Thread(target=self.KeyEvent)
        self.KeyEventThread.daemon = True
        self.KeyEventThread.start()
    
    def KeyEvent(self):
        while True:
            if self.LastPressed == True:
                if self.GetKey():
                    self.KeyCount = self.KeyCount + 1
                else:
                    self.LastPressed = False
            else:
                if self.GetKey():
                    self.LastPressed = True
                    self.KeyCount = self.KeyCount + 1
            time.sleep(0.01)
    
    def LongPress(self):
        if self.KeyCount >= 500:
            return True
        else:
            return False
    
    def Press(self):
        return self.GetKey()

key = Key(Bits.GetKey)

Role()

while True:
    if key.Press():
        break

while True:
    try:
        DefenceLink(1)
    except KeyboardInterrupt:
        chassis.stop()
        peripheral.StopDribble()
        break

from ReasonData import config

class RobomasterBits:
    def __init__(self):
        import serial
        self.ser = serial.Serial()
        self.ser.port = config().read("BASE","COM")
        self.ser.baudrate = 115200
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.parity = serial.PARITY_NONE
        self.ser.timeout = 0.2
        self.ser.write("command;".encode('utf-8'))
        if "ok" in self.ser.readall().decode('utf-8'):
            pass
        else:
            raise ConnectionRefusedError("Failed To Connect SDK,errorMsg: %s",self.ser.readall().decode('utf-8'))

    def send(self,command:str):
        self.ser.open()
        command += ';'
        self.ser.write(command.encode('utf-8'))
        recv = self.ser.readall().decode('utf-8')
        self.ser.close()
        return recv

    def SetMotor(self, w1:int, w2:int, w3:int, w4:int):
        recv = self.send("chassis wheel w1%s w2 %s w3 %s w4 %s",(w1,w2,w3,w4))
        return recv

    def GetBatt(self):
        return self.send("robot battery ?")
    
    def GetMovedDistance(self, x:int, y:int):
        return self.send("chassis move x %s y %s",(x,y))
    
    def GetMovedPos(self):
        return self.send("chassis position ?")
    
    def GetYaw(self):
        #TODO GetAngle
        recv = self.send("chassis attitude ?")
        return recv
    
    



import math
from ReasonData.config import Settings
import time, threading

class Car:
    def __init__(self,SetMotorFunc,GetYaw=None,MotorEncoder=None):
        self.SetMotorFunc = SetMotorFunc
        self.GetYaw = GetYaw
        self.MotorEncoder = MotorEncoder
        self.Kp = 1
        self.cfg = Settings
        if self.cfg.Debug.Database:
            from ReasonData.data import Outputs
            self.DataBase = Outputs()
        if self.cfg.Debug.FullLog:
            from ReasonData import logger
            self.logger = logger
    
    def SetMotor(self,speedCache:list[int,int,int,int]):
        number = [self.cfg.Ports.LeftFront,self.cfg.Ports.LeftBack,self.cfg.Ports.RightFront,self.cfg.Ports.RightBack]
        speed = [0,0,0,0]
        for n in number:
            if speedCache[n-1] > self.cfg.ExpectedVals.MaxSpeedValue:
                speed[n-1] = self.cfg.ExpectedVals.MaxSpeedValue
            else:
                speed[n-1] = int(speedCache[n-1])
        if self.cfg.Debug.FullLog:
            if self.MotorEncoder:
                EncoderCache = self.MotorEncoder()
                Encoder =[0,0,0,0]
                for n in number:
                    Encoder[n-1] = int(EncoderCache[n-1])
                self.logger.debug("SetMotorVals: (%s, %s, %s, %s) | With feedbackVals %s" %
                                   (speed[0], speed[1], speed[2], speed[3],Encoder)
                                   )
            else:
                self.logger.debug("SetMotorVals: %s, %s, %s, %s" % (speed[0], speed[1], -speed[2], -speed[3]))
        self.SetMotorFunc(speed[0], speed[1], speed[2], speed[3])
    
    def SetKp(self,Kp):
        self.Kp = Kp
    
    def Compass(self):
        return self.GetYaw()

    def RelMoveVetor(self,SpeedX,SpeedY,SpeedZ):
        '''
        a vector movement (SpeedX,SpeedY,SpeedZ) without YawCorrect
        '''
        Speed1 = SpeedX - SpeedY - SpeedZ
        Speed2 = SpeedX + SpeedY - SpeedZ
        Speed3 = SpeedX + SpeedY + SpeedZ
        Speed4 = SpeedX - SpeedY + SpeedZ
        output = [Speed1, Speed2, Speed3, Speed4]
        self.SetMotor(output)
    
    def RelMoveAngle(self,Angle,Speed,Kp:float|None = None):
        '''
        angle movement (Angle,Speed) without YawCorrect
        '''
        rad = math.radians(Angle)
        SpeedX = int(math.sin(rad) * Speed)
        SpeedY = int(math.cos(rad) * Speed)
        self.RelMoveVetor(SpeedX,SpeedY,0,Kp)

    def AbsMoveAngle(self,FacingAngle,MovingAngle,Speed,Kp:float|None = None):
        '''
        angle movement (FacingAngle,MovingAngle,speed) with YawCorrect
        '''
        Yaw = self.GetYaw()
        rad = math.radians((MovingAngle-Yaw+FacingAngle))
        SpeedX = int(math.cos(rad) * Speed)
        SpeedY = int(math.sin(rad) * Speed)
        self.AbsMoveVetor(SpeedX,SpeedY,FacingAngle,Kp)
        
    def AbsMoveVetor(self,SpeedX,SpeedY,FacingAngle,Kp:float|None = None):
        '''
        vector movement (SpeedX,SpeedY,SpeedZ) with YawCorrect
        '''
        Yaw = self.GetYaw()
        # TODO change facing
        # FacingAngle = 360 - FacingAngle
        Error = Yaw - FacingAngle
        Error = -( (Error + 180) % 360 - 180 ) # Normalize to [-180, 180]
        if not Kp:
            Kp = self.Kp
        SpeedZ = Error * Kp
        WheelX = SpeedX * math.cos(math.radians(Yaw)) + SpeedY * math.sin(math.radians(Yaw))
        WheelY = - SpeedX * math.sin(math.radians(Yaw)) - SpeedY * math.cos(math.radians(Yaw))
        print(SpeedX,SpeedY,SpeedZ,WheelX,WheelY)
        self.RelMoveVetor(WheelX, WheelY, SpeedZ)
    
    def RelXMove(self,Angle,Speed):
        self.RelMoveVetor(90,Angle,Speed)
   
    def RelYMove(self,Angle,Speed,Kp=None):
        self.RelMoveVetor(0,Angle,Speed,Kp)

    def AbsTurn(self,Angle):
        if self.GetYaw is None:
            return False
        self.AbsMoveVetor(0,0,Angle)

    def RelTurn(self,Speed): #自转
        self.RelMoveVetor(0,0,Speed)
    
    def TurnTo(self,Angle,Kp=None):
        while True:
            Error = self.GetYaw() - Angle
            if not abs(Error) < 5:
                self.AbsMoveVetor(0,0,Angle)
            else:
                self.stop()
                break
            time.sleep(0.03)
            
    def stop(self):
        self.SetMotor([0,0,0,0])

        

class Peripherals:
    def __init__(self,IOFunc):
        self.SetIO = IOFunc
        from ReasonData import Settings
        self.cfg = Settings
        from ReasonData import logger
        self.logger = logger
        self.SetIO(self.cfg.Ports.ElecMagnet,1)
        self.DribbleStatus = False

    def ShootBall(self):
        self.SetIO(self.cfg.Ports.ElecMagnet,0)
        time.sleep(0.3)
        self.SetIO(self.cfg.Ports.ElecMagnet,1)
        self.logger.info("Used ElecMagnet")

    def Dribble(self,Status:bool):
        if self.DribbleStatus == Status:
            pass
        else:
            if Status:
                self.SetIO(self.cfg.Ports.Dribble,1)
            else:
                self.SetIO(self.cfg.Ports.Dribble,0)
            self.DribbleStatus = Status
            self.logger.info("Dribble set to %s" % ("ON" if Status else "OFF"))

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


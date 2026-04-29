import math
from ReasonData.config import Settings
from ReasonData import logger
import time, threading

class Car:
    def __init__(self,SetMotorFunc,GetYaw=None,MotorEncoder=None):
        self.SetMotorFunc = SetMotorFunc
        self.GetYaw = GetYaw
        self.MotorEncoder = MotorEncoder
        self.Kp = 3
        self.KpW = 1
        self.cfg = Settings
        self.speedCache = [0,0,0,0]
        import simple_pid
        self.Yaw_PID = simple_pid.PID(2.3, 0.08, 0.05, setpoint=0,output_limits=(-100,100),auto_mode=True)
        # Always use the unified project logger (ReasonData.logger)
        # to avoid AttributeError when FullLog=False.
        self.logger = logger
        # TODO Database is ready to archieve
        # if self.cfg.Debug.Database:
        #     from ReasonData.data import Outputs
        #     self.DataBase = Outputs()
        # Note: debug verbosity is still controlled by cfg.Debug.FullLog in calls.
    
    def SetMotor(self,speedCache:list[int,int,int,int]):
        self.speedCache = speedCache
        number = [self.cfg.Ports.LeftFront,self.cfg.Ports.LeftBack,self.cfg.Ports.RightFront,self.cfg.Ports.RightBack]
        speed = [0,0,0,0]
        for n in range(4):
            if speedCache[number[n]-1] > self.cfg.ExpectedVals.MaxSpeedValue:
                speed[n] = self.cfg.ExpectedVals.MaxSpeedValue
            else:
                speed[n] = int(speedCache[number[n]-1])
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

    def RobotMoveVector(self,SpeedX,SpeedY,SpeedZ):
        '''
        a vector movement (SpeedX,SpeedY,SpeedZ) without YawCorrect
        '''
        Speed1 = SpeedX*self.Kp - SpeedY*self.Kp - SpeedZ*self.KpW
        Speed2 = SpeedX*self.Kp + SpeedY*self.Kp - SpeedZ*self.KpW
        Speed3 = SpeedX*self.Kp + SpeedY*self.Kp + SpeedZ*self.KpW
        Speed4 = SpeedX*self.Kp - SpeedY*self.Kp + SpeedZ*self.KpW
        output = [Speed1, Speed2, Speed3, Speed4]
        self.SetMotor(output)
    
    def RobotMoveAngle(self,Angle,Speed,Kp:float|None = 1):
        '''
        angle movement (Angle,Speed) without YawCorrect
        '''
        rad = math.radians(Angle)
        SpeedX = int(math.sin(rad) * Speed)
        SpeedY = int(math.cos(rad) * Speed)
        self.RobotMoveVector(SpeedY,SpeedX,0)

    def FieldMoveAngle(self,FacingAngle,MovingAngle,Speed,Kp:float|None = None):
        '''
        angle movement (FacingAngle,MovingAngle,speed) with YawCorrect
        '''
        Yaw = self.GetYaw()
        rad = math.radians((MovingAngle-Yaw+FacingAngle))
        SpeedX = int(math.cos(rad) * Speed)
        SpeedY = int(math.sin(rad) * Speed)
        self.FieldMoveVector(SpeedX,SpeedY,FacingAngle,Kp)
        
    def FieldMoveVector(self,SpeedX,SpeedY,FacingAngle,Kp:float|None = None,KpW:float|None = None):
        '''
        vector movement (SpeedX,SpeedY,AngleZ˚,Angle_Gain) with YawCorrect
        '''
        Yaw = self.GetYaw()
        # TODO change facing
        # FacingAngle = 360 - FacingAngle
        Delta = Yaw - FacingAngle
        Error = -( (Delta + 180) % 360 - 180 ) # Normalize to [-180, 180]
        Normaled_Yaw =  ( (Yaw + 180) % 360 - 180 )
        Normaled_Facing =  ( (FacingAngle + 180) % 360 - 180 )
        if not Kp:
            Kp = self.Kp
        if not KpW:
            KpW = self.KpW
        if Settings.Debug.Beta:
            self.Yaw_PID.setpoint = Normaled_Facing
            SpeedZ = self.Yaw_PID(Normaled_Yaw)
            logger.error(f"PID output: {SpeedZ}, Yaw: {Normaled_Yaw}, FacingAngle: {Normaled_Facing}")
        else:
            SpeedZ = Error * KpW
        if abs(Error) < Settings.ExpectedVals.ErrorRange:
            self.Yaw_PID.reset()  # Reset PID when within acceptable error range
        WheelX = SpeedX * math.cos(math.radians(Yaw)) + SpeedY * math.sin(math.radians(Yaw))
        WheelY = - SpeedX * math.sin(math.radians(Yaw)) + SpeedY * math.cos(math.radians(Yaw))
        # Only print high-frequency motion logs when FullLog is enabled
        if getattr(self.cfg.Debug, "FullLog", False):
            self.logger.info(f"used FieldMoveVector: {WheelX, WheelY, SpeedZ}")
        self.RobotMoveVector(WheelX, WheelY, SpeedZ)
    
    def RobotXMove(self,Angle,Speed):
        self.RobotMoveVector(90,Angle,Speed)
   
    def RobotYMove(self,Angle,Speed,Kp=None):
        self.RobotMoveVector(0,Angle,Speed,Kp)

    def FieldTurn(self,Angle):
        if self.GetYaw is None:
            return False
        self.FieldMoveVector(0,0,Angle)

    def RobotTurn(self,Speed): #自转
        self.RobotMoveVector(0,0,Speed)
    
    def TurnTo(self,Angle,Kp=None):
        while True:
            Error = self.GetYaw() - Angle
            if not abs(Error) < 5:
                self.FieldMoveVector(0,0,Angle)
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
        self.logger.info("Used ElecMagnet")

    def Dribble(self,Status):
        if Status == True:
            self.SetIO(self.cfg.Ports.Dribble,1.0)
        elif Status == False:
            self.SetIO(self.cfg.Ports.Dribble,0.0)
        else:
            self.SetIO(self.cfg.Ports.Dribble,Status)
        self.DribbleStatus = Status
        self.logger.info("Dribble set to %s" % str(Status))
        # self.logger.info("Dribble set to %s" % ("ON" if Status else "OFF"))

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


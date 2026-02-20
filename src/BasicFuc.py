from LunaPre import *

def roundThresholdJudger(iValue, iRound, iMiddleValue, iOffset):
    iValue = iValue % iRound
    
    iLowerThreshold = (iMiddleValue - iOffset) % iRound
    iUpperThreshold = (iMiddleValue + iOffset) % iRound

    if iLowerThreshold <= iUpperThreshold:
        return iLowerThreshold <= iValue <= iUpperThreshold

    else:
        return iValue >= iLowerThreshold or iValue <= iUpperThreshold

def FindNearstAngle(arr, target):
    return min(arr, key=lambda x: abs(x - target))

def linear_map(value, input_range, output_range):
    """
    通用线性映射函数
    
    Args:
        value: 要映射的值
        input_range: 输入范围 (min, max)
        output_range: 输出范围 (min, max)
    
    Returns:
        映射后的值
    """
    input_min, input_max = input_range
    output_min, output_max = output_range
    
    # 计算映射
    input_span = input_max - input_min
    output_span = output_max - output_min
    
    scaled_value = (value - input_min) / input_span
    mapped_value = output_min + (scaled_value * output_span)
    
    return mapped_value

def ChasingBall():
    iBX,iBY = UnitedPosition.Robot_Ball_Pos()
    Compass = chassis.GetYaw()
    Angle = (math.degrees(math.atan2(iBX, iBY)) + 360) % 360
    AbsAngle = Angle + Compass
    Kp = 1.5
    SpeedX = iBX * Kp
    SpeedY = iBY * Kp
    chassis.AbsMoveVetor(SpeedX,SpeedY,AbsAngle) # 1.5 is a factor to make the robot turn faster, you can adjust it as needed

class LockSeries:
    def __init__(self):
        self.D_time = 0

    def Lockballangle(self):#贝尔巴托夫转身
        # iBX,iBY = UnitedPosition.AbsBallPos()#获取球的位置
        #  = lBallPos[0],lBallPos[1]#将球的位置赋值给iBX和iBY
        
        # Compass = chassis.GetYaw()#获取机器人的航向
        # Angle = (math.degrees(math.atan2(iBX, iBY))) % 360
        Angle = UnitedPosition.RelBallAngle()
        # Fangle = -(Angle - Compass)
        logger.debug("Ball Angle: %f" % Angle)
        chassis.AbsTurn(Angle)

    def Lockballmove(self):
        lBallPos = UnitedPosition.AbsBallPos()
        iBX,iBY = lBallPos[0],lBallPos[1]
        chassis.AbsMoveVetor(iBX*4,iBY*4,0)

    def LockBallSlip(self):
        iBX,iBY = UnitedPosition.Robot_Ball_Pos()
        if [iBX,iBY] == [0xfff,0xfff]:
            chassis.stop()
            return False
        elif [iBX,iBY] == [0xddd,0xddd]:
            chassis.stop()
            return True
        else:
            if iBY == 0:
                Fangle = 0
            else:
                Fangle = int(math.degrees(math.atan2(iBY,iBX)))
            if iBX > 75 or iBY > 75:
                Kp = 7
                KpZ = 1
            elif 25 < iBX <= 75 or 25 < iBY <= 75:
                Kp = 6
                KpZ = 1
            else:
                Kp = 4
                KpZ = 1.5
            # Kp = 3
            # KpZ = Fangle*self.D_time/500
            SpeedX = iBX * Kp
            SpeedY = iBY * Kp
            Fangle = Fangle * KpZ
            if SpeedX > 300:
                SpeedX = 300
            if SpeedY > 300:
                SpeedY = 300
            if SpeedX < -300:
                SpeedX = -300
            if SpeedY < -300:
                SpeedY = -300
            # print(SpeedX,SpeedY,Fangle) # 1.5 is a factor to make the robot turn faster, you can adjust it as needed
            chassis.RelMoveVetor(SpeedX*1.5,SpeedY,Fangle) # 1.5 is a factor to make the robot turn faster, you can adjust it as needed
            return False

lockball = LockSeries()

def NormalShoot(x=1):
    BX, BY = UnitedPosition.Robot_Ball_Pos()
    logger.debug("Current Position: %s" % [BX, BY])
    logger.debug("Ball Position: %s" % [BX, BY])
    if BX == 0 and BY == 0:
        logger.info("Ball not found, stopping chassis.")
        # Pos2Pos([0, -85, 0], False)
        # chassis.stop()
        UnitedPosition.Pos2Pos([0, -85, 0])
        peripheral.Dribble(False)
    elif BX == 0 and 5 < (BY) <= 9:
        # for _ in range(3):
        #     LockBallSlip()
        #     BX, BY = UnitedPosition.Relative_Ball_Position()
        #     if not BX == 0 and BY <= 10:
        #         break

        time.sleep(0.03)
        logger.info("Ball is at front")
        if x == 0:
            return True
        # BX, BY = UnitedPosition.Relative_Ball_Position()
        # if not -5 < BX < 5 and 0 < BY < 10:
        #     break
        peripheral.Dribble(True)
        for _ in range(10):
            BX,BY = UnitedPosition.Robot_Ball_Pos()
            if not BX == 0 and BY <= 9:
                break
            X,Y,_ = UnitedPosition.Field_Robot_Pos()
            DeltaY = cfg.read("Position","Height")/2 - Y
            DeltaX = X
            Theta = math.atan2(DeltaY, DeltaX)
            Theta = math.degrees(Theta)
            Aim = Theta - 90
            Compass = compass()
            Delta = abs(Aim - Compass)
        # while True:
        #     chassis.GoZ(Compass)
            if Delta > 60:
                chassis.AbsTurn(Aim,50)
                break
            else:
                chassis.AbsMoveAngle(0,Aim,50)
            
            time.sleep(0.03)
        
        peripheral.ShootBall()
    else:
        peripheral.Dribble(True)
        lockball().LockBallSlip()

# def CircleAround_old(iAimAngle):
# #    if abs(compass.read() - iAimAngle) >= 180:
#     iDirectionFactor = 1
# #    else:
# #        iDirectionFactor = -1
#     while(1):
#         lBallPos = GetBallPos()
#         iX, iY = lBallPos[0], lBallPos[1]
#         if iY > 0:
#             iDeltaAngle = -int(math.degrees(math.atan2(iY, iX)) - 90)     
#         print(iX,iY,iDeltaAngle,compass.read())
#         if roundThresholdJudger(compass.read(), 360, iAimAngle, 3):
#             break
#         else:
#             set_motor.RPM(iDirectionFactor * 30,-iDirectionFactor * (130 - iDeltaAngle),-iDirectionFactor * 30,iDirectionFactor * (130 - iDeltaAngle))
#     car.stop()

def CircleAround(iAimAngle):
    iCompass = int(compass())
    if roundThresholdJudger(iCompass,360,iAimAngle+90,90):
        iDirectionFactor = 1
    else:
        iDirectionFactor = -1
    lBallPos = UnitedPosition.Robot_Ball_Pos()

    TempBX = -lBallPos[1]
    TempBY = lBallPos[0]

    lBallPos = [TempBX,TempBY]


    while(1):
        lBallPos = UnitedPosition.Robot_Ball_Pos()

        TempBX = -lBallPos[1]
        TempBY = lBallPos[0]

        lBallPos = [TempBX,TempBY]


        iX, iY = lBallPos[0], lBallPos[1]
        if iY > 0:
            iDeltaAngle = -int(math.degrees(math.atan2(iY, iX)) - 90)     
        # print(iX,iY,iDeltaAngle,compass())
        if roundThresholdJudger(compass(), 360, iAimAngle, 3):
            break
        else:
            chassis.SetMotor([iDirectionFactor * 15,-iDirectionFactor * (150 - iDeltaAngle),-iDirectionFactor * 15,iDirectionFactor * (150 - iDeltaAngle)])
        time.sleep(.03)
    chassis.TurnTo(iAimAngle)
    while(1):
        lBallPos = UnitedPosition.Robot_Ball_Pos()

        TempBX = -lBallPos[1]
        TempBY = lBallPos[0]

        lBallPos = [TempBX,TempBY]

        iBX = lBallPos[0]
        if lBallPos[0] + lBallPos[1] > 20:
            break
        if iBX <= -Settings.ExpectedVals.ErrorRange/3:
            logger.debug('atleft')
            chassis.AbsMoveAngle(iAimAngle,iAimAngle + 90,40)
        elif iBX >= Settings.ExpectedVals.ErrorRange/3:
            logger.debug('atright')
            chassis.AbsMoveAngle(iAimAngle,iAimAngle + 90,-40)
        else:
            for _ in range(3):
                chassis.stop()
            break
        time.sleep(.03)
    print('stopped')

def TurnTo_with_ball(Aim_Angle,SpeedK:int | None = 1):
    yaw = chassis.GetYaw()
    ball = UnitedPosition.Robot_Ball_Pos()
    Aim_Angle = Aim_Angle%360
    logger.success(Aim_Angle)
    if lockball.LockBallSlip() == True:
        iDeltaAngle = -int(math.degrees(math.atan2(ball[0], ball[1])) - 90)
        Error = ( (yaw - Aim_Angle + 180) % 360 - 180 ) # Normalize to [-180, 180]
        if Error < 180:
            iDirectionFactor = -1
        elif Error >= 180:
            iDirectionFactor = 1
        if not abs(Aim_Angle-yaw) <= 10:
            logger.error(Error)
            chassis.SetMotor([
                iDirectionFactor * 15 * SpeedK *Error/30,
                -iDirectionFactor * (150 - iDeltaAngle) * SpeedK *Error/30,
                -iDirectionFactor * 15 * SpeedK *Error/30,
                iDirectionFactor * (150 - iDeltaAngle) * SpeedK *Error/30
                ])
            return False
        else:
            return True
    else:
        return lockball.LockBallSlip()


def GoBack():
    UnitedPosition.Pos2Pos(cfg.read("Position","Home"),False)


def NormalShoot(x=1,HomePos=[0,-20,0]):
    BX, BY = UnitedPosition.Robot_Ball_Pos()

    logger.debug("Ball Position: %s" % [BX, BY])
    if BX == 0 and BY == 0:
        logger.info("Ball not found, stopping chassis.")
        Positions.Pos2Pos(HomePos)
        peripheral.Dribble(False)
    elif BX == 0 and 5 < (BY) <= 9:
        time.sleep(0.03)
        logger.info("Ball is in front")
        if x == 0:
            return True
        for _ in range(3):
            NormalShoot(0)
            time.sleep(0.03)
        peripheral.Dribble(True)
        for _ in range(10):
            BX,BY = UnitedPosition.AbsBallPos()
            if not BX == 0 and BY <= 9:
                break
            X,Y,_ = UnitedPosition.Field_Robot_Pos()()
            DeltaY = cfg.Bounds.Long/2 - Y
            DeltaX = X
            Theta = math.atan2(DeltaY, DeltaX)
            Theta = math.degrees(Theta)
            Aim = Theta - 90
            Compass = compass()
            Delta = abs(Aim - Compass)
            if Delta > 60:
                chassis.AbsTurn(Aim,50)
                break
            else:
                chassis.AbsMoveVetor(Aim,0,50,0.8)
            
            time.sleep(0.03)
        
        peripheral.ShootBall()
    else:
        peripheral.Dribble(True)
        LockSeries().LockBallSlip()


def Slipsideshot(EnemyPos,GoalPos): #溜边 10,-90
    '''
    #### Yaw : 指南针 
    ##### <code>Yaw: int | [0,360]</code>
    #### LocalPos : 本地坐标 
    ##### <code>LocalPos: list | [[-80,80],[-100,100],[0,360]]</code>
    ### EnemyPos : 敌人坐标
    ### GoalPos : 球门坐标
    '''
    peripheral.Dribble(True)
    LocalPos = UnitedPosition.AbsBallPos()
    LocalX,LocalY,_ = LocalPos
    EnemyX,EnemyY = EnemyPos
    _ , GoalY = GoalPos
    GoalY = 80
    AimX  = 40
    AimY  = 80
    if LocalX > 0:
        k = 45
    else:
        AimX = -AimX
        k = -45
    Angle = (math.degrees(math.atan2(LocalX - EnemyX, LocalY - EnemyY)-270)) % 360
    UnitedPosition.Pos2Pos([AimX,AimY,Angle],False,200)
    if abs(AimX-LocalX)+abs(AimX-LocalX) < 5:
        DeltaY = LocalY - GoalY
        DeltaX = LocalX
        Theta =((math.degrees(math.atan2(DeltaX, DeltaY)-270)) % 360) + 180 +k
        Compass = Theta
        for _ in range(20):
            # if not AbsBallPos() == [1207, 1207]:
            #     break
            chassis.AbsTurn(Compass)
            time.sleep(0.03)
        peripheral.ShootBall()
        logger.success("Ball is in possession, start shotting.")

def OHMYBACK(HomePos=[0,-20,0]):
    #正常
    if UnitedPosition.AbsBallPos() == [0xddd,0xddd]:
        peripheral.Dribble(True)
        iLocX,iLocY,_ = UnitedPosition.Field_Robot_Pos()
        try:
            Cx ,Cy = UnitedPosition.Field_Chassis_Pos()
        except:
            Cx = 0
            Cy = 0
        if [Cx ,Cy] == [0,0]:
            if iLocX > 0:
                Angle = 90
            else:
                Angle = 270
        else:
            Fangle =(-int(math.degrees(math.atan2(Cy,Cx)) - 90)+ compass())%360
            CDistance = math.sqrt(Cx**2 + Cy**2)
            ChassisX = CDistance * math.sin(math.radians(Fangle))+iLocX
            ChassisY = CDistance * math.cos(math.radians(Fangle))+iLocY
            Angle = (math.degrees(math.atan2(iLocX - ChassisX, iLocY - ChassisY)-270)) % 360
        if iLocX > 0:
            ShootY = 65
        else:
            ShootY = -65
        if abs(ShootY - iLocX) < 5 :
            if abs(iLocY - 75) < 5:
                X,Y,_ = UnitedPosition.Field_Robot_Pos()()
                DeltaY = cfg.Bounds.Long/2 - Y
                DeltaX = X
                Theta = math.atan2(DeltaY, DeltaX)
                Theta = math.degrees(Theta)
                Aim = Theta - 90
                Compass = compass()
                deltaT = abs(Aim - Compass)
                if deltaT < 8:
                    peripheral.ShootBall()
                else:
                    chassis.stop()
                    time.sleep(0.5)
                    chassis.AbsTurn(Aim,50)
                    peripheral.ShootBall()
                
            else:
                UnitedPosition.Pos2Pos([85,ShootY,Angle],5)
        else:
            UnitedPosition.Pos2Pos([85,ShootY,Angle],5)
    else:
        NormalShoot(x=0,HomePos=HomePos)
        

"""BasicFuc

历史策略常用函数集合。

注意：本文件存在大量历史代码。此次改动目标：
- 修复明显的语法/运行时错误（重复定义、错误调用）
- 按 README 命名规范补齐 PascalCase 新名称，同时保留旧名称别名
- 避免把 `from LunaPre import *` 继续扩散到外部：本文件内部仍可使用，但对外建议显式导入
"""

from __future__ import annotations

from LunaPre import *

def RoundThresholdJudger(iValue, iRound, iMiddleValue, iOffset):
    iValue = iValue % iRound
    
    iLowerThreshold = (iMiddleValue - iOffset) % iRound
    iUpperThreshold = (iMiddleValue + iOffset) % iRound

    if iLowerThreshold <= iUpperThreshold:
        return iLowerThreshold <= iValue <= iUpperThreshold

    else:
        return iValue >= iLowerThreshold or iValue <= iUpperThreshold

def FindNearestAngle(arr, target):
    return min(arr, key=lambda x: abs(x - target))

def LinearMap(value, input_range, output_range):
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
    iBX,iBY = UnitedPosition.RobotBallPos()
    # Robot-frame pursuit: RobotBallPos() already returns (x_forward, y_left).
    # Use robot-series motion only to avoid mixing field-frame yaw compensation.
    if [iBX, iBY] == [0xfff, 0xfff]:
        chassis.stop()
        return
    if [iBX, iBY] == [0xddd, 0xddd]:
        peripheral.Dribble(True)
        return

    Kp = 1.5
    # 0°=forward, 90°=left, 180°=back, 270°=right
    ball_angle = (math.degrees(math.atan2(iBY, iBX)) + 360) % 360
    speed = min(300, int(math.sqrt(iBX * iBX + iBY * iBY) * Kp))
    chassis.RobotMoveAngle(ball_angle, speed)

class LockSeries:
    def __init__(self):
        self.D_time = 0

    def Lockballangle(self):#贝尔巴托夫转身
        # iBX,iBY = UnitedPosition.AbsBallPos()#获取球的位置
        #  = lBallPos[0],lBallPos[1]#将球的位置赋值给iBX和iBY
        
        # Compass = chassis.GetYaw()#获取机器人的航向
        # Angle = (math.degrees(math.atan2(iBX, iBY))) % 360
        Angle = UnitedPosition.RobotBallAngle()
        logger.debug("Ball Angle: %f" % Angle)
        # Robot-frame turn only; do not apply field yaw correction here.
        chassis.RobotTurn((Angle - 180) / 2)

    def Lockballmove(self):
        lBallPos = UnitedPosition.FieldBallPos()
        iBX,iBY = lBallPos[0],lBallPos[1]
        # Convert to robot-frame pursuit to avoid field/robot mixing.
        rBX, rBY = UnitedPosition.RobotBallPos()
        if [rBX, rBY] in ([0xfff, 0xfff], [0xddd, 0xddd]):
            return
        chassis.RobotMoveAngle((math.degrees(math.atan2(rBY, rBX)) + 360) % 360, min(300, int(math.hypot(rBX, rBY) * 4)))

    def LockBallSlip(self):
        iBX,iBY = UnitedPosition.RobotBallPos()
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
                KpZ = 3
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
            # Keep the entire slip logic in robot-frame motion.
            chassis.RobotMoveVector(SpeedX*1.5,SpeedY,Fangle) # 1.5 is a factor to make the robot turn faster, you can adjust it as needed
            return False

lockball = LockSeries()

def NormalShoot(x=1, HomePos=[0, -20, 0]):
    """兼容版 NormalShoot（保留旧名）。

    说明：历史代码里本函数被定义过两次，且包含明显错误（lockball()、FieldRobotPos()()）。
    现在统一为一个实现。
    """

    bx, by = UnitedPosition.RobotBallPos()
    logger.debug("Ball Position: %s" % [bx, by])

    # No ball
    if [bx, by] == [0xFFF, 0xFFF] or (bx == 0 and by == 0):
        logger.info("Ball not found, go home.")
        UnitedPosition.Pos2Pos(HomePos)
        peripheral.Dribble(False)
        return False

    # Catch ball
    if [bx, by] == [0xDDD, 0xDDD]:
        # already holding, caller decides what to do
        return True

    # Ball in front area (legacy heuristic)
    if bx == 0 and 5 < by <= 9:
        time.sleep(0.03)
        logger.info("Ball is in front")
        if x == 0:
            return True

        peripheral.Dribble(True)
        for _ in range(10):
            bx, by = UnitedPosition.RobotBallPos()
            if not (bx == 0 and by <= 9):
                break

            x0, y0, _ = UnitedPosition.FieldRobotPos()
            # fallback: use field center and bounds to compute aim
            delta_y = cfg.Bounds.Long / 2 - y0
            delta_x = x0
            theta = math.degrees(math.atan2(delta_y, delta_x))
            aim = theta - 90
            current = compass()
            delta = abs(aim - current)

            if delta > 60:
                chassis.FieldTurn(aim)
                break
            else:
                chassis.FieldMoveAngle(0, aim, 50)
            time.sleep(0.03)

        peripheral.ShootBall()
        return True

    peripheral.Dribble(True)
    return lockball.LockBallSlip()

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
    if RoundThresholdJudger(iCompass,360,iAimAngle+90,90):
        iDirectionFactor = 1
    else:
        iDirectionFactor = -1
    lBallPos = UnitedPosition.RobotBallPos()

    TempBX = -lBallPos[1]
    TempBY = lBallPos[0]

    lBallPos = [TempBX,TempBY]


    while(1):
        lBallPos = UnitedPosition.RobotBallPos()

        TempBX = -lBallPos[1]
        TempBY = lBallPos[0]

        lBallPos = [TempBX,TempBY]


        iX, iY = lBallPos[0], lBallPos[1]
        if iY > 0:
            iDeltaAngle = -int(math.degrees(math.atan2(iY, iX)) - 90)     
        # print(iX,iY,iDeltaAngle,compass())
        if RoundThresholdJudger(compass(), 360, iAimAngle, 3):
            break
        else:
            chassis.SetMotor([iDirectionFactor * 15,-iDirectionFactor * (150 - iDeltaAngle),-iDirectionFactor * 15,iDirectionFactor * (150 - iDeltaAngle)])
        time.sleep(.03)
    chassis.TurnTo(iAimAngle)
    while(1):
        lBallPos = UnitedPosition.RobotBallPos()

        TempBX = -lBallPos[1]
        TempBY = lBallPos[0]

        lBallPos = [TempBX,TempBY]

        iBX = lBallPos[0]
        if lBallPos[0] + lBallPos[1] > 20:
            break
        if iBX <= -Settings.ExpectedVals.ErrorRange/3:
            logger.debug('atleft')
            chassis.FieldMoveAngle(iAimAngle,iAimAngle + 90,40)
        elif iBX >= Settings.ExpectedVals.ErrorRange/3:
            logger.debug('atright')
            chassis.FieldMoveAngle(iAimAngle,iAimAngle + 90,-40)
        else:
            for _ in range(3):
                chassis.stop()
            break
        time.sleep(.03)
    print('stopped')

def TurnTo_with_ball(Aim_Angle,SpeedK:int | None = 1):
    yaw = chassis.GetYaw()
    ball = UnitedPosition.RobotBallPos()
    Aim_Angle = Aim_Angle%360
    logger.success(Aim_Angle)
    if lockball.LockBallSlip() == True:
        # Robot-frame ball angle (0=forward)
        iDeltaAngle = int(math.degrees(math.atan2(ball[1], ball[0])))
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
    # config.json currently stores Home under CheckPoints
    try:
        home = [cfg.CheckPoints.Home[0], cfg.CheckPoints.Home[1], cfg.CheckPoints.Home[2]]
    except Exception:
        home = [0, -20, 0]
    UnitedPosition.Pos2Pos(home)

# ========= Backward-compatible aliases =========

# old names -> new names
roundThresholdJudger = RoundThresholdJudger
FindNearstAngle = FindNearestAngle
linear_map = LinearMap



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
    LocalPos = UnitedPosition.FieldBallPos()
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
            chassis.FieldTurn(Compass)
            time.sleep(0.03)
        peripheral.ShootBall()
        logger.success("Ball is in possession, start shotting.")

def OHMYBACK(HomePos=[0,-20,0]):
    #正常
    if UnitedPosition.FieldBallPos() == [0xddd,0xddd]:
        peripheral.Dribble(True)
        iLocX,iLocY,_ = UnitedPosition.FieldRobotPos()
        try:
            Cx ,Cy = UnitedPosition.FieldChassisPos()
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
                X,Y,_ = UnitedPosition.FieldRobotPos()
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
                    chassis.FieldTurn(Aim)
                    peripheral.ShootBall()
                
            else:
                UnitedPosition.Pos2Pos([85,ShootY,Angle],5)
        else:
            UnitedPosition.Pos2Pos([85,ShootY,Angle],5)
    else:
        NormalShoot(x=0,HomePos=HomePos)
        

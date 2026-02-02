from BasicFuc import *

try:
    # peripheral.Dribble(1)
    # time.sleep(3)
    while(1):
        if UnitedPosition.Restricted_Area():
            BallPosition = UnitedPosition.Relative_Ball_Position()
            RoboPosition = UnitedPosition.AbsRoboPosition()
            yaw = chassis.GetYaw()
            # print("BalllPosition:",BalllPosition)
            # time.sleep(1)
            goal_angle_deep = UnitedPosition.Local2Angle([230,0])+180
            goal_angle_home = UnitedPosition.Local2Angle([100,0])
            if BallPosition == [0xfff,0xfff]:
                # peripheral.Dribble(0.7)
                UnitedPosition.Pos2Pos([Settings.CheckPoints.Home[0]+20,Settings.CheckPoints.Home[1],0],4.5)
            elif BallPosition == [0xddd,0xddd]:
                if RoboPosition[0] < 70:
                    if TurnTo_with_ball((goal_angle_deep)%360,0.6):
                        chassis.RelMoveAngle(0,100)
                        peripheral.ShootBall()
                        logger.warning("Processing :27")
                else:
                    peripheral.Dribble(0.2)
                    if TurnTo_with_ball((goal_angle_home)%360,1):
                        peripheral.Dribble(0.2)
                        chassis.RelMoveAngle(0,300)
                        time.sleep(.2)
                        peripheral.ShootBall()
                        logger.warning("Processing :45")
                    else:
                        logger.warning("Processing :47")
                         
                    chassis.stop()
                    time.sleep(2)
            else:
                logger.warning("Processing :44")
                # peripheral.Dribble(0.7)
                lockball.LockBallSlip()
        time.sleep(.03)
except Exception as e:
    raise e
finally:
    UnitedPosition.stop()

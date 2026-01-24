from kits import *

def NormalShoot(x=1):
    BX, BY = GetBallPos()
    logger.debug("Current Position: %s" % [BX, BY])
    logger.debug("Ball Position: %s" % [BX, BY])
    if BX == 0 and BY == 0:
        logger.info("Ball not found, stopping chassis.")
        # Pos2Pos([0, -85, 0], False)
        # chassis.stop()
        Pos2Pos([0, -85, 0])
        peripheral.Dribble(False)
    elif BX == 0 and 5 < (BY) <= 9:
        # for _ in range(3):
        #     LockBallSlip()
        #     BX, BY = GetBallPos()
        #     if not BX == 0 and BY <= 10:
        #         break

        time.sleep(0.03)
        logger.info("Ball is in front")
        if x == 0:
            return True
        # BX, BY = GetBallPos()
        # if not -5 < BX < 5 and 0 < BY < 10:
        #     break
        peripheral.Dribble(True)
        for _ in range(10):
            BX,BY = GetBallPos()
            if not BX == 0 and BY <= 9:
                break
            X,Y,_ = GetPos()
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
                chassis.Turn(Aim,50)
                break
            else:
                chassis.GoA(Aim,0,50,0.8)
            
            time.sleep(0.03)
        
        peripheral.ShootBall()
    else:
        peripheral.Dribble(True)
        LockBallSlip()



while True:
    try:
        # Lockballslip()
        NormalShoot()
        # OHMYBACK()
        print(GetBallPos())
        time.sleep(.03)
    except KeyboardInterrupt:
        print("Exiting...")
        chassis.stop()
        peripheral.Dribble(False)
        break
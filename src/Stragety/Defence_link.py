from kits import *
def DefenceLink(x):#"0":[75,95],"1":[35,85]
    #Comlpeted
    time.sleep(0.03)
    lBallPos = GetBallPos()
    iBX,iBY = lBallPos[0],lBallPos[1]
    Compass = compass()
    Angle = (math.degrees(math.atan2(iBX, iBY)) + 360) % 360 
    Fangle = Angle + Compass
    bX, bY = AbsBallPos()
    logger.debug("bX"+str(bX)+" bY"+str(bY))
    IX = GetPos()[0]
    IY = GetPos()[1]
    if x == 1:
        defend_x = 0
        defend_y = -80
        if [bX,bY] == [1024,1024]: #没有扫描到球 那就锁定车辆位置
            defend_x = 0
            defend_y = -80
            Cx = GetEnemyPos()[0]
            Cy = GetEnemyPos()[1]
            defend_y = 0
            Fangle =(-int(math.degrees(math.atan2(Cy,Cx)) - 90)+ compass())%360
            CDistance = math.sqrt(Cx**2 + Cy**2)
            ChassisX = CDistance * math.sin(math.radians(Fangle))+IX
            ChassisY = CDistance * math.cos(math.radians(Fangle))+IY

            logger.debug("CX"+str(ChassisX)+" CY"+str(ChassisY))

            peripheral.StopDribble()
            if ChassisY == -95 :
                ABSx = ChassisX  * 10
            elif ChassisY < -80:
                ABSx = ChassisX 
            else:
                ABSx = ChassisX / (95+ChassisY) * 15
            
            if abs(ABSx) <= 35:
                defend_x = ABSx
                defend_y = -80
            elif abs(ABSx) > 35 and abs(ABSx) < 42:
                defend_x = ABSx
                defend_y = -80 - (abs(ABSx)-35)
            else:
                if ChassisX > 0:
                    defend_x = 42
                else:
                    defend_x = -42
                defend_y = -95 + (80+ChassisY)/abs(ChassisX) * 45
        else: #有球锁球
            peripheral.DribbleBall()
            if bY == -95:
                ABSx = bX  * 10
            else:
                ABSx = bX / (95+bY) * 10
            print(ABSx)
            if abs(ABSx) <= 35:
                defend_x = ABSx
                defend_y = -80
            elif abs(ABSx) > 35 and abs(ABSx) < 42:
                defend_x = ABSx
                defend_y = -80 - (abs(ABSx)-35)
            else:
                if bX > 0:
                    defend_x = 42
                else:
                    defend_x = -42
                defend_y = -95 + (80+bY)/abs(bX) * 45
                logger.debug("DX"+str(defend_x)+" DY"+str(defend_y))
    else:
        defend_x = 0
        defend_y = -85
        if [bX,bY] == [1024,1024]: 
            peripheral.StopDribble()
            defend_x = 0
            defend_y = -85
            Fangle = 0
        else: 
            peripheral.DribbleBall()
            if bY == -95:
                ABSx = bX  * 10
            elif bY <= -85:
                ABSx = bX
            else:
                ABSx = bX / (95+bY) * 10
            print(ABSx)
            if abs(ABSx) <= 35:
                defend_x = ABSx
                defend_y = -85
            elif abs(ABSx) > 35 and abs(ABSx) < 42:
                defend_x = ABSx
                defend_y = -85 - (abs(ABSx)-35)
            else:
                if bX > 0:
                    defend_x = 42
                else:
                    defend_x = -42
                defend_y = -95 + (85+bY)/abs(bX) * 45

    if abs(defend_x - IX) <10 and abs(defend_y - IY)<10:
        chassis.stop()
        chassis.GoZ(Fangle)
    else:
        Pos2Pos([defend_x, defend_y, Fangle], False,200)

from kits import *
def FollowPRatio(x):
    #Completed
    time.sleep(0.03)

    defend_x = 0
    defend_y = 0
    bX,bY= AbsBallPos()
    lPos = GetPos()
    iX= lPos[0]
    iY= lPos[1]
    Cx = GetEnemyPos()[0]
    Cy = GetEnemyPos()[1]
    defend_y = 0
    Fangle =(-int(math.degrees(math.atan2(Cy,Cx)) - 90)+ compass())%360
    CDistance = math.sqrt(Cx**2 + Cy**2)
    ChassisX = CDistance * math.sin(math.radians(Fangle))+iX
    ChassisY = CDistance * math.cos(math.radians(Fangle))+iY
    if x == 1:
        PeerX, PeerY = PeerPosition[0], PeerPosition[1]
        if [bX,bY] == [1024,1024]:
            defend_x = 0
            defend_y = -85

            peripheral.DribbleBall()
            if abs(iX) <= 60:
                defend_x = ChassisX 
                defend_y = -40 + ChassisY * 0.47  # 等比前移
                defend_y = max(-85, min(0, defend_y))
            else:
                defend_x = 0    
                defend_y = -40 + ChassisY * 0.47  # 等比前移
                defend_y = max(-85, min(0, defend_y))
        else:
            peripheral.DribbleBall()
            if abs(iX) <= 60:
                defend_x = PeerX  
                defend_y = -40 + PeerY * 0.47  # 等比前移
                defend_y = max(-85, min(0, defend_y))
            else:
                defend_x = 0    
                defend_y = -40 + PeerY * 0.47  # 等比前移
                defend_y = max(-85, min(0, defend_y))
    else:
            lBallPos = GetBallPos()
            iBX,iBY = lBallPos[0],lBallPos[1]
            Compass = compass()
            Angle = (math.degrees(math.atan2(iBX, iBY)) + 360) % 360
            Fangle = Angle + Compass
            peripheral.DribbleBall()
            if [AbsBallPos[0],AbsBallPos[1]] == [1024,1024]:
                defend_x = 0
                defend_y = -80
                Fangle = 0
            else:
                if abs(iX) <= 60:
                    defend_x = bX  
                    defend_y = -80
                else:
                    defend_x = 0    
                    defend_y = -80
    Pos2Pos([defend_x, defend_y, Fangle],True,200)




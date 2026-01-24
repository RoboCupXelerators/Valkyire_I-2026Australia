from kits import *
def FollowPPanning(x):
    #Completed
    time.sleep(0.03)
    defend_x = 0
    defend_y = 0
    bX,bY= AbsBallPos()
    lPos = GetPos()
    iX= lPos[0]
    if x == 1 or GetEnemyPos() == [1024,1024]:
        PeerX = PeerPosition[0]
        ChassisX = GetEnemyPos()[0]
        if [bX,bY] == [1024,1024]:
                peripheral.DribbleBall()
                Fangle = AbsChassisAngle()
                if abs(iX) <= 60:
                    defend_x = ChassisX 
                    defend_y = -85
                else:
                    defend_x = 0    
                    defend_y = -85
        else:
            peripheral.DribbleBall()
            if abs(iX) <= 60:
                defend_x = -PeerX  
                defend_y = -85
            else:
                defend_x = 0    
                defend_y = -85
    else:
        lBallPos = GetBallPos()
        iBX,iBY = lBallPos[0],lBallPos[1]
        Compass = compass()
        Angle = (math.degrees(math.atan2(iBX, iBY)) + 360) % 360
        Fangle = Angle + Compass
        if [bX,bY] == [1024,1024]:
                peripheral.DribbleBall()
                if abs(iX) <= 60:
                    defend_x = bX 
                    defend_y = -85
                else:
                    defend_x = 0    
                    defend_y = -85
        else:
            peripheral.DribbleBall()
            if abs(iX) <= 60:
                defend_x = bX  
                defend_y = -85
            else:
                defend_x = 0    
                defend_y = -85
    Pos2Pos([defend_x, defend_y, Fangle], 400)




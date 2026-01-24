from kits import *
from Defence_Back import *
from Defence_Follow_panning import *
from Defence_Follow_ratio import *
from Defence_link import *
from Offence_Back import *
# ArisuCam = ArisuIntelligence()

#Math Mod
###################################################################################################

def OD(): 
    iX,iY = GetPos()[0],GetPos()[1]
    bx,by = AbsBallPos()
    Cx = GetEnemyPos()[0]
    Cy = GetEnemyPos()[1]
    Fangle =(-int(math.degrees(math.atan2(Cy,Cx)) - 90)+ compass())%360
    CDistance = math.sqrt(Cx**2 + Cy**2)
    ChassisY = CDistance * math.cos(math.radians(Fangle))+iY
    if abs(iX) <35 and abs(iY) <85: #防止出界限
        chassis.SetMotor(100,100,100,100)
    if abs(iX) > 75 : 
        print(1)
        if iX>0:
            chassis.SetMotor(-100,100,100,-100)
        else:
            chassis.SetMotor(100,-100,-100,100)
    elif abs(iY) > 95:
        print(2)
        if iY > 0:
            chassis.SetMotor(-100,-100,-100,-100)
        else:
            chassis.SetMotor(100,100,100,100)




    else:
        if PeerPosition == [1024,1024]:
            if [bx,by] == [1024,1024]:
                if ChassisY > 0 or GetEnemyPos() == [1204,1204]: #对方车辆位于对方半场
                    if PeerPosition[1]> iY :
                        print(2)
                        Pos2Pos([0,-40,0],False,100)
                    else:
                        Pos2Pos([0,-85,0],False,100)
                else: #有可能是背身持球 没有扫描到球，锁车
                    if ChassisY > -50:
                        print(3.25)
                        FollowPRatio(0)
                    else:
                        print(3.75)
                        DefenceLink(1)
            else:
                if BallFlag == [1,0]:
                    print(4)
                    Offence()
                else:
                    if bx > -50:
                        print(8)
                        FollowPRatio(0)
                    else:
                        print(9)
                        DefenceLink(1)
        else:
            if [bx,by] == [1024,1024]:
                if ChassisY > 0 or GetEnemyPos() == [1204,1204]: #对方车辆位于对方半场
                    if PeerPosition[1]> iY :
                        print(2)
                        Pos2Pos([0,-85,0],False,100)
                    else:
                        Pos2Pos([0,-40,0],False,100)
                else: #有可能是背身持球 没有扫描到球，锁车
                    if ChassisY > -50:
                        print(3.25)
                        FollowPRatio(1)
                    else:
                        print(3.75)
                        DefenceLink(1)
            else:
                if by > 0:
                    if BallFlag == [1,0]:
                        print(4)
                        Offence()
                    elif BallFlag == [0,1]:
                        print(5)
                        FollowPRatio(0)
                    else:
                        print(6)
                        NormalShoot(0)
                else:
                    if BallFlag == [1,0]:
                        print(7)
                        Offence()
                    elif BallFlag == [0,1]:
                        if bx > -50:
                            print(8)
                            FollowPRatio(0)
                        else:
                            print(9)
                            DefenceLink(1)
                    else:
                        print(10)
                        NormalShoot(0)

def OffenceNew():
    AbsBX,AbsBY = AbsBallPos()
    if abs(AbsBX) > 50:
        OHMYBACK()
    else:
        NormalShoot()

def Offence(HomePos):
    iX = GetPos()[0]
    iY = GetPos()[1]
    if abs(iX) > 35:
        OHMYBACK(HomePos)
    else:
        NormalShoot(HomePos)

def OffDenfence():
    bx,by = AbsBallPos()
    if [bx,by] == [1024,1024]:
        DefenceBack()
    else:
        if [bx,by] == [1207,1207]:
            Offence()
        else:
            if by > -20:
                LockBallSlip()
            else:
                if bx > -50:
                    FollowPRatio(0)
                else:
                    DefenceLink(0)



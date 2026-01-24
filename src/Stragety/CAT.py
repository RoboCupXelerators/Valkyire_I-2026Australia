from kits import *

border = 130

def getadc():
    global iTof1,iTof0,iTof2,iTof3,GreyLeft,GreyBack,GreyRight,bX,bY,fx,fy

    ballX,ballY = GetBallPos()
    if not [ballX,ballY] == [1024,1024]:
        if ballY > 0:
            fx = ballX
            fy = ballY
            bX,bY = 0,0
        else:
            bX = ballX
            bY = ballY
            fx,fy = 0,0
    else:
        bY,bY,fx,fy = 0,0,0,0
    iTof0,iTof1,iTof2,iTof3 = GetDistance()
    GreyBack,GreyLeft,GreyRight = AVO()

def AVO():
    if iTof2 <= 40 and (iTof1 > 40 or iTof3 > 40):
        GreyBack = False
    else:
        GreyBack = True
    if iTof1 < 20:
        GreyRight = False
    else:
        GreyRight = True
    if iTof3 < 20:
        GreyLeft = False
    else:
        GreyLeft = True
    return GreyBack,GreyLeft,GreyRight

def kick_ball(status):
    #弹射
    peripheral.ShootBall()


def move(x,y):    #xy方向移动
    spd=math.sqrt(x**2+y**2)
    chassis.GoV(int(spd*math.sin(math.atan2(y,x)-math.radians(45))),int(spd*math.cos(math.atan2(y,x)-math.radians(45))))
#    print("spdx=%d,spdy=%d"%(int(spd*math.cos(math.atan2(y,x)-math.radians(45))),int(spd*math.sin(math.atan2(y,x)-math.radians(45)))))

def wait(s):
    time.sleep(int(s))

def turn_to(degree):
    chassis.Turn(degree,100)

def go_back():
    Pos2Pos([0,0,0])

def offense():
    if(fy>0):
        if(fx>0 or fx<0):
            if(iTof1<50):
                chassis.GoY(30,1000)
            elif(iTof2<50):
                chassis.GoY(330,1000)
            else:
                chassis.GoY(100)
            peripheral.Dribble(1)
            kick_ball(1)
            chassis.stop()
            peripheral.Dribble(0)
            kick_ball(0)
        elif(fx<=0):
            if(iTof3<44):
                chassis.GoX(-300)
            else:
                chassis.GoX(0,-300)
        elif(fx>=0):
            if(iTof3<44):
                chassis.GoX(300)
            else:
                chassis.GoX(-300,0)
    else:
        move((fx-160)*6,(240-fy)*4)

def defense():
    if(bY>0):
        if(bX>3 and bY<0):
            chassis.GoY(0)
        elif(bX<=3):
            chassis.GoY(150)
        elif(bX>=3):
            chassis.GoY(-150)
    else:
        move((160-bX)*7,(bY-240)*4)

def follow():
    turn_to(0)
    if(fx or fy):
        if (10<iTof0<44):
            if (iTof3>=border):
                global IrCount
                if (iTof1 > iTof2):
                    k = 1
                if (iTof1 < iTof2):
                    k = -1
                chassis.GoA(0,90,k*IrCount*10)
                IrCount = IrCount + 1
        if(iTof0<=44):
            if(fx<=-3 and iTof1>border):
                #如果球在左边
                chassis.GoY(-150)
                turn_to(90+iAngle)
                iAngle = iAngle + 1
            elif(fx>=3 and iTof3>border):
                #如果球在右边
                chassis.GoX(150)
                turn_to(90-iAngle)
                iAngle = iAngle + 1
            elif(not GreyLeft and not GreyRight and not GreyBack):
                #离开禁区
                chassis.GoY(80)
            else:
                global IrCount
                IrCount = 1
                chassis.GoZ(0)
        elif(iTof1<=border or iTof2<=border):
            if(fx<=-3 or fx>=3 and fy==9):
                chassis.GoY(0)
            elif(fx<=-3 or fx>=3):
                chassis.GoY(150)
            else:
                offense()
        else:
            offense()

    else:
        if(iTof3<=44):
            if(bX>=150 and iTof1>border):
                chassis.GoX(-150)
            elif(bX<=75 and iTof2>border):
                chassis.GoX(150)
            else:
                chassis.GoZ(0)
        elif(iTof1<=border or iTof2<=border):
            if(bX<=75 or bX>=150 and bY>130):
                chassis.GoY(0)
            elif(bX<=75 or bX>=150):
                chassis.GoY(-160)
            else:
                defense()
        else:
            defense()


while(True):
    if bX<13 and bY<13:
        go_back()
    else:
        follow()
    





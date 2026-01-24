from kits import *

def DefenceBack():
    bX,bY =  AbsBallPos()[0],AbsBallPos()[1]
    PeerY = Peerstatus()[3][1]
    lPos = GetPos()
    iY = lPos[1]
    if [bX,bY] == [1024,1024]:
        DBackPos = [0, -80, 0]
    else:
        if bY <= 0:
            if  iY > PeerY:
                DBackPos = [20, -80, 0]
            else:
                DBackPos = [-20, -80, 0]
        else:
            DBackPos = [0, -80, 0]
    Pos2Pos(DBackPos, False, 200)
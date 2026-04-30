from BasicFuc import *
import math

try:
    while not key():
        time.sleep(1)
    UnitedPosition.Move2Path([[-70, 30, 0]], 0)
    while not lockball.LockBallSlip():
    peripheral.Dribble(0.22)
    UnitedPosition.Move2Path([[-40, 30, 45], [-40, 45, 45], [-30, 50, 30], [0, 55, 0], [30, 50, -30], [40, 45, -45], [55, 40, -45]], 0.5)
    peripheral.ShootBall()
except:
    raise
finally:
    UnitedPosition.Stop()
    pass

from LunaPre import *
from BasicFuc import lockball, TurnTo_with_ball
import math
if __name__ == "__main__":
    try:
        while True:
            UnitedPosition.Move2Path([[-17,17,0],[17,17,0],[-36,-66,0]])
    finally:
        UnitedPosition.Stop()
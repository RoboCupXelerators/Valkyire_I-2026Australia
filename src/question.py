from BasicFuc import *

try:
    points_list = [
                    [0,-80,0],
                    [-100,-80,0],
                    
                    [-80,0,0],
                   [-100,80,0],
                   
                   [0,80,0],
                   [100,80,0],

                   [80,0,0],
                   [100,-80,0]
                   ]
    points_cache = []
    for _ in range(3):
        print(UnitedPosition.AbsRoboPosition())
        time.sleep(1)
    Position_Point = UnitedPosition.AbsRoboPosition()
    Position_Point = [100,80,0]
    cache = 0
    for i in range(len(points_list)):
        if (Position_Point[0]-points_list[i][0]) < 10 and (Position_Point[1]-points_list[i][1]) < 10:
            cache = i
            break
        else:
            continue
    for e in range(int(len(points_list)/2)+1):
        points_cache.append(points_list[i%len(points_list)])
        print(i%(len(points_list)))
        i += 1
        
    # points_cache.reverse()
    points_cache.append([0,0,0])
    UnitedPosition.Move2Path(points_cache,7)
    print(points_cache)
except Exception as e:
    raise e
finally:
    peripheral.Dribble(False)
    chassis.stop()
    UnitedPosition.stop()
    chassis.stop()

from kits import GetPos,logger
import time
cache = GetPos()

while(1):
    time.sleep(0.1)
    cache2 = GetPos()
    logger.debug("Error X: %s,Error Y: %s" % (int(cache[0] - cache2[0]),int(cache[1] - cache2[1])))
    logger.debug(" X: %s, Y: %s" % (int(cache2[0]),int(cache2[1])))
    cache = cache2
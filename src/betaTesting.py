from LunaPre import *
#长260
try:
    while(1):
        rob = UnitedPosition.AbsRoboPosition()
    
except Exception as e:
    raise e

finally:
    if Settings.Debug.Beta == True:
        snap2 = tracemalloc.take_snapshot()
        stats = snap2.compare_to(snap_start, 'lineno')

        import pandas as pd

        data = []
        for stat in stats:
            tb = stat.traceback[0] if stat.traceback else None
            data.append({
                "file": tb.filename if tb else "unknown",
                "line": tb.lineno if tb else 0,
                "size": stat.size,
                "count": stat.count
            })
        df = pd.DataFrame(data)
        from ReasonData import DATA_DIR
        df.to_csv(str(DATA_DIR / "tracemalloc.csv"), index=False)

    chassis.stop()
    lidar.stop()
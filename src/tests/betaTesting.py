from LunaPre import *
#长260
try:
    while(1):
        rob = UnitedPosition.AbsRoboPosition()
        ball = UnitedPosition.Relative_Ball_Position()
        time.sleep(0.1)
        # print("rob:",rob,"ball:",ball)
    
except Exception as e:
    raise e

finally:
    UnitedPosition.stop()
    chassis.stop()

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
from BasicFuc import *
from LunaPre import Settings, UnitedPosition, chassis, logger, peripheral
from LunaPre.strategy_tools import safe_stop

def safe_stop(chassis, peripheral=None, position=None) -> None:
    if peripheral is not None:
        try:
            peripheral.Dribble(False)
        except Exception:
            pass
    try:
        chassis.stop()
    except Exception:
        pass
    if position is not None:
        try:
            position.Stop()
        except AttributeError:
            try:
                position.stop()
            except Exception:
                pass
        except Exception:
            pass

def logic_step():
    if lockball.LockBallSlip():
        logger.info("LockBallSlip executed successfully.")
        pose = UnitedPosition.FieldRobotPose()
        Condition_1 = [70,30]
        if abs(pose[1]) > Condition_1[1]:
            OhMyBackStep()
        else:
            # NormalShoot(1)
            pass
    else:
        if UnitedPosition.RobotBallPos() == [0xFFF, 0xFFF]:
            logger.info("Ball position is [0xFFF, 0xFFF], moving to home position.")
            UnitedPosition.Pos2Pos(Settings.CheckPoints.Home)


if __name__ == "__main__":
    try:
        while True:
            logic_step()
    except KeyboardInterrupt:
        logger.info("Program interrupted by user.")
    finally:
        safe_stop(chassis, peripheral, UnitedPosition)
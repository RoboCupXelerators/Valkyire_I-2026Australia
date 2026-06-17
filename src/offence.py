from BasicFuc import *
import os
import time


BALL_NOT_FOUND = [0xFFF, 0xFFF]
BALL_HELD = [0xDDD, 0xDDD]

OFFENCE_DT = float(os.environ.get("OFFENCE_DT", "0.03"))
ATTACK_DIR = 1 if float(os.environ.get("OFFENCE_ATTACK_SIGN", "1")) >= 0 else -1
OHMYBACK_Y = float(os.environ.get("OFFENCE_OHMYBACK_Y", "45"))
SLIP_Y = float(os.environ.get("OFFENCE_SLIP_Y", "26"))
QUICK_SHOT_X = float(os.environ.get("OFFENCE_QUICK_SHOT_X", "70"))
TRICK_CYCLE = float(os.environ.get("OFFENCE_TRICK_CYCLE", "1.8"))

started_at = time.time()
state = "boot"


def OffenceStep():
    global state

    ball = UnitedPosition.RobotBallPos()
    home = list(Settings.CheckPoints.Home)
    while len(home) < 3:
        home.append(0)
    search_home = [home[0] + 20, home[1], 0]

    if ball == BALL_NOT_FOUND:
        peripheral.Dribble(0.7)
        UnitedPosition.Pos2Pos(search_home, 4.5)
        state = "search_ball"
        return state

    if ball != BALL_HELD:
        logger.warning(1)
        peripheral.Dribble(0.7)
        lockball.LockBallSlip()
        state = "lock_ball"
        return state

    goal_angle = UnitedPosition.Local2Angle([ATTACK_DIR * 110, 0]) % 360

    try:
        x, y, _ = UnitedPosition.FieldRobotPos()
    except Exception:
        x, y = 0, 0

    progress = ATTACK_DIR * float(x)
    trick_slot = int((time.time() - started_at) / max(TRICK_CYCLE, OFFENCE_DT))

    if abs(float(y)) >= OHMYBACK_Y:
        if OhMyBackStep(HomePos=search_home, attack_dir=ATTACK_DIR):
            state = "oh_my_back_shot"
        else:
            state = "oh_my_back"
        return state

    if progress >= QUICK_SHOT_X:
        if TurnTo_with_ball(goal_angle):
            peripheral.ShootBall()
            chassis.stop()
            state = "quick_shot"
        else:
            state = "turn_to_goal"
        return state

    if abs(float(y)) >= SLIP_Y and trick_slot % 2 == 0:
        enemies = UnitedPosition.FieldChassisPos()
        enemy = enemies[0][:2] if enemies and len(enemies[0]) >= 2 else [0, 0]
        Slipsideshot(enemy)
        state = "slip_side"
        return state

    if trick_slot % 3 == 0:
        lockball.Lockballangle()
        state = "belbatov_turn"
        return state

    peripheral.Dribble(0.8)
    chassis.FieldMoveVector(45 * ATTACK_DIR, -float(y) * 1.2, goal_angle)
    state = "feint_drive"
    return state


def OffenceStop():
    global state
    peripheral.Dribble(False)
    chassis.stop()
    try:
        UnitedPosition.Stop()
    except AttributeError:
        UnitedPosition.stop()
    chassis.stop()
    state = "stopped"


def main():
    try:
        while True:
            OffenceStep()
            time.sleep(OFFENCE_DT)
    finally:
        OffenceStop()


if __name__ == "__main__":
    main()

from LunaPre import *
from BasicFuc import lockball, TurnTo_with_ball
import math


def _read_state():
    """Read a fresh robot pose and ball state for the current control cycle."""
    robot_pos = UnitedPosition.FieldRobotPose()
    ball_pos = UnitedPosition.RobotBallPos()
    ball_state = UnitedPosition.BallState(ball_pos)
    side = 1 if robot_pos[1] > 0 else -1
    return robot_pos, ball_state, side


def Macon_kick():
    robot_pos, ball_state, side = _read_state()
    if ball_state == "held":
        peripheral.Dribble(0.7)
        TurnTo_with_ball(90 * side)
        threshold = [[40, 30 * side], [90, 80 * side]]
        if UnitedPosition.PointInShape(robot_pos, threshold):
            if abs((robot_pos[2] + 50*side)%360) < 10:
                peripheral.ShootBall()
                return True
            else:
                chassis.FieldTurn(40 * side)
            return False
        else:
            UnitedPosition.Pos2Pos([56, 75 * side, 90])
            return False
    else:
        return False

def back_kick():
    robot_pos, ball_state, side = _read_state()
    if ball_state == "held":
        peripheral.Dribble(0.7)
        back_point = 80
        if robot_pos[0] > back_point:
            aim = [121.5, -30 * side]
            goal = math.degrees(math.atan2(aim[1] - robot_pos[1],aim[0] - robot_pos[0],)) % 360
            if abs(robot_pos[2] - goal) > 8:
                TurnTo_with_ball(goal)
            else:
                peripheral.ShootBall()
                return True
            return False
        else:
            UnitedPosition.Pos2Pos([back_point, robot_pos[1], robot_pos[2]])
    else:
        return False

def IsOpenGoal():
    window = UnitedPosition.ShootWindow(
        target="opponent",
        use_estimated_goal=False,
        blocker_margin_deg=3.0,
    )
    return (
        window["goal_key"] is not None
        and window["window_open"]
        and len(window["blockers"]) == 0
    )

def OpenGoalShoot():
    robot_pos = UnitedPosition.FieldRobotPose()
    window = UnitedPosition.ShootWindow(
        target="opponent",
        use_estimated_goal=False,
        current_yaw_deg=robot_pos[2],
    )

    if not IsOpenGoal():
        target_yaw = window.get("target_yaw_deg")
        if target_yaw is not None:
            TurnTo_with_ball(target_yaw)
        return False

    if window["can_shoot_now"]:
        peripheral.ShootBall()
        return True

    target_yaw = window.get("target_yaw_deg")
    if target_yaw is not None:
        TurnTo_with_ball(target_yaw)
    return False

def defence():#防守机
    robot_pos, ball_state, side = _read_state()
    if ball_state == "held":
        if robot_pos[2] < 90 or robot_pos[2] > 270:
            chassis_positions = UnitedPosition.FieldChassisPos()
            if chassis_positions and chassis_positions[0][1] < robot_pos[1]:
                peripheral.Dribble(0.7)
                peripheral.ShootBall()
            else:
                peripheral.Dribble(0.7)
                TurnTo_with_ball(0)
                peripheral.ShootBall()
        else:
            peripheral.Dribble(0.7)
            if UnitedPosition.PointInShape(robot_pos, [[-80, 75], [-90, -75]]):
                chassis.FieldTurn(150 * side)
                peripheral.ShootBall()
            else:
                chassis.FieldTurn(0)
                peripheral.ShootBall()
    else:
        UnitedPosition.PredictBallLanding()


if __name__ == "__main__":
    try:
        while True:
            if UnitedPosition.RestrictedArea():
                robot_pos, ball_state, side = _read_state()
                if ball_state == "held":
                    if robot_pos[0] < 56 and (robot_pos[1] > 50 or robot_pos[1] < -50):
                        Macon_kick()
                    elif robot_pos[0] >= 56 and (robot_pos[1] > 50 or robot_pos[1] < -50):
                        back_kick()
                    elif robot_pos[0] > -30 and -50 < robot_pos[1] < 50:
                        OpenGoalShoot()
                elif ball_state == "lost":
                    defence_points = UnitedPosition.FieldChassisPos()
                    if defence_points:
                        point = defence_points[0]
                        UnitedPosition.Pos2Pos([point[0] - 20 * side, point[1] - 20 * side, 0])
                else:
                    lockball.LockBallSlip()
    finally:
        chassis.stop()
        peripheral.Dribble(False)
        UnitedPosition.Stop()


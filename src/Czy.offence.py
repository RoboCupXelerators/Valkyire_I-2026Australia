from LunaPre import *
from BasicFuc import lockball, TurnTo_with_ball
from LunaPre.strategy_tools import angle_error_deg
import math
import time

post_shot_until=0.0

def read_state():
    """Read a fresh robot pose and ball state for the current control cycle."""
    robot_pos = UnitedPosition.FieldRobotPose()
    ball_pos = UnitedPosition.RobotBallPos()
    ball_state = UnitedPosition.BallState(ball_pos)
    side = 1 if robot_pos[1] > 0 else -1
    return robot_pos, ball_state, side

def default_aim(robot_pos):
    return math.degrees(math.atan2(0.0 - robot_pos[1], 121.5 - robot_pos[0])) % 360

def IsOpenGoal(window):
    return (
        window["goal_key"] is not None
        and window["window_open"]
        and len(window["blockers"]) <=1
    )
def OpenGoalShoot():
    global post_shot_until
    robot_pos = UnitedPosition.FieldRobotPose()
    window = UnitedPosition.ShootWindow(
        target="opponent",
        use_estimated_goal=False,
        current_yaw_deg=robot_pos[2],
    )
    if window.get("goal_key") is None:
        est_window = UnitedPosition.ShootWindow(
            target="opponent",
            use_estimated_goal=True,
            current_yaw_deg=robot_pos[2],
        )
        if est_window.get("goal_key") is not None:
            window = est_window

    if not IsOpenGoal(window):
        target_yaw = window.get("target_yaw_deg")
        if target_yaw is None:
            target_yaw = default_aim(robot_pos)
        TurnTo_with_ball(target_yaw)
        return False

    if window["can_shoot_now"] and (0 < robot_pos[2] < 90 or 270 < robot_pos < 360)>0:
        peripheral.ShootBall()
        time.sleep(0.03)
        post_shot_until = time.time() + 0.6
        return True
    
    target_yaw = window.get("target_yaw_deg")
    if target_yaw is None:
        target_yaw = default_aim(robot_pos)
    TurnTo_with_ball(target_yaw)
    return False

def Macon_kick_Plus():#弹墙版
    robot_pos, ball_state, side = read_state()
    if ball_state == "held":
        peripheral.Dribble(0.5)
        TurnTo_with_ball(100 * side)
        threshold = [[40, -80], [80, 80]]
        if UnitedPosition.PointInShape(robot_pos, threshold):
            if abs(angle_error_deg(robot_pos[2],55*side)) < 10:
                peripheral.ShootBall()
                time.sleep(0.03)
                return True
            else:
                TurnTo_with_ball(55 * side)
            return False
        else:
            UnitedPosition.Pos2Pos([56, 75 * side, 90])
            return False
    else:
        return False

def Macon_kick():
    robot_pos, ball_state, side = read_state()
    if ball_state == "held":
        peripheral.Dribble(0.5)
        TurnTo_with_ball(100 * side)
        threshold = [[80, 70*side], [90, 60*side]]
        if UnitedPosition.PointInShape(robot_pos, threshold):
            time.sleep(1)
            TurnTo_with_ball(50)
            peripheral.ShootBall()
            time.sleep(0.03)
            return True
        else:
            TurnTo_with_ball(55 * side)
        return False
    else:
        UnitedPosition.Pos2Pos([90, 65 * side, 90])
        return False

def back_kick():
    robot_pos, ball_state, side = read_state()
    if ball_state == "held":
        peripheral.Dribble(0.7)
        back_point = 80
        if robot_pos[0] > back_point:
            aim = [121.5, -30 * side]
            goal = math.degrees(math.atan2(aim[1] - robot_pos[1],aim[0] - robot_pos[0],)) % 360
            if abs(angle_error_deg(robot_pos[2],goal)) > 8:
                TurnTo_with_ball(goal)
            else:
                peripheral.ShootBall()
                time.sleep(0.03)
                return True
            return False
        else:
            UnitedPosition.Pos2Pos([back_point, robot_pos[1], robot_pos[2]])
    else:
        return False

if __name__ == "__main__":
    while True:
        if UnitedPosition.RestrictedArea():
            robot_pos, ball_state, side = read_state()
            if ball_state == "held":
                if robot_pos[0] < 56 and (robot_pos[1] > 50 or robot_pos[1] < -50):
                    Macon_kick()
                elif robot_pos[0] >= 56 and (robot_pos[1] > 50 or robot_pos[1] < -50):
                    back_kick()
                elif -50 <= robot_pos[1] <= 50:
                    OpenGoalShoot()
            elif ball_state == "lost":
                defence_points = UnitedPosition.FieldChassisPos()
                if defence_points:
                    point = defence_points[0]
                    UnitedPosition.Pos2Pos([point[0] - 20 * side, point[1] - 20 * side, 0])
                else:
                    UnitedPosition.Pos2Pos([-20, 0, 0])
            else:
                lockball.LockBallSlip()
        time.sleep(0.03)
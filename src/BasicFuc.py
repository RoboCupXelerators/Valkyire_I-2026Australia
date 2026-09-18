cr from the
                # previous accepted target.  The ball can move between those
                # observations, so comparing the candidate lists for exact
                # equality would reject a real moving target forever.
                self._lockball_jump_candidate = candidate
                self._lockball_jump_candidate_frames += 1

                if self._lockball_jump_candidate_frames < self.ball_jump_confirm_frames:
                    return self._lockball_last_seen, False

                self._lockball_last_seen = candidate
                self._lockball_last_seen_ts = now
                self._clear_ball_jump_candidate()
                self._reset_lockball_pid()
                return candidate, False

        self._clear_ball_jump_candidate()

        self._lockball_last_seen = candidate
        self._lockball_last_seen_ts = now
        return ball, False

    def LockBallSlip(self):
        """追到 ``CatchVal`` 指定的持球位置。"""
        self._reset_on_pose_change()
        if not self._chassis_can_move():
            # PID output must not accumulate while referee permission blocks
            # wheel commands, or it will surge when permission returns.
            self._reset_lockball_pid()
            chassis.stop()
            return False

        ball, caught = self._current_or_recent_ball()
        if caught:
            peripheral.Dribble(0.3)
            chassis.stop()
            self._reset_lockball_pid()
            return True
        elif ball == BALL_NOT_FOUND:
            peripheral.Dribble(0.2)
            chassis.stop()
            self._reset_lockball_pid()
            return False
        else:
            peripheral.Dribble(0.3)

        ball_x, ball_y = ball[0], ball[1]
        if ball_x == 0 and ball_y == 0:
            ball_angle = 0
        else:
            ball_angle = int(math.degrees(math.atan2(ball_y, ball_x)))

        # Large heading errors and translation create an orbit: the ball moves
        # across the camera while the robot keeps turning, then the controller
        # reverses direction on the next frame. For a ball in front of the
        # robot, align in place first. The rear-ball controller below has its
        # own sign and speed limits, so it must keep priority for ball_x <= 0.
        if ball_x > 0 and abs(ball_angle) >= self.turn_only_angle:
            self._reset_lockball_pid()
            speed_w = max(
                -self.turn_only_max,
                min(self.turn_only_max, float(ball_angle) * self.turn_only_kp),
            )
            peripheral.Dribble(0.3)
            chassis.RobotMoveVector(0.0, 0.0, speed_w)
            return False

        # A rear ball cannot be reached by forward PID. Turn toward it quickly
        # while using a small lateral correction, then hand control back to the
        # normal capture loop once the ball is in front of the robot.
        if ball_x <= 0:
            self._reset_lockball_pid()
            speed_x = max(-self.rear_forward_max, min(0.0, float(ball_x) * self.rear_forward_kp))
            speed_y = max(-self.rear_lateral_max, min(self.rear_lateral_max, float(ball_y) * self.rear_lateral_kp))
            speed_w = max(-self.rear_angular_max, min(self.rear_angular_max, float(ball_angle) * self.rear_angular_kp))
            peripheral.Dribble(0.3)
            chassis.RobotMoveVector(speed_x, speed_y, speed_w)
            return False

        speed_x = -self.pan_x(ball_x)
        speed_y = -self.pan_y(ball_y)
        speed_w = -self.pan_w(ball_angle)

        # If the ball is already closer than CatchVal but still free, keep a
        # small forward creep so a ~1cm shortfall still gets eaten into the
        # dribbler instead of freezing with speed_x=0.
        if ball_x < self.catch_x and abs(ball_y - self.catch_y) < 2.5:
            creep = min(self.near_creep_max, max(0.04, (self.catch_x - float(ball_x)) * 0.05))
            speed_x = max(float(speed_x), creep)

        # Gentle final dock: cap the forward speed on a continuous v = sqrt(2 a d)
        # braking curve so the robot does not punch a low-power-held ball back out
        # on contact while still approaching at full speed from far away. The
        # cruise limit comes from the live pan_x output limit so tuning one knob
        # (ForwardMaxMps) automatically moves where braking starts.
        if self.near_dock_decel > 0.0:
            cruise = abs(float(self.pan_x.output_limits[0]))
            remaining_m = max(0.0, float(ball_x) - self.catch_x) / 100.0
            brake_start_m = (cruise * cruise) / (2.0 * self.near_dock_decel)
            if cruise > 0.0 and remaining_m < brake_start_m:
                speed_x = min(
                    float(speed_x),
                    math.sqrt(2.0 * self.near_dock_decel * remaining_m),
                )

        peripheral.Dribble(0.3)
        chassis.RobotMoveVector(speed_x, speed_y, speed_w)
        return False

    def slip(self):
        return self.LockBallSlip()


lockball = LockSeries()

def Circle(r, k = 1,d = 1):
    chassis.RobotMoveTwist(0.1*r*d*k,-0.9*r*d*k,3/r*d*k)

# TurnTo_with_ball tuning. The orbit radius matches the historical wheel mix;
# the align tolerance is the heading band inside which the skill reports done.
TURN_ORBIT_RADIUS = 0.6
TURN_ALIGN_TOLERANCE_DEG = 15.0


def TurnTo_with_ball(Aim_Angle, SpeedK: float | None = 1):
    """带球绕球转向到目标角度。

    这个函数不是普通原地转向。它故意用四轮差速让机器人围着已控住的球转，
    真实机器上比直接 FieldTurn/RobotTurn 更稳；模拟器物理不准时可能表现很差。

    语义：未持球时只做追球并返回 False；持球后绕球转向，直到航向进入
    ``TURN_ALIGN_TOLERANCE_DEG`` 才返回 True（此时会主动停车）。``SpeedK``
    缩放绕球速度（None 等价 1.0），增益会被限制在 [-1, 1] 之内。

    TODO: 后续把轮速常量、角度误差和速度倍率整理成可读参数，并单独记录
    真机/模拟器差异。不要在未真机验证前改成普通旋转。
    """
    yaw = chassis.GetYaw()
    Aim_Angle = Aim_Angle % 360

    # Not holding the ball yet: keep chasing this tick and report "not done".
    # The old code called LockBallSlip() twice and returned the second result,
    # which could report success without ever aligning.
    if not lockball.LockBallSlip():
        return False
    peripheral.Dribble(0.2)

    # Shortest signed heading error to the aim, normalized to [-180, 180).
    error = angle_error_deg(Aim_Angle, yaw)
    if abs(error) < TURN_ALIGN_TOLERANCE_DEG:
        # Aligned: stop rather than let the previous orbit command keep turning.
        chassis.stop()
        return True

    # Intentional raw wheel mix: orbit around the held ball while turning.
    # LinearMap(error) gives the signed gain in [-1, 1]; the fixed d=-1 keeps
    # the historical sign convention (positive error turns counter-clockwise).
    speed_k = 1.0 if SpeedK is None else float(SpeedK)
    gain = LinearMap(error, [-180, 180], [-1, 1]) * speed_k
    gain = max(-1.0, min(1.0, gain))
    Circle(TURN_ORBIT_RADIUS, gain, -1)
    return False


def Slipsideshot(attack_dir=1): #溜边
    """One tick of sideline carry and shot."""
    robot_ball = UnitedPosition.RobotBallPos()
    close_ball = False
    try:
        close_ball = 0.0 <= float(robot_ball[0]) <= 24.0 and abs(float(robot_ball[1])) <= 16.0
    except Exception:
        close_ball = False
    if not close_ball and robot_ball != BALL_HELD:
        return lockball.LockBallSlip()
    try:
        local_x, local_y, _ = UnitedPosition.FieldRobotPose()
    except Exception:
        return False
    peripheral.Dribble(1.0)
    attack_dir = 1 if attack_dir >= 0 else -1
    side = 1.0 if local_y >= 0.0 else -1.0
    half_long = float(getattr(cfg.Bounds, "Long", 260)) * 0.5
    half_short = float(getattr(cfg.Bounds, "Short", 200)) * 0.5
        edge_y = side * min(68.0, half_short - 24.0)
    corner_progress = half_long - 30.0
    progress = attack_dir * local_x
    edge_error = edge_y - local_y
    forward_yaw = 0.0 if attack_dir > 0 else 180.0
    hide_yaw = (forward_yaw + side * attack_dir * 20.0) % 360.0
    near_corner = progress >= corner_progress - 16.0
    edge_tolerance = 24.0 if near_corner else 9.0
    if abs(edge_error) > edge_tolerance and progress < corner_progress - 14.0:
        carry_yaw = hide_yaw
        yaw_error = angle_error_deg(compass(), carry_yaw)
        speed_x = attack_dir * (0.16 if progress < corner_progress - 12.0 else 0.07)
        speed_y = max(-0.13, min(0.13, edge_error * 0.0027))
        if abs(yaw_error) > 25.0:
            speed_x *= 0.55
            speed_y *= 0.35
        chassis.FieldMoveVector(speed_x, speed_y, carry_yaw, KpW=max(0.18, min(0.55, abs(yaw_error) / 80.0)))
        return False
    if progress < corner_progress:
        carry_yaw = hide_yaw
        yaw_error = angle_error_deg(compass(), carry_yaw)
        speed_x = attack_dir * (0.25 if progress < corner_progress - 28.0 else 0.16)
        speed_y = max(-0.05, min(0.05, edge_error * 0.0016))
        if abs(yaw_error) > 25.0:
            speed_x *= 0.55
            speed_y *= 0.35
        chassis.FieldMoveVector(speed_x, speed_y, carry_yaw, KpW=max(0.18, min(0.50, abs(yaw_error) / 85.0)))
        return False
    aim = UnitedPosition.Pos2Angle([local_x, local_y], [attack_dir * half_long, -side * 18.0]) % 360
    yaw_error = angle_error_deg(compass(), aim)
    if abs(yaw_error) <= 18.0 and abs(edge_error) <= 34.0:
        peripheral.ShootBall()
        peripheral.Dribble(False)
        logger.success("Slipsideshot shot.")
        return True
    speed_x = -attack_dir * 0.03 if progress > corner_progress + 8.0 else 0.0
    speed_y = max(-0.02, min(0.02, edge_error * 0.0005))
    chassis.FieldMoveVector(speed_x, speed_y, aim, KpW=max(0.22, min(0.60, abs(yaw_error) / 80.0)))
    return False

"""Strategy v5 - comprehensive rewrite of offence approach logic.

Root cause analysis from v4 test:
- R1: flank_ball=924, approach=856, retreat=107 → flank/approach/retreat cycle
- R3: retreat_home=1366 → gives up too easily
- Ball spends 83% of time in R2's half (x<0), so R2 challenging is CORRECT

The real problem: offence can't win the ball from defence in a head-on contest.
Flanking doesn't help because after flanking, LockBallSlip still fails against
a defender who is also running LockBallSlip on the same ball.

Solution: Instead of flanking (which doesn't work against LockBallSlip),
use a "wait and pounce" strategy:
1. When stuck in approach, DON'T flank. Instead, back off to a position
   between the ball and our goal, creating space.
2. Wait for the ball to come to us (defender may clear it our way).
3. If ball gets close enough while we're positioned, immediately LockBallSlip.

This is more realistic football: if you can't win the ball, drop back and
wait for a loose ball or a clear that comes your way.
"""
import re

off_path = r'D:\Github\Software\src\offence.py'
with open(off_path, 'r', encoding='utf-8') as f:
    off = f.read()

# 1. Replace config: remove flank params, add drop_back params
old_config = """    approach_stuck_threshold_s: float = float(os.environ.get("OFFENCE_APPROACH_STUCK_S", "4.0"))
    flank_speed_gain: float = float(os.environ.get("OFFENCE_FLANK_GAIN", "3.0"))
    flank_offset_cm: float = float(os.environ.get("OFFENCE_FLANK_OFFSET", "25"))
    flank_max_attempts: int = int(os.environ.get("OFFENCE_FLANK_MAX_ATTEMPTS", "2"))"""

new_config = """    approach_stuck_threshold_s: float = float(os.environ.get("OFFENCE_APPROACH_STUCK_S", "3.0"))
    drop_back_distance_cm: float = float(os.environ.get("OFFENCE_DROP_BACK_DIST", "40"))
    drop_back_speed_gain: float = float(os.environ.get("OFFENCE_DROP_BACK_GAIN", "1.6"))
    pounce_distance_cm: float = float(os.environ.get("OFFENCE_POUNCE_DIST", "35"))"""

off = off.replace(old_config, new_config)
print("offence.py: replaced flank config with drop_back config")

# 2. Replace init: remove flank state, add drop_back state
old_init = """        self._approach_enter_ts = None
        self._flank_side = 1  # +1 = left flank, -1 = right flank
        self._flank_attempts = 0
        self._enemy_cache = None"""

new_init = """        self._approach_enter_ts = None
        self._drop_back_active = False
        self._enemy_cache = None"""

off = off.replace(old_init, new_init)
print("offence.py: replaced flank init with drop_back init")

# 3. Replace step() ball handling: remove retreat_home, add drop_back
old_step = """        if ball == BALL_HELD:
            self._approach_enter_ts = None  # reset stuck timer when we have the ball
            self._flank_attempts = 0
            self._play_with_ball(pose)
            return

        # If retreating, move to home first before re-approaching
        if self.state == "retreat_home":
            self._set_dribble(False)
            arrived = UnitedPosition.Pos2Pos(self.home, self.cfg.search_speed_gain)
            if arrived:
                self._approach_enter_ts = None
                self._flank_attempts = 0
                self._set_state("approach_ball")
            return

        self._approach_ball(ball)"""

new_step = """        if ball == BALL_HELD:
            self._approach_enter_ts = None
            self._drop_back_active = False
            self._play_with_ball(pose)
            return

        # If in drop_back mode, position between ball and our goal
        if self._drop_back_active:
            self._drop_back_and_wait(ball, pose)
            return

        self._approach_ball(ball)"""

off = off.replace(old_step, new_step)
print("offence.py: replaced step() with drop_back logic")

# 4. Replace _approach_ball: trigger drop_back instead of flank
old_approach = """    def _approach_ball(self, ball: list | tuple) -> None:
        now = time.time()
        if self._approach_enter_ts is None:
            self._approach_enter_ts = now

        stuck_duration = now - self._approach_enter_ts

        # If stuck too long in approach, switch to flanking maneuver
        if stuck_duration > self.cfg.approach_stuck_threshold_s:
            self._flank_ball(ball)
            return

        self._set_state("approach_ball")
        lockball.LockBallSlip()"""

new_approach = """    def _approach_ball(self, ball: list | tuple) -> None:
        now = time.time()
        if self._approach_enter_ts is None:
            self._approach_enter_ts = now

        stuck_duration = now - self._approach_enter_ts

        # If stuck too long in approach, drop back and wait for opportunity
        if stuck_duration > self.cfg.approach_stuck_threshold_s:
            self._drop_back_active = True
            self._approach_enter_ts = None
            return

        self._set_state("approach_ball")
        lockball.LockBallSlip()"""

off = off.replace(old_approach, new_approach)
print("offence.py: replaced _approach_ball with drop_back trigger")

# 5. Replace _flank_ball with _drop_back_and_wait
old_flank = """    def _flank_ball(self, ball: list | tuple) -> None:
        \"\"\"Side-step around the ball to avoid a direct confrontation with defenders.\"\"\"
        self._set_state("flank_ball")
        self._set_dribble(False)

        ball_field = UnitedPosition.FieldBallPos()
        pose = UnitedPosition.FieldRobotPos()
        if not _valid_xy(ball_field) or not _valid_pose(pose):
            lockball.LockBallSlip()
            return

        bx, by = float(ball_field[0]), float(ball_field[1])

        # Choose flank side away from nearest enemy
        enemies = self._field_enemies()
        if enemies:
            left_score = sum(1 for e in enemies if len(e) >= 2 and float(e[1]) < by)
            right_score = sum(1 for e in enemies if len(e) >= 2 and float(e[1]) >= by)
            self._flank_side = -1 if left_score <= right_score else 1

        offset = self.cfg.flank_offset_cm * self._flank_side
        flank_y = _clamp(by + offset, -self.field_short / 2.0 + 20.0, self.field_short / 2.0 - 20.0)
        approach_x = bx - self.attack_dir * 15.0
        flank_yaw = UnitedPosition.Pos2Angle([approach_x, flank_y], [bx, by]) % 360

        target = [approach_x, flank_y, flank_yaw]
        safe_target = self._project_strategy_target(target)

        arrived = UnitedPosition.Pos2Pos(safe_target, self.cfg.flank_speed_gain)
        if arrived:
            self._flank_attempts += 1
            if self._flank_attempts >= self.cfg.flank_max_attempts:
                # Too many attempts, retreat to home to regroup
                self._set_state("retreat_home")
                self._approach_enter_ts = None
                self._flank_attempts = 0
                self._set_dribble(False)
            else:
                # Try to grab ball from the flank position
                self._approach_enter_ts = None
                lockball.LockBallSlip()"""

new_flank = """    def _drop_back_and_wait(self, ball: list | tuple, pose: list | tuple | None) -> None:
        \"\"\"Drop back to a position between ball and our goal, wait for a loose ball.\"\"\"
        ball_field = UnitedPosition.FieldBallPos()
        if not _valid_xy(ball_field) or not _valid_pose(pose):
            # Can't position, just keep trying
            self._drop_back_active = False
            lockball.LockBallSlip()
            return

        bx, by = float(ball_field[0]), float(ball_field[1])
        rx, ry = float(pose[0]), float(pose[1])

        # Ball distance in robot frame
        ball_dist = math.hypot(float(ball[0]), float(ball[1]))

        # If ball is close enough, pounce! Switch back to approach
        if ball_dist <= self.cfg.pounce_distance_cm:
            self._drop_back_active = False
            self._approach_enter_ts = None
            self._set_state("approach_ball")
            lockball.LockBallSlip()
            return

        # Position between ball and our goal, offset back from ball
        back_x = bx - self.attack_dir * self.cfg.drop_back_distance_cm
        back_y = by * 0.5  # Shift toward center to cover more angles
        back_x = _clamp(back_x, -self.field_long / 2.0 + 20.0, self.field_long / 2.0 - 20.0)
        back_y = _clamp(back_y, -self.field_short / 2.0 + 20.0, self.field_short / 2.0 - 20.0)

        # Face toward the ball
        face_yaw = UnitedPosition.Pos2Angle([back_x, back_y], [bx, by]) % 360

        target = [back_x, back_y, face_yaw]
        safe_target = self._project_strategy_target(target)

        self._set_state("drop_back")
        self._set_dribble(False)
        arrived = UnitedPosition.Pos2Pos(safe_target, self.cfg.drop_back_speed_gain)

        # Even while positioning, if ball gets close, pounce
        if ball_dist <= self.cfg.pounce_distance_cm:
            self._drop_back_active = False
            self._approach_enter_ts = None
            self._set_state("approach_ball")
            lockball.LockBallSlip()"""

off = off.replace(old_flank, new_flank)
print("offence.py: replaced _flank_ball with _drop_back_and_wait")

with open(off_path, 'w', encoding='utf-8') as f:
    f.write(off)

print("\nDone! v5 strategy patch applied.")

import sys
from pathlib import Path
# Add the source directory before importing strategy modules so direct
# execution from the repository root uses the same runtime as the launcher.
SRC_DIR = Path(__file__).resolve().parent
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from BasicFuc import *
from LunaPre import Communication, Settings, UnitedPosition, chassis, logger, peripheral, runtime, cfg
from LunaPre.strategy_tools import BALL_NOT_FOUND, safe_stop, BALL_HELD, angle_error_deg, clamp
import numpy as np
import time
import math


# 模拟器的正式场地坐标（单位：厘米）。不要再使用 900x600 的旧假设：
# 当前 Unity 场地是约 260x200cm，越过禁区/边线会触发裁判处罚。
FIELD_LONG = float(getattr(cfg.Bounds, "Long", 260.0))
FIELD_SHORT = float(getattr(cfg.Bounds, "Short", 200.0))
restricted = getattr(cfg.Motion, "RestrictedArea", None)
penalty_x_limit = float(getattr(restricted, "XLimit", FIELD_LONG / 2 - 30.0))
clearance = float(getattr(restricted, "Clearance", 5.0))
# 这是合法射门接近点，不是把机器人开进球门/禁区的门线目标。
# 再留出 25cm 制动余量，避免动态底盘在停止前越过禁区线。
GOAL_LINE_X = max(20.0, penalty_x_limit - clearance - 25.0)

# 状态定义
STATE_FIND_BALL = 0  # 找球阶段
STATE_GO_TO_GOAL = 1  # 前往门线阶段
STATE_FINISHED = 2    # 完成任务停止

current_state = STATE_FIND_BALL
ball_found_time = None  # 记录找到球的时间

# ==================== 后场弹墙射门参数（260x200cm） ====================
# 坐标约定与 FieldRobotPose 一致: [x_forward, y_left, yaw]
# 半场: X=±130, Y=±100。进攻方向 ATTACK_DIR=+1 时对方球门约在 x=+130。
ATTACK_DIR = 1
SHOT_GOAL_HEADING_LIMIT_DEG = 89.0
BOUNCE_WALL_MARGIN_CM = 8.0   # 侧墙内侧留白，避免目标贴死边界
BOUNCE_SETUP_X_CM = 15.0      # 后场起飞点（沿进攻方向的 x）
BOUNCE_SETUP_Y_CM = 72.0      # 贴边带球目标 |y|
BOUNCE_MIN_SIDE_CM = 55.0     # |y| 太小，弹墙角太钝，先去边路
BOUNCE_MAX_PROGRESS_CM = 75.0 # 太靠前不再走后场弹墙，留给大脚/上平面
BOUNCE_AIM_TOL_DEG = 12.0     # 对准镜像点后允许的航向误差
BOUNCE_CARRY_SPEED = 0.22     # 带球去起飞点的场地方向速度
BOUNCE_DRIBBLE = 0.8

# ==================== 队内通信适配层 ====================
# 复用 LunaPre.communication.Communication 的 10 字节帧：
#   [帧头0x23, 识别ID, 角色, 状态, x+, y+, x-, y-, reserve, 帧尾0x42]
# 位置按“正负拆分”传厘米值（每路 0~255，覆盖 ±255cm，够当前 260x200 场地）。
# reserve 是唯一空闲字节，用来压缩朝向与吸球器状态：
#   bit0~6 = yaw 档位（0~127，约 2.81 度/档）   bit7 = dribbler ready
# 策略侧只广播自己的状态，由接收方自行差分推导队友速度，不读 Unity 真值。
#
# 队内通信参数原先挂在 AustraliaStrategy 配置段，现已改为本文件内的常量，
# 不再读写运行时 config.json。_cfg_lookup 仍用于可选字段（如 RoboInfo.Color）。
def _cfg_lookup(*keys, default=None):
    """按属性路径安全取配置；任一层缺失返回 default。"""
    node = cfg
    for key in keys:
        try:
            node = getattr(node, key)
        except Exception:
            return default
    return default if node is None else node


TEAM_STATE_SEND_PERIOD_S = 0.05   # 约 20Hz 广播
TEAM_STATE_MAX_AGE_S = 0.30       # 300ms 没有新帧即判定队友数据过期
TEAM_RECEIVE_DRAIN_MAX = 16   # 每周期最多清空 16 帧；最新帧用于状态，跳变帧单独留事件
TEAM_YAW_STEPS = 128
TEAM_YAW_MASK = 0x7F
TEAM_DRIBBLE_BIT = 0x80
TEAM_COOP_ENABLED = True      # 队友数据新鲜时启用抢球分工；置 False 即退化为单机
TEAM_LINK_RETRY_S = 2.0       # 链路建立失败后的重试间隔（原来失败即永久放弃）
# 队友持球时本机的支援位（原先是散在函数里的魔数）
TEAM_SUPPORT_BACK_CM = 35.0        # 沿进攻方向后退的距离
TEAM_SUPPORT_LATERAL_LIMIT_CM = 60.0
TEAM_SUPPORT_LATERAL_GAIN = 0.5
TEAM_SUPPORT_SPEED_GAIN = 0.8
# 角色码覆盖。配置里没有队色字段（RoboInfo.Color 既不在 DEFAULT_CONFIG 也不在
# 控制面板模板里），所以默认只能报黄队。要报蓝队就改成
# Communication.ROLE_OFFENCE_BLUE，或给 config.json 的 RoboInfo 加 "Color": "B"。
TEAM_FORCE_ROLE = None

_TEAM_LINK = None             # Communication 实例；None 表示未接入
_TEAM_LINK_TRIED = False
_TEAM_LINK_NEXT_TRY = 0.0     # 下次允许重试建立链路的时刻
_TEAM_LINK_WARNED = False     # 链路失败只告警一次，避免刷屏
_TEAM_SAMPLE = (None, -math.inf)     # (解析后的队友帧, 收到时刻)
_TEAM_EVENT = None            # (事件名, 帧, 时刻)：队友状态跳变，取一次即失效
_TEAM_LAST_SEND_TIME = -math.inf
_TEAM_LAST_SENT_STATUS = None
_TEAM_ROBOT_ID = None
_TEAM_PEER_ID = None


def _team_robot_id():
    """本机 ID，取自 RoboInfo.number。"""
    global _TEAM_ROBOT_ID
    if _TEAM_ROBOT_ID is None:
        try:
            _TEAM_ROBOT_ID = int(getattr(cfg.RoboInfo, "number", 1))
        except Exception:
            _TEAM_ROBOT_ID = 1
    return _TEAM_ROBOT_ID


def _team_peer_id():
    """队友 ID：配置的 RobotIds 只在本机号包含于其中时才生效，否则成对推断。"""
    global _TEAM_PEER_ID
    if _TEAM_PEER_ID is not None:
        return _TEAM_PEER_ID
    robot_id = _team_robot_id()

    ids = [1, 2]

    # 正式策略把 RobotIds 硬编码成 [1, 2]。如果本机是 3 号，这份配置根本
    # 不适用于我们——不能顺手挑一个"不等于自己的"就当成队友（那会把队友
    # 错配成 1 号）。只有本机确实在名单里，才按名单配对。
    if robot_id in ids:
        peer = next((value for value in ids if value != robot_id), None)
        if peer is not None:
            _TEAM_PEER_ID = peer
            return _TEAM_PEER_ID

    # 否则按 3<->4 / 1<->2 成对推断，和流程图里的编队一致。
    _TEAM_PEER_ID = {3: 4, 4: 3, 1: 2, 2: 1}.get(robot_id, 1)
    return _TEAM_PEER_ID


def _team_status_for(ball_state):
    """球状态 -> 协议状态码（已持球 / 正在抢球 / 没球）。"""
    if ball_state == "held":
        return Communication.STATUS_CATCH_BALL
    if ball_state == "free":
        return Communication.STATUS_CHASING
    return Communication.STATUS_NO_BALL


def _team_role():
    """角色码：配置里没有队色字段，可用 TEAM_FORCE_ROLE 强制指定。"""
    if TEAM_FORCE_ROLE is not None:
        return int(TEAM_FORCE_ROLE)
    color = str(_cfg_lookup("RoboInfo", "Color", default="Y")).upper()
    return (
        Communication.ROLE_OFFENCE_BLUE
        if color.startswith("B")
        else Communication.ROLE_OFFENCE_YELLOW
    )


def _team_encode_reserve(yaw_deg, dribble_ready=False):
    """把朝向量化进 reserve 低 7 位，bit7 放 dribbler 状态。"""
    try:
        yaw = float(yaw_deg) % 360.0
    except Exception:
        yaw = 0.0
    code = int(round(yaw * TEAM_YAW_STEPS / 360.0)) % TEAM_YAW_STEPS
    reserve = code & TEAM_YAW_MASK
    if dribble_ready:
        reserve |= TEAM_DRIBBLE_BIT
    return reserve


def _team_decode_reserve(reserve):
    """reserve -> (yaw_deg, dribble_ready)。"""
    try:
        reserve = int(reserve) & 0xFF
    except Exception:
        return 0.0, False
    yaw = (reserve & TEAM_YAW_MASK) * 360.0 / TEAM_YAW_STEPS
    return yaw, bool(reserve & TEAM_DRIBBLE_BIT)


def _team_local_pose():
    """本机场地位姿；无效时返回 None。"""
    try:
        pose = UnitedPosition.FieldRobotPose()
        if _valid_robot_pose(pose):
            return [float(pose[0]), float(pose[1]), float(pose[2])]
    except Exception:
        pass
    return None


def _team_dribble_ready():
    return bool(getattr(peripheral, "DribbleStatus", False))


def _team_ensure_link():
    """惰性建立队内链路；失败则静默退化为单机，不影响原有射门/追球。

    失败后**不会**永久放弃：每隔 ``TEAM_LINK_RETRY_S`` 再试一次。
    （原来用 ``_TEAM_LINK_TRIED`` 做单向门，第一次调用时若 Bits 尚未就绪，
    队内通信就再也不会恢复。）
    """
    global _TEAM_LINK, _TEAM_LINK_TRIED, _TEAM_LINK_NEXT_TRY, _TEAM_LINK_WARNED
    if _TEAM_LINK is not None:
        return _TEAM_LINK
    now = time.monotonic()
    if _TEAM_LINK_TRIED and now < _TEAM_LINK_NEXT_TRY:
        return None
    _TEAM_LINK_TRIED = True
    _TEAM_LINK_NEXT_TRY = now + TEAM_LINK_RETRY_S

    def _give_up(reason):
        global _TEAM_LINK_WARNED
        if not _TEAM_LINK_WARNED:
            logger.warning("Team comm unavailable (will retry): %s; solo mode" % reason)
            _TEAM_LINK_WARNED = True
        return None

    try:
        bits = runtime.Bits
    except Exception:
        bits = None
    if bits is None:
        return _give_up("runtime Bits unavailable")
    peer_id = _team_peer_id()
    if peer_id is None or int(peer_id) == int(_team_robot_id()):
        return _give_up("no distinct peer id")
    try:
        _TEAM_LINK = Communication(bits=bits, peer_id=int(peer_id))
    except Exception as error:
        _TEAM_LINK = None
        return _give_up(error)
    _TEAM_LINK_WARNED = False
    logger.info(
        "Team comm online: robot_id=%s peer_id=%s period=%.3fs"
        % (_team_robot_id(), _team_peer_id(), TEAM_STATE_SEND_PERIOD_S)
    )
    return _TEAM_LINK


def _team_send(ball_state, now):
    """按周期或状态跳变时广播本机状态（含位置与朝向）。"""
    global _TEAM_LAST_SEND_TIME, _TEAM_LAST_SENT_STATUS
    link = _team_ensure_link()
    if link is None:
        return False

    status = _team_status_for(ball_state)
    changed = status != _TEAM_LAST_SENT_STATUS
    due = (now - _TEAM_LAST_SEND_TIME) >= TEAM_STATE_SEND_PERIOD_S
    if not changed and not due:
        return False

    pose = _team_local_pose()
    if pose is None:
        return False
    position = [int(round(pose[0])), int(round(pose[1]))]
    reserve = _team_encode_reserve(pose[2], _team_dribble_ready())
    link.SendStatus(
        status=status,
        role=_team_role(),
        position=position,
        reserve=reserve,
    )
    _TEAM_LAST_SEND_TIME = now
    _TEAM_LAST_SENT_STATUS = status
    if changed:
        logger.info(
            "Team comm -> peer=%s status=0x%02X pos=(%d,%d) reserve=%d"
            % (_team_peer_id(), status, position[0], position[1], reserve)
        )
    return True


def _team_receive(now):
    """非阻塞清空接收队列。

    - **最新帧**用于位置/朝向/持球状态（``_TEAM_SAMPLE``）。
    - 队列里出现过的**状态跳变帧**会额外留成一次事件（``_TEAM_EVENT``）。

    为什么要额外留事件：协议只有 CATCH_BALL / CHASING / NO_BALL 三个状态码，
    没有独立的"球权丢失"事件，只能靠观察到跳变来表达。原来只保留最后一帧，
    如果对端队列里堆了 2 帧以上，"已持球 -> 没球"这个中间帧会被直接丢掉，
    本机就感知不到队友丢球了。
    """
    global _TEAM_SAMPLE, _TEAM_EVENT
    link = _team_ensure_link()
    if link is None:
        return None
    latest = None
    transition = None
    # 基线：优先用上一次采样的状态；首次收到数据时用本批第一帧建立基线，
    # 这样"本批内部"的跳变（例如同批先 0x01 后 0xFF）也能被识别出来。
    previous, _ = _TEAM_SAMPLE
    previous_status = previous.get("status") if isinstance(previous, dict) else None
    baseline = previous_status
    for _ in range(TEAM_RECEIVE_DRAIN_MAX):
        frame = link.Receive()
        if not frame:
            break
        status = int(frame.get("status", -1))
        if baseline is None:
            baseline = status
        elif status != baseline and transition is None:
            transition = frame      # 记下第一帧跳变，别被后面的帧覆盖
        latest = frame
    if latest is None:
        return None
    if int(latest.get("id", -1)) != int(_team_peer_id()):
        return None

    previous, previous_time = _TEAM_SAMPLE
    if isinstance(previous, dict):
        dt = now - previous_time
        if 0.0 < dt <= TEAM_STATE_MAX_AGE_S:
            x0, y0 = previous.get("signed_xy", [0, 0])[:2]
            x1, y1 = latest.get("signed_xy", [0, 0])[:2]
            yaw0 = previous.get("yaw_deg", 0.0)
            yaw1, _ = _team_decode_reserve(latest.get("reserve", 0))
            latest["vx"] = (float(x1) - float(x0)) / dt
            latest["vy"] = (float(y1) - float(y0)) / dt
            latest["wz"] = angle_error_deg(float(yaw0), float(yaw1)) / dt

    yaw_deg, dribble = _team_decode_reserve(latest.get("reserve", 0))
    latest["yaw_deg"] = yaw_deg
    latest["dribble_ready"] = dribble
    latest["has_ball"] = int(latest.get("status", 0xFF)) == Communication.STATUS_CATCH_BALL
    latest["chasing"] = int(latest.get("status", 0xFF)) == Communication.STATUS_CHASING
    _TEAM_SAMPLE = (latest, now)

    if transition is not None:
        status = int(transition.get("status", -1))
        if status == Communication.STATUS_CATCH_BALL:
            name = "teammate_gained_ball"
        elif status == Communication.STATUS_NO_BALL:
            name = "teammate_lost_ball"
        else:
            name = "teammate_state_changed"
        _TEAM_EVENT = (name, transition, now)
        logger.info("Team comm event: %s status=0x%02X" % (name, status & 0xFF))
    return latest


def _team_take_event(max_age_s=None):
    """取出并清除最近的队友状态跳变事件；没有或已过期则返回 None。

    消费一次即失效，避免同一事件被反复当成"刚刚发生"。
    """
    global _TEAM_EVENT
    event, _TEAM_EVENT = _TEAM_EVENT, None
    if event is None:
        return None
    if max_age_s is not None and (time.monotonic() - event[2]) > float(max_age_s):
        return None     # 已经过期，直接丢弃
    return event


def _team_tick(ball_state):
    """每个控制周期调用一次：先收后发，收发都不阻塞决策。"""
    now = time.monotonic()
    teammate = _team_receive(now)
    _team_send(ball_state, now)
    return teammate


def _fresh_teammate():
    """返回尚未过期的队友状态字典，过期或未接入时返回 None。"""
    frame, timestamp = _TEAM_SAMPLE
    if not isinstance(frame, dict):
        return None
    age = time.monotonic() - timestamp
    if age < 0.0 or age > TEAM_STATE_MAX_AGE_S:
        return None
    return frame


def _teammate_holds_ball():
    if not TEAM_COOP_ENABLED:
        return False
    frame = _fresh_teammate()
    return bool(frame and frame.get("has_ball"))


def _team_distance_to_ball(field_pos):
    pose = _team_local_pose()
    if pose is None or field_pos is None:
        return None
    try:
        return math.hypot(float(field_pos[0]) - pose[0], float(field_pos[1]) - pose[1])
    except Exception:
        return None


def _team_support_step():
    """队友已持球时本机让位：退到中场支援位，等待传球而不是双机抢球。"""
    pose = _team_local_pose()
    if pose is None:
        chassis.stop()
        return "support_no_pose"
    support_x = -ATTACK_DIR * TEAM_SUPPORT_BACK_CM
    support_y = max(
        -TEAM_SUPPORT_LATERAL_LIMIT_CM,
        min(TEAM_SUPPORT_LATERAL_LIMIT_CM, pose[1] * TEAM_SUPPORT_LATERAL_GAIN),
    )
    peripheral.Dribble(False)
    _safe_navigate_to(
        [support_x, support_y, _opponent_goal_yaw()], TEAM_SUPPORT_SPEED_GAIN
    )
    return "support"


def read_ball_snapshot():
    global last_ball_field, last_ball_seen_time, search_target_yaw, search_sweep_side

    ball_pos = UnitedPosition.RobotBallPos()
    ball_state = UnitedPosition.BallStatus(ball_pos)

    field_pos = None
    if ball_state == "free":
        try:
            candidate = UnitedPosition.FieldBallPos()
        except Exception:
            candidate = None
        if _valid_field_ball(candidate):
            field_pos = [float(candidate[0]), float(candidate[1])]
            last_ball_field = list(field_pos)
            last_ball_seen_time = time.monotonic()
            search_target_yaw = None
            search_sweep_side = 1.0

    return ball_pos, ball_state, field_pos

def _field_halves():#场地尺寸换算成半场长度
    #返回 (half_long, half_short)。260x200 -> (130, 100)。
    return float(FIELD_LONG) * 0.5, float(FIELD_SHORT) * 0.5


SAFETY_RECOVERY_SPEED_GAIN = 0.8


def _valid_robot_pose(pose):
    """Return whether a field pose is usable for safety decisions."""
    try:
        return (
            pose is not None
            and len(pose) >= 3
            and all(math.isfinite(float(value)) and abs(float(value)) < 1000.0 for value in pose[:3])
        )
    except Exception:
        return False


def _safe_navigate_to(target, speed_gain=None):
    """Use the path-aware navigation API for all field-coordinate targets."""
    try:
        return UnitedPosition.Navigate2(target, speed_gain)
    except AttributeError:
        # Keep compatibility with older position facades that only expose Pos2Pos.
        return UnitedPosition.Pos2Pos(target, speed_gain)


def _motion_safety_guard():
    """Block unsafe motion and command a recovery target when needed.

    ProjectSafePose applies the configured restricted-area limits and boundary
    clearance. This guard runs before every strategy tick.
    """
    pose = UnitedPosition.FieldRobotPose()
    if not _valid_robot_pose(pose):
        chassis.stop()
        return False

    safe_target = UnitedPosition.ProjectSafePose(pose)
    if safe_target is None:
        return True

    peripheral.Dribble(False)
    logger.warning(
        "Motion safety recovery: pose=%s target=%s"
        % ([round(float(value), 1) for value in pose[:3]],
           [round(float(value), 1) for value in safe_target[:3]])
    )
    _safe_navigate_to(safe_target, SAFETY_RECOVERY_SPEED_GAIN)
    return False


def _safe_field_move_vector(speed_x, speed_y, facing_angle, Kp=None, KpW=None):
    """Send field motion only while the current pose is inside safe limits."""
    if not _motion_safety_guard():
        return False
    return chassis.FieldMoveVector(speed_x, speed_y, facing_angle, Kp=Kp, KpW=KpW)


def _opponent_goal_yaw():
    return 0.0 if ATTACK_DIR >= 0 else 180.0


def _is_facing_opponent_goal():
    """Allow a shot only when the field-frame heading is not toward our goal."""
    try:
        pose = UnitedPosition.FieldRobotPose()
        if not _valid_robot_pose(pose):
            return False
        error = abs(angle_error_deg(float(pose[2]), _opponent_goal_yaw()))
        return error <= SHOT_GOAL_HEADING_LIMIT_DEG
    except Exception:
        return False


def _pose_is_safe():
    """位姿是否已在不需纠正的安全区。

    等价于 ``UnitedPosition.ProjectSafePose(pose) is None``，但**不带副作用**。
    原来这里直接调 ``UnitedPosition.RestrictedArea()`` 当谓词用，有两个坑：
      1) 该函数返回 True 表示"已经安全"，和它的名字/注释的直觉相反；
      2) 它内部在判定为不安全时会真的下发 ``Pos2Pos`` 移动指令——
         把一个会动机器人的操作写在 ``if`` 条件里很危险。
    """
    try:
        pose = UnitedPosition.FieldRobotPose()
        if not _valid_robot_pose(pose):
            return False
        return UnitedPosition.ProjectSafePose(pose) is None
    except Exception:
        return False


def _shoot_if_safe():
    if not _is_facing_opponent_goal():
        logger.warning(
            "Shot blocked: heading is more than %.1f degrees from opponent goal"
            % SHOT_GOAL_HEADING_LIMIT_DEG
        )
        return False
    peripheral.ShootBall()
    return True


def BounceWallGeometry(robot_pose, attack_dir=None):
    """用台球镜像法计算弹墙瞄准角。

    步骤:
    1. 按机器人当前 y 选最近侧墙（上边 y>0 / 下边 y<0）
    2. 把对方球门中心关于该侧墙做镜像
    3. 从机器人指向镜像点的方向，就是打墙后进门的出射方向
    4. 机器人到镜像点的连线与墙的交点，就是理论反弹点

    返回 dict，方便单步打印调试。
    """
    if attack_dir is None:#规范化attack_dir,确保只能是±1
        attack_dir = ATTACK_DIR
        if attack_dir >= 0 :
            attack_dir = 1
        else:
            attack_dir = -1

    x = float(robot_pose[0])
    y = float(robot_pose[1])
    half_long, half_short = _field_halves()

    if y >= 0.0:
        side = 1.0
    else:
        side = -1.0
    wall_y = side * (half_short - BOUNCE_WALL_MARGIN_CM)  # 墙线约 ±92
    goal = [attack_dir * (half_long - 5.0), 0.0]          # 球门中心略内收
    mirrored = [goal[0], 2.0 * wall_y - goal[1]]          # 镜像球门计算

    aim_yaw = float(UnitedPosition.Pos2Angle([x, y], mirrored)) % 360.0

    dx = mirrored[0] - x##########
    dy = mirrored[1] - y
    if abs(dy) < 1e-6:
        bounce = [x, wall_y]
    else:
        t = (wall_y - y) / dy
        bounce = [x + t * dx, wall_y]

    return {
        "side": side,#打哪一侧
        "wall_y": wall_y,#在哪面墙附近
        "goal": goal,#球门中心坐标
        "mirrored": mirrored,#镜像球门坐标
        "bounce": bounce,#理论反弹点坐标
        "aim_yaw": aim_yaw,#该转到的航向
        "progress": attack_dir * x,#沿进攻方向的前进距离，越大越靠前
    }


def BackWallBounceStep(attack_dir=None):
    """后场弹墙射门：单帧 tick，循环调用。

    状态机（返回值，方便下一步逐项测，返回的是字符串）:
      - locking      : 还没持球，继续 LockBallSlip
      - setup_side   : |y| 不够，先带球贴边
      - setup_depth  : 太靠前，先撤到后场起飞带
      - aiming       : 已在合适区，绕球转向镜像瞄准角
      - shoot        : 航向到位，出脚
      - lost_pose    : 定位失败

    注意: 弹墙不走 CanShoot/ShootWindow（那是直射窗口）。
    """
    if attack_dir is None:
        attack_dir = ATTACK_DIR
    if attack_dir >= 0:
        attack_dir = 1
    else:
        attack_dir = -1

    # Step A: 判断有没有持球
    # 没球就先锁球；有球才计算弹墙几何。
    ball = UnitedPosition.RobotBallPos()
    ball_status = UnitedPosition.BallStatus(ball)
    if ball_status == "lost":
        peripheral.Dribble(False)
        return "locking"
    if ball_status != "held" and not lockball.LockBallSlip():
        peripheral.Dribble(True)
        return "locking"

    # Step B: 读位姿，算弹墙几何
    try:
        pose = UnitedPosition.FieldRobotPose()
        x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])
    except Exception:
        chassis.stop()
        return "lost_pose"

    geo = BounceWallGeometry([x, y, yaw], attack_dir=attack_dir)
    side = geo["side"]
    aim_yaw = geo["aim_yaw"]
    progress = geo["progress"]
    peripheral.Dribble(BOUNCE_DRIBBLE)
#BounceWallGeometry()这个函数返回的值是一串字典，里面包含了弹墙射门所需的各种几何信息。
#包括打哪一侧、在哪面墙附近、球门中心坐标、镜像球门坐标、理论反弹点坐标、该转到的航向以及沿进攻方向的前进距离等。
#这里是对字典里的数值的读取
    logger.info(
        "BounceWall geo side=%s wall_y=%.1f bounce=%s aim=%.1f progress=%.1f"
        % (
            side,
            geo["wall_y"],
            [round(geo["bounce"][0], 1), round(geo["bounce"][1], 1)],
            aim_yaw,
            progress,
        )
    )

    #Step C: 去边路起飞点
    #弹墙需要足够横向空间；中路直打墙几乎垂直反弹，难进门。
    if abs(y) < BOUNCE_MIN_SIDE_CM:
        target_x = attack_dir * BOUNCE_SETUP_X_CM
        target_y = side * BOUNCE_SETUP_Y_CM
        carry_yaw = float(UnitedPosition.Pos2Angle([x, y], [target_x, target_y])) % 360.0
        err_y = target_y - y
        err_x = target_x - x
        speed_x = max(-BOUNCE_CARRY_SPEED, min(BOUNCE_CARRY_SPEED, err_x * 0.01))
        speed_y = max(-BOUNCE_CARRY_SPEED, min(BOUNCE_CARRY_SPEED, err_y * 0.012))
        _safe_field_move_vector(speed_x, speed_y, carry_yaw, KpW=0.35)
        return "setup_side"

    #Step D: 限制在后场/中后场
    #已经很靠前时弹墙收益低，先撤回到起飞带再打。
    if progress > BOUNCE_MAX_PROGRESS_CM:
        target_x = attack_dir * BOUNCE_SETUP_X_CM
        target_y = side * BOUNCE_SETUP_Y_CM
        carry_yaw = float(UnitedPosition.Pos2Angle([x, y], [target_x, target_y])) % 360.0
        _safe_field_move_vector(
            -attack_dir * BOUNCE_CARRY_SPEED,
            max(-0.12, min(0.12, (target_y - y) * 0.01)),
            carry_yaw,
            KpW=0.35,
        )
        return "setup_depth"

    #Step E: 航向对准镜像点后出脚
    #用绕球转向保持持球；到位后 ShootBall。
    yaw_err = abs(angle_error_deg(yaw, aim_yaw))
    if yaw_err <= BOUNCE_AIM_TOL_DEG and abs(y) >= BOUNCE_MIN_SIDE_CM:
        if not _shoot_if_safe():
            TurnTo_with_ball(_opponent_goal_yaw())
            return "aiming"
        peripheral.Dribble(False)
        logger.success(
            "BackWallBounce shot aim=%.1f yaw=%.1f bounce=%s"
            % (aim_yaw, yaw, [round(geo["bounce"][0], 1), round(geo["bounce"][1], 1)])
        )
        return "shoot"

    TurnTo_with_ball(aim_yaw)
    return "aiming"

def activate_func(input_score):
    x = input_score
    y1 = 1 / (1 + math.e ** (-x))  # sigmoid
    # y11=math.e**(-x)/((1+math.e**(-x))**2)
    y11 = 1 / (2 + math.e ** (-x)+ math.e ** (x))  # sigmoid的导数
    y2 = (math.e ** (x) - math.e ** (-x)) / (math.e ** (x) + math.e ** (-x))  # tanh
    y22 = 1-y2*y2  # tanh函数的导数
    y3 = np.where(x < 0, 0, x)  # relu
    y33 = np.where(x < 0, 0, 1)  # ReLU函数导数
    if input_score < 0:#原先为return y33
        return 0.0
    else:
        return 1.0

def UplaneScore(robot_pose):#上平面射门评分，基于位置坐标和朝向角度
    x, y, w = robot_pose
    x = x + 120
    w =  (math.sin(math.radians(w)))
    if y < 0:
        w = - w
    return round((abs(x/240)*0.2 - abs(y/90)*0.5 + w),2)

def BowScore(robot_pose):#弧线射门评分
    x, y, w = robot_pose
    aim = (UnitedPosition.Pos2Angle([x,y],[100,0])+180)%360
    # 必须用环绕后的角度差：w 和 aim 都在 [0,360)，直接相减会把"差 2 度"算成"差 358 度"。
    error_aUw = abs(angle_error_deg(w, aim))
    return round((activate_func((360-error_aUw)/360-0.5)*0.8 - abs(x/120)*0.1 + abs(y/90)*0.1),2)

def HuckScore(robot_pose):#大脚射门评分，考虑面向对方球门程度
    x, y, w = robot_pose
    aim = UnitedPosition.Pos2Angle([x,y],[100,0])%360
    error_aUw = abs(angle_error_deg(w, aim))
    return round((activate_func((360-error_aUw)/360-0.5)*0.8 + abs(x/120)*0.1 - abs(y/90)*0.1),2)

def JumpScore(robot_pose):#弹墙候选评分（原跳射），偏后场+边路
    x, y, w = robot_pose
    if y > 0:
        aim_pose = [100,180]
    else:
        aim_pose = [100,-180]
    aim_w = UnitedPosition.Pos2Angle([x,y],aim_pose)%360
    error_aUw = abs(angle_error_deg(w, aim_w))
    return round((activate_func((360-error_aUw)/360-0.95) - (x/240)*0.5),2)

UPPLANE_SHOT_REGION = [[50, 30], [100, 90]]   # 上平面射门区域的两个对角点（不改动它）

def UplaneStep(thershold = None):
    # 注意：这里原来把默认参数直接当工作副本用（默认值是全进程唯一的对象，
    # 被就地取负后会逐次翻转区域，导致下半场的射门区域每隔一拍跳到上半场）。
    # 现在统一在本地做一份拷贝，默认值只读。
    if thershold is None:
        thershold = [list(point) for point in UPPLANE_SHOT_REGION]
    pose = UnitedPosition.FieldRobotPose()#获取机器人位置
    x, y, _ = pose
    if y < 0:
        angle_k = -1
        # 下半场把区域镜像到 y<0，同样不动传入的对象。
        thershold = [[px, -py] for px, py in thershold]
    else:
        angle_k = 1
    if UnitedPosition.PointInShape(pose,thershold):#判断是否在射门区域内
        if _is_facing_opponent_goal() and UnitedPosition.CanShoot():#在就直接射门
            _shoot_if_safe()
        else:#不在就转向射门角度
            if not _is_facing_opponent_goal():
                TurnTo_with_ball(_opponent_goal_yaw())
            else:
                try:
                    TurnTo_with_ball((-UnitedPosition.ShootWindow()["target_yaw_deg"]))#计算射门角度并转向
                except Exception:
                    TurnTo_with_ball(UnitedPosition.Pos2Angle([x,y],[100,0])%360)#算不出来直接转向特定角度球门
    elif not abs(y) > 50:#不在射门区域内但在中线附近
        if TurnTo_with_ball(90*angle_k):
            _safe_field_move_vector(0,1,90*angle_k)
    else:
        if TurnTo_with_ball(90*angle_k):
            _safe_field_move_vector(0.91,0,90*angle_k)
def _shooting_step():#射门策略选择
    # 位姿只取一次并校验：原来这一支里 FieldRobotPose() 被取了 5 次，
    # 只要有一次返回 None，UplaneScore 里的 `x, y, w = robot_pose` 就会
    # 抛 TypeError 打死主循环。
    robot_pose = UnitedPosition.FieldRobotPose()
    if not _valid_robot_pose(robot_pose):
        chassis.stop()
        return "no_pose"
    logger.success("UplaneScore: "+str(UplaneScore(robot_pose)))#上平面射门评分，基于位置坐标和朝向角度
    logger.success("HuckScore: "+str(HuckScore(robot_pose)))#大脚射门评分，考虑面向对方球门程度
    logger.success("JumpScore: "+str(JumpScore(robot_pose)))#跳射评分，适合边路位置
    logger.warning("BowScore: "+str(BowScore(robot_pose)))#弧线射门评分
    # time.sleep(0.3)
    # return
    logger.info(UnitedPosition.RobotBallPos())
    if _pose_is_safe():#当前位姿在安全区内（等价于"不在禁区/不越界"）
        if UnitedPosition.BallStatus() != "lost":#球未丢失
            if lockball.LockBallSlip():#成功持球
                uplane_score = UplaneScore(robot_pose)
                huck_score = HuckScore(robot_pose)
                jump_score = JumpScore(robot_pose)
                scores = [
                    uplane_score,
                    huck_score,
                    jump_score]
                logger.info("LockBallSlip executed successfully.")
                # 注意：activate_func 实际是阶跃函数，平局很常见；下面的顺序决定平局归属。
                if max(scores) == huck_score:#大脚射门评分最高
                    logger.info("Exe huck")
                    if _is_facing_opponent_goal() and UnitedPosition.CanShoot():
                        _shoot_if_safe()
                    else:
                        if not _is_facing_opponent_goal():
                            TurnTo_with_ball(_opponent_goal_yaw())
                        else:
                            try:
                                TurnTo_with_ball((-UnitedPosition.ShootWindow()["target_yaw_deg"]))
                            except Exception:
                                TurnTo_with_ball(UnitedPosition.Pos2Angle(robot_pose,[100,0])%360)
                elif max(scores) == uplane_score:#上平面射门评分最高
                    if lockball.slip():
                        UplaneStep()
                elif max(scores) == jump_score:#原跳射分支：改为后场弹墙射门
                    logger.info("Exe back-wall bounce")
                    bounce_state = BackWallBounceStep(attack_dir=ATTACK_DIR)
                    logger.info(f"BackWallBounce state={bounce_state}")
                #elif max(scores) == BowScore(robot_pose):#弧线射门评分最高
                #    print('USE BowScore')#BowScoreshoot()未定义U型射的射门方式和通讯
                #    #BowScoreshoot()未定义U型射的射门方式
        else:
            peripheral.Dribble(False)
            if UnitedPosition.BallStatus() == "lost":
                logger.info(f"Ball position is {BALL_NOT_FOUND}, moving to home position.")
                _safe_navigate_to(Settings.CheckPoints.Home)
    return "shooting"
BALL_CLOSE_DIST = 50.0
BALL_FAR_DIST = 120.0
BALL_FACE_ALIGN_TOL_DEG = 8.0
BALL_FACE_TURN_MAX_DEG_S = 104.5
BALL_FACE_TURN_KP = 3.8
INTERCEPT_SPEED_GAIN = 1.4
LAST_BALL_MEMORY_S = 0.8
SEARCH_GOAL_SWEEP_HALF_DEG = 30.0
SEARCH_GOAL_SWEEP_TOL_DEG = 8.0
last_ball_field = None
last_ball_seen_time = 0.0
search_target_yaw = None
search_sweep_side = 1.0
# ATTACK_DIR 已上移到弹墙参数区
# 拦截球函数，AI定义，需要验证
def treat(ball_pos=None):#威胁值函数，现在没调用
    """威胁值。注意两点：
    1) 现在没有任何地方调用它，流程图里"威胁值小于 5"的分支因此不可达；
    2) 注释已说明：距离应当改成"敌方机器到球"的距离，但这里目前算的是
       机器人与球的相对距离，量纲上不可能达到 5（需要距离 < 0.4cm）。
    这里只修掉除零/裸 except，语义留给你决定。
    """
    try:
        if ball_pos is None:
            ball_pos = UnitedPosition.RobotBallPos()
        distance_cm = math.hypot(float(ball_pos[0]), float(ball_pos[1]))
    except Exception:
        return -1  # 球位置不可用
    if not math.isfinite(distance_cm) or distance_cm <= 1e-6:
        return -1  # 距离为 0 或非有限值，无法计算威胁值
    return 2.0 / distance_cm
def _valid_field_ball(field_pos):
    if field_pos is None:
        return False
    try:
        if len(field_pos) < 2:
            return False
        if list(field_pos[:2]) in (list(BALL_NOT_FOUND), list(BALL_HELD)):
            return False
        x = float(field_pos[0])
        y = float(field_pos[1])
        return math.isfinite(x) and math.isfinite(y) and abs(x) < 1000.0 and abs(y) < 1000.0
    except Exception:
        return False


def _turn_to_ball_before_chase(ball_pos):
    """Turn in place toward the visible ball before issuing chase motion."""
    try:
        ball_x = float(ball_pos[0])
        ball_y = float(ball_pos[1])
    except Exception:
        chassis.stop()
        return False

    ball_angle = math.degrees(math.atan2(ball_y, ball_x))
    if abs(ball_angle) <= BALL_FACE_ALIGN_TOL_DEG:
        return True

    # Use one continuous proportional command: fast for a large error, then
    # naturally slower near the target so the first turn settles without
    # crossing the ball angle.
    speed_w = clamp(
        ball_angle * BALL_FACE_TURN_KP,
        -BALL_FACE_TURN_MAX_DEG_S,
        BALL_FACE_TURN_MAX_DEG_S,
    )
    peripheral.Dribble(False)
    chassis.RobotMoveVector(0.0, 0.0, speed_w)
    return False


def InterceptBall(ball_pos=None, field_pos=None):
    if ball_pos is None:
        ball_pos = UnitedPosition.RobotBallPos()

    if ball_pos == BALL_NOT_FOUND:
        return "lost"

    if UnitedPosition.BallStatus(ball_pos) == "held":
        peripheral.Dribble(True)
        return "ball_held"

    dist = math.hypot(float(ball_pos[0]), float(ball_pos[1]))#

    if not _turn_to_ball_before_chase(ball_pos):
        return "turning_to_ball"

    # Start chase motion only after the robot is facing the ball.

    # 近球：直接使用机器人坐标追球
    # 中距离：继续使用机器人坐标追球
    if dist <= BALL_CLOSE_DIST:
        if lockball.LockBallSlip():
            peripheral.Dribble(True)
            return "ball_held"
        return "locking"

    if dist <= BALL_FAR_DIST:
        lockball.LockBallSlip()
        return "chasing"

    # 远距离：使用场地坐标追球
    if _valid_field_ball(field_pos):
        target_x = float(field_pos[0]) - ATTACK_DIR * 10.0
        target_y = float(field_pos[1])
        _safe_navigate_to(
            [target_x, target_y, 0.0],
            INTERCEPT_SPEED_GAIN,
        )
        return "intercepting"

    # Relative ball coordinates are still useful if field pose is unavailable.
    lockball.LockBallSlip()
    return "chasing"
def _legacy_defence():
    """Compatibility wrapper for callers of the removed defence loop."""
    ball_pos, ball_state, field_pos = read_ball_snapshot()
    return defence(ball_pos, ball_state, field_pos)


def SearchBall():
    """Follow the last seen ball briefly, then sweep the heading."""
    global search_target_yaw, search_sweep_side

    peripheral.Dribble(False)
    now = time.monotonic()
    if last_ball_field is not None and now - last_ball_seen_time <= LAST_BALL_MEMORY_S:
        _safe_navigate_to(
            [last_ball_field[0], last_ball_field[1], 0.0],
            0.8,
        )
        return "last_ball_search"

    try:
        pose = UnitedPosition.FieldRobotPose()
        if not pose or len(pose) < 3 or any(abs(float(value)) >= 1000.0 for value in pose[:2]):
            chassis.stop()
            return "no_pose"
        goal_yaw = 0.0 if ATTACK_DIR >= 0 else 180.0
        current_yaw = float(pose[2])
        goal_error = angle_error_deg(current_yaw, goal_yaw)

        # Keep the search heading inside the opponent-goal +/-50 degree
        # window. If the robot starts outside it, return to the goal direction.
        if abs(goal_error) > SEARCH_GOAL_SWEEP_HALF_DEG:
            search_target_yaw = goal_yaw
        elif search_target_yaw is None or abs(angle_error_deg(current_yaw, search_target_yaw)) <= SEARCH_GOAL_SWEEP_TOL_DEG:
            if search_target_yaw is not None:
                search_sweep_side *= -1.0
            search_target_yaw = (
                goal_yaw + search_sweep_side * SEARCH_GOAL_SWEEP_HALF_DEG
            ) % 360.0
        chassis.FieldTurn(search_target_yaw)
        return "sweeping"
    except Exception:
        chassis.stop()
        return "no_pose"


def defence(ball_pos, ball_state, field_pos):
    """Dispatch ball state without a distance-based chase gate."""
    # 队友刚丢球的那一拍不要再执行"让位支援"：球已经变自由了，本机应该
    # 立刻参与争抢，而不是继续往支援位退。
    event = _team_take_event(TEAM_STATE_MAX_AGE_S)
    teammate_just_lost = bool(event and event[0] == "teammate_lost_ball")
    if not teammate_just_lost and _teammate_holds_ball():
        # 流程图分支：另一台机器已持球 -> 本机不再双机抢球，退到支援位等传球。
        return _team_support_step()
    if ball_state == "free":
        return InterceptBall(ball_pos, field_pos)
    if ball_state == "lost":
        return SearchBall()
    return "held"


def logic_step():
    """Execute one bounded gameplan tick using the current simulator state."""
    if not _motion_safety_guard():
        # 安全恢复时吸球器已关闭，按“球权丢失”上报，避免队友误判本机还在控球。
        _team_tick("lost")
        return "safety_recovery"

    ball_pos, ball_state, field_pos = read_ball_snapshot()
    # 每个控制周期收发一次队内状态：先清空接收队列取最新帧，再按 50ms 或状态跳变广播。
    _team_tick(ball_state)
    if ball_state == "held":
        return _shooting_step()
    return defence(ball_pos, ball_state, field_pos)


def main(period_s=0.02):
    """Run the gameplan at the simulator control cadence."""
    deadline = time.monotonic()
    try:
        while True:
            # 单拍异常只记日志，不让主循环整体退出。原来 main() 只捕获
            # KeyboardInterrupt，任意一次 TypeError 都会让机器人彻底停摆。
            try:
                logic_step()
            except Exception as error:
                logger.warning("Gameplan tick failed: %s" % error)
            now = time.monotonic()
            deadline += float(period_s)
            # 单拍耗时持续超过 period_s 时，deadline 会永久落后，
            # max(0.0, 负数) 恒为 0 -> 退化成不休眠的忙循环。落后了就重锚。
            if deadline < now:
                deadline = now
            time.sleep(max(0.0, deadline - now))
    except KeyboardInterrupt:
        pass
    finally:
        safe_stop(chassis, peripheral, UnitedPosition)


if __name__ == "__main__":
    main()

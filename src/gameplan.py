import sys
from pathlib import Path
from BasicFuc import *
from LunaPre import Settings, UnitedPosition, chassis, logger, peripheral
from LunaPre.strategy_tools import BALL_NOT_FOUND, safe_stop
import numpy as np

# 将 src 目录添加到 Python 搜索路径，确保能找到 LunaPre 模块
SRC_DIR = Path(__file__).resolve().parents[1]
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

import time
import math
from LunaPre import chassis, runtime, cfg
from LunaPre.positions import Positions

# 初始化定位系统
UnitedPosition = Positions()

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

def activate_func(input_score):
    x = input_score
    y1 = 1 / (1 + math.e ** (-x))  # sigmoid
    # y11=math.e**(-x)/((1+math.e**(-x))**2)
    y11 = 1 / (2 + math.e ** (-x)+ math.e ** (x))  # sigmoid的导数
    y2 = (math.e ** (x) - math.e ** (-x)) / (math.e ** (x) + math.e ** (-x))  # tanh
    y22 = 1-y2*y2  # tanh函数的导数
    y3 = np.where(x < 0, 0, x)  # relu
    y33 = np.where(x < 0, 0, 1)  # ReLU函数导数
    return y33

def UplaneScore(pose):#上平面射门评分，基于位置坐标和朝向角度
    x, y, w = pose
    x = x + 120
    w =  (math.sin(math.radians(w)))
    if y < 0:
        w = - w
    print(w)
    return round((abs(x/240)*0.2 - abs(y/90)*0.5 + w),2)

def BowScore(pose):#弧线射门评分
    x, y, w = pose
    aim = (UnitedPosition.Pos2Angle([x,y],[100,0])+180)%360
    error_aUw = abs(w - aim)
    return round((activate_func((360-error_aUw)/360-0.5)*0.8 - abs(x/120)*0.1 + abs(y/90)*0.1),2)

def HuckScore(pose):#大脚射门评分，考虑面向对方球门程度
    x, y, w = pose
    aim = UnitedPosition.Pos2Angle([x,y],[100,0])%360
    error_aUw = abs(w - aim)
    return round((activate_func((360-error_aUw)/360-0.5)*0.8 + abs(x/120)*0.1 - abs(y/90)*0.1),2)

def JumpScore(pose):#跳射评分，适合边路位置
    x, y, w = pose
    if y > 0:
        aim_pose = [100,180]
    else:
        aim_pose = [100,-180]
    aim_w = UnitedPosition.Pos2Angle([x,y],aim_pose)%360
    error_aUw = abs(w - aim_w)
    return round((activate_func((360-error_aUw)/360-0.95) - (x/240)*0.5),2)

def UplaneStep(thershold = [[50,30],[100,90]]):
    pose = UnitedPosition.FieldRobotPose()#方向获取与Y方向处理
    x, y, _ = pose
    if y < 0:
        angle_k = -1
        for t in range(len(thershold)):
            thershold[t-1][1] = -thershold[t-1][1]
            try:
                thershold[t-1][2] = -thershold[t-1][2] %360
            except:
                pass
    else:
        angle_k = 1
    if UnitedPosition.PointInShape(pose,thershold):#判断是否在射门区域内
        if UnitedPosition.CanShoot():#在就直接射门
            peripheral.ShootBall()
        else:#不在就转向射门角度
            try:
                TurnTo_with_ball((-UnitedPosition.ShootWindow()["target_yaw_deg"]))#计算射门角度并转向
            except:
                TurnTo_with_ball(UnitedPosition.Pos2Angle([x,y],[100,0])%360)#算不出来直接转向特定角度球门
    elif not abs(y) > 50:#不在射门区域内但在中线附近
        if TurnTo_with_ball(90*angle_k):
            chassis.FieldMoveVector(0,1,90*angle_k)
    else:
        if TurnTo_with_ball(90*angle_k):
            chassis.FieldMoveVector(0.91,0,90*angle_k)
def logic_step():#射门策略选择
    logger.success("UplaneScore: "+str(UplaneScore(UnitedPosition.FieldRobotPose())))#上平面射门评分，基于位置坐标和朝向角度
    logger.success("HuckScore: "+str(HuckScore(UnitedPosition.FieldRobotPose())))#大脚射门评分，考虑面向对方球门程度
    logger.success("JumpScore: "+str(JumpScore(UnitedPosition.FieldRobotPose())))#跳射评分，适合边路位置
    logger.warning("BowScore: "+str(BowScore(UnitedPosition.FieldRobotPose())))#弧线射门评分
    # time.sleep(0.3)
    # return
    logger.info(UnitedPosition.RobotBallPos())
    if UnitedPosition.RestrictedArea():#检查机器人石否在禁区内
        if not UnitedPosition.RobotBallPos() == BALL_NOT_FOUND:#球未丢失
            if lockball.LockBallSlip():#成功持球
                pose = UnitedPosition.FieldRobotPose()
                uplane_score = UplaneScore(pose)
                huck_score = HuckScore(pose)
                jump_score = JumpScore(pose)
                scores = [
                    uplane_score,
                    huck_score,
                    jump_score]
                logger.info("LockBallSlip executed successfully.")
                if max(scores) == huck_score:#大脚射门评分最高
                    logger.info("Exe huck")
                    if UnitedPosition.CanShoot():
                        peripheral.ShootBall()
                    else:
                        try:
                            TurnTo_with_ball((-UnitedPosition.ShootWindow()["target_yaw_deg"]))
                        except:
                            TurnTo_with_ball(UnitedPosition.Pos2Angle(pose,[100,0])%360)
                elif max(scores) == uplane_score:#上平面射门评分最高
                    if lockball.slip():
                        UplaneStep()
                elif max(scores) == jump_score:#跳射评分最高
                    peripheral.ShootBall()
                elif max(scores) == BowScore(pose):#弧线射门评分最高
                    print('USE BowScore')#BowScoreshoot()未定义U型射的射门方式和通讯
                    #BowScoreshoot()未定义U型射的射门方式
        else:
            peripheral.Dribble(False)
            if UnitedPosition.RobotBallPos() == BALL_NOT_FOUND:
                logger.info(f"Ball position is {BALL_NOT_FOUND}, moving to home position.")
                UnitedPosition.Pos2Pos(Settings.CheckPoints.Home)
BALL_CLOSE_DIST = 50.0
BALL_FAR_DIST = 120.0
INTERCEPT_SPEED_GAIN = 1.4
ATTACK_DIR = 1  # 攻击方向
# 拦截球函数，AI定义，需要验证
threat_value=-1
def treat():#威胁值函数
    try:
        distance_cm = math.hypot(ball_pos[0], ball_pos[1])
        threat_value=2*distance_cm**-1#威胁值计算，距离越近威胁值越大,这个距离要改成敌方机器离球的距离。
        return threat_value
    except:
        threat_value = -1  # 如果球位置不可用，设置威胁值为 -1
        return threat_value
def InterceptBall():
    """拦截球函数"""
    ball_pos = UnitedPosition.RobotBallPos()
    ball_field = UnitedPosition.FieldBallPos()
    
    ball_found = (ball_pos != BALL_NOT_FOUND) or (ball_field and ball_field[0] < 1000)
    
    if ball_pos != BALL_NOT_FOUND and ball_pos != BALL_HELD:
        dist_rb = math.hypot(ball_pos[0], ball_pos[1])
    else:
        dist_rb = 999.0
    
    approach_yaw = 0.0 if ATTACK_DIR >= 0 else 180.0
    
    if not ball_found or dist_rb > BALL_FAR_DIST:
        if ball_field and ball_field[0] < 1000:
            target_x = float(ball_field[0]) - ATTACK_DIR * 10.0
            target_y = float(ball_field[1])
            UnitedPosition.Pos2Pos([target_x, target_y, approach_yaw], INTERCEPT_SPEED_GAIN)
        return "intercepting"
    
    if dist_rb > BALL_CLOSE_DIST:
        if ball_field and ball_field[0] < 1000:
            target_x = float(ball_field[0]) - ATTACK_DIR * 5.0
            target_y = float(ball_field[1])
            UnitedPosition.Pos2Pos([target_x, target_y, approach_yaw], INTERCEPT_SPEED_GAIN)
        return "intercepting"
    
    if lockball.LockBallSlip():
        peripheral.Dribble(True)
        return "ball_held"
    
    return "intercepting"
def defence():
    # 获取机器人位置
        robot_pose = UnitedPosition.FieldRobotPose()
        robot_x, robot_y, robot_yaw = robot_pose
        
        # 获取球位置
        ball_pos = UnitedPosition.RobotBallPos()
        ball_state = UnitedPosition.BallState(ball_pos)
        robot_defence=0#通讯，这个变量没定义
        if other_robot_defence==0 :#通讯，这个变量没定义
            print('defence')
            if ball_state == BALL_NOT_FOUND:
                # 如果球未找到，移动到防守位置
                defence_position = [0, 30]  # 防守位置坐标（根据实际情况调整）
                UnitedPosition.Pos2Pos(defence_position)
            else:
                if threat_value < 1/15:
                    InterceptBall()
        else:
            if threat_value < 1/15:#############pid怎么改？
                ball_pos = UnitedPosition.RobotBallPos()
                if UnitedPosition.BallState(ball_pos) == "free":
                    # 使用 PID 计算 Y 轴移动速度，保持在球的正对面
                    speed_y = pan_y(ball_pos[1]) 
                    # 保持航向角为 0 (面向对方球门)，沿 Y 轴进行 PID 修正移动
                    chassis.FieldMoveVector(0, -speed_y, 0)
                else:
                    InterceptBall()

                
#def cooperate():
#    # 获取机器人位置
#        robot_pose = UnitedPosition.FieldRobotPose()
#        robot_x, robot_y, robot_yaw = robot_pose
#        
#        # 获取球位置
#        ball_pos = UnitedPosition.RobotBallPos()
#        ball_state = UnitedPosition.BallState(ball_pos)
        
            
            
while(1):
    # 获取机器人位置
    robot_pose = UnitedPosition.FieldRobotPose()
    robot_x, robot_y, robot_yaw = robot_pose
    
    # 获取球位置
    ball_pos = UnitedPosition.RobotBallPos()
    ball_state = UnitedPosition.BallState(ball_pos)
    treat()
    if distance_cm < 15:
        print('catchball')#通讯还没写
        logic_step()
    else:    
        defence()#防守
        #elif other_robot_catch==1:#通讯
        #    cooperate()#协作
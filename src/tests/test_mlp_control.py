"""测试MLP控制 - 验证机器人能否移动"""

import os
import sys
import time

# 设置环境变量
os.environ["ROBOCUPX_BITS"] = "SIM"

# 添加src到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from utils.XeleratorSimBits import XeleratorSimBits

def test_connection():
    """测试连接"""
    print("=" * 50)
    print("测试1: 连接Unity仿真器")
    print("=" * 50)
    
    bits = XeleratorSimBits(
        unity_ip="127.0.0.1",
        cmd_port=20001,
        status_port=20002,
        robot_id=1,
    )
    
    print(f"连接成功!")
    print(f"  cmd_port: {bits.cmd_port}")
    print(f"  status_port: {bits.status_port}")
    print(f"  robot_id: {bits.robot_id}")
    
    return bits

def test_get_state(bits, duration=3.0):
    """测试状态获取"""
    print("\n" + "=" * 50)
    print("测试2: 获取状态 (持续3秒)")
    print("=" * 50)
    
    start = time.time()
    count = 0
    
    # 诊断信息
    print(f"  robot_id: {bits.robot_id}")
    print(f"  cmd_port: {bits.cmd_port}")
    print(f"  status_port: {bits.status_port}")
    
    while time.time() - start < duration:
        # 获取位姿
        x, y, yaw = bits.GetPose()
        
        # 获取球位置
        ball_pos = bits.GetBallPos()
        has_ball = (ball_pos[0] == 0xDDD and ball_pos[1] == 0xDDD)
        
        # 获取敌方位置
        enemies = bits.GetEnemyPos()
        
        count += 1
        
        if count % 30 == 0:  # 每秒打印一次
            print(f"[{time.time()-start:.1f}s] "
                  f"pos=({x:.1f}, {y:.1f}) yaw={yaw:.1f} | "
                  f"ball=({ball_pos[0]:.1f}, {ball_pos[1]:.1f}) has_ball={has_ball} | "
                  f"enemies={len(enemies)}")
        
        time.sleep(1.0 / 30.0)
    
    print(f"状态获取频率: {count / duration:.1f} Hz")
    
    # 检查是否有收到数据
    rx_count = bits._rx_count
    tx_count = bits._tx_count
    print(f"  TX packets: {tx_count}")
    print(f"  RX packets: {rx_count}")
    
    return True

def test_movement(bits, duration=5.0):
    """测试移动"""
    print("\n" + "=" * 50)
    print("测试3: 移动机器人 (前进3秒，后退2秒)")
    print("=" * 50)
    
    # 前进
    print("前进中...")
    start = time.time()
    while time.time() - start < 3.0:
        bits.SetTwist(0.5, 0.0, 0.0)  # 前进 0.5 m/s
        x, y, yaw = bits.GetPose()
        if int((time.time() - start) * 10) % 10 == 0:
            print(f"  pos=({x:.1f}, {y:.1f}) yaw={yaw:.1f}")
        time.sleep(1.0 / 50.0)
    
    # 停止
    bits.SetTwist(0.0, 0.0, 0.0)
    time.sleep(0.5)
    
    # 后退
    print("后退中...")
    start = time.time()
    while time.time() - start < 2.0:
        bits.SetTwist(-0.3, 0.0, 0.0)  # 后退 0.3 m/s
        x, y, yaw = bits.GetPose()
        if int((time.time() - start) * 10) % 10 == 0:
            print(f"  pos=({x:.1f}, {y:.1f}) yaw={yaw:.1f}")
        time.sleep(1.0 / 50.0)
    
    # 停止
    bits.SetTwist(0.0, 0.0, 0.0)
    
    x, y, yaw = bits.GetPose()
    print(f"最终位置: ({x:.1f}, {y:.1f}) yaw={yaw:.1f}")
    return True

def test_rotation(bits, duration=4.0):
    """测试旋转"""
    print("\n" + "=" * 50)
    print("测试4: 旋转机器人 (左转2秒，右转2秒)")
    print("=" * 50)
    
    # 左转
    print("左转中...")
    start = time.time()
    while time.time() - start < 2.0:
        bits.SetTwist(0.0, 0.0, 1.0)  # 左转 1.0 rad/s
        x, y, yaw = bits.GetPose()
        if int((time.time() - start) * 10) % 10 == 0:
            print(f"  yaw={yaw:.1f}")
        time.sleep(1.0 / 50.0)
    
    # 停止
    bits.SetTwist(0.0, 0.0, 0.0)
    time.sleep(0.5)
    
    # 右转
    print("右转中...")
    start = time.time()
    while time.time() - start < 2.0:
        bits.SetTwist(0.0, 0.0, -1.0)  # 右转 1.0 rad/s
        x, y, yaw = bits.GetPose()
        if int((time.time() - start) * 10) % 10 == 0:
            print(f"  yaw={yaw:.1f}")
        time.sleep(1.0 / 50.0)
    
    # 停止
    bits.SetTwist(0.0, 0.0, 0.0)
    
    x, y, yaw = bits.GetPose()
    print(f"最终yaw: {yaw:.1f}")
    return True

def test_dribble_kick(bits, duration=3.0):
    """测试盘球和踢球"""
    print("\n" + "=" * 50)
    print("测试5: 盘球和踢球")
    print("=" * 50)
    
    # 开启盘球
    print("开启盘球...")
    start = time.time()
    while time.time() - start < 2.0:
        bits.VirtualPorts(4, 1.0)  # Dribble on
        has_ball = bits.HasBall()
        if int((time.time() - start) * 10) % 10 == 0:
            print(f"  has_ball={has_ball}")
        time.sleep(1.0 / 50.0)
    
    # 踢球
    print("踢球!")
    bits.VirtualPorts(3, 0)  # Kick trigger
    time.sleep(0.1)
    bits.VirtualPorts(3, 1)  # Release
    
    # 关闭盘球
    bits.VirtualPorts(4, 0.0)
    print("完成")
    return True

def main():
    print("\n" + "=" * 60)
    print(" MLP 控制测试 - 验证机器人能否移动")
    print("=" * 60)
    print("\n请确保Unity仿真器已启动!")
    print("等待3秒后开始测试...\n")
    time.sleep(3.0)
    
    try:
        # 测试连接
        bits = test_connection()
        
        # 测试状态获取
        test_get_state(bits)
        
        # 测试移动
        test_movement(bits)
        
        # 测试旋转
        test_rotation(bits)
        
        # 测试盘球踢球
        test_dribble_kick(bits)
        
        print("\n" + "=" * 60)
        print(" 所有测试完成!")
        print("=" * 60)
        
        # 关闭连接
        bits.Close()
        
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())
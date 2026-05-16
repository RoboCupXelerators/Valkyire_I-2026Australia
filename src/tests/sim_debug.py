"""模拟器通信诊断脚本"""
import os
import sys
import time

# 设置环境
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

def main():
    print("=" * 50)
    print("模拟器通信诊断")
    print("=" * 50)
    
    # 1. 检查配置
    print("\n[1] 检查配置...")
    try:
        from ReasonData import Settings
        print(f"  RoboInfo.Bit: {Settings.RoboInfo.Bit}")
        print(f"  Sim.UnityIP: {getattr(Settings.Sim, 'UnityIP', '127.0.0.1')}")
        print(f"  Sim.RobotId: {getattr(Settings.Sim, 'RobotId', 1)}")
        print(f"  Sim.PortBase: {getattr(Settings.Sim, 'PortBase', 20001)}")
    except Exception as e:
        print(f"  配置加载失败: {e}")
        return
    
    # 2. 初始化 Bits
    print("\n[2] 初始化 XeleratorSimBits...")
    try:
        from utils.XeleratorSimBits import XeleratorSimBits
        bits = XeleratorSimBits(
            unity_ip="127.0.0.1",
            cmd_port=20001,
            status_port=20002,
            robot_id=1
        )
        print("  Bits 初始化成功!")
        print(f"  cmd_port: {bits.cmd_port}")
        print(f"  status_port: {bits.status_port}")
        print(f"  robot_id: {bits.robot_id}")
    except Exception as e:
        print(f"  Bits 初始化失败: {e}")
        return
    
    # 3. 测试 UDP 通信
    print("\n[3] 测试 UDP 通信 (等待 2 秒)...")
    print("  如果 Unity 模拟器未启动，将看不到数据更新...")
    
    start_time = time.time()
    rx_count_start = bits._rx_count
    tx_count_start = bits._tx_count
    
    time.sleep(2.0)
    
    rx_delta = bits._rx_count - rx_count_start
    tx_delta = bits._tx_count - tx_count_start
    
    print(f"  发送包数: {tx_delta}")
    print(f"  接收包数: {rx_delta}")
    
    if rx_delta == 0:
        print("\n  ⚠️  警告: 未收到任何来自 Unity 的数据包!")
        print("  可能原因:")
        print("    1. Unity 模拟器未启动")
        print("    2. Unity 模拟器端口配置不匹配")
        print("    3. 防火墙阻止 UDP 通信")
    else:
        print(f"\n  ✓ 正常: 收到 {rx_delta} 个数据包")
    
    # 4. 检查位置数据
    print("\n[4] 检查位置数据...")
    for i in range(3):
        x, y, yaw = bits.GetPose()
        has_ball = bits.HasBall()
        ball_pos = bits.GetBallPos()
        print(f"  [{i+1}] Pose: x={x:.2f}cm, y={y:.2f}cm, yaw={yaw:.2f}°")
        print(f"      HasBall: {has_ball}, BallPos: {ball_pos}")
        time.sleep(0.5)
    
    # 5. 检查校准文件
    print("\n[5] 检查 Unity 场景校准...")
    try:
        from utils.XeleratorSimBits import XeleratorSimBits
        ox, oy = XeleratorSimBits._try_guess_origin_from_unity_scene()
        print(f"  origin_x_forward_cm: {ox}")
        print(f"  origin_y_left_cm: {oy}")
        
        calib_file = XeleratorSimBits._calibration_file()
        print(f"  校准文件路径: {calib_file}")
        print(f"  校准文件存在: {calib_file.exists()}")
        
        if calib_file.exists():
            import json
            data = json.loads(calib_file.read_text(encoding="utf-8"))
            print(f"  校准文件内容: {data}")
    except Exception as e:
        print(f"  校准检查失败: {e}")
    
    # 6. 测试命令发送
    print("\n[6] 测试命令发送...")
    bits.SetTwist(0.5, 0.0, 0.0)  # 向前移动
    print("  发送移动命令: vx=0.5, vy=0, wz=0")
    time.sleep(1.0)
    
    x, y, yaw = bits.GetPose()
    print(f"  移动后位置: x={x:.2f}cm, y={y:.2f}cm, yaw={yaw:.2f}°")
    
    bits.SetTwist(0, 0, 0)  # 停止
    print("  发送停止命令")
    
    # 清理
    bits.Close()
    
    print("\n" + "=" * 50)
    print("诊断完成")
    print("=" * 50)

if __name__ == "__main__":
    main()
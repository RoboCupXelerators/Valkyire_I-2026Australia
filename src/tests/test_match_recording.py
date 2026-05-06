#!/usr/bin/env python3
"""测试比赛记录和回放功能

使用方法:
    # 测试数据库初始化
    python test_match_recording.py --init
    
    # 测试模拟数据记录（不需要 Unity）
    python test_match_recording.py --mock
    
    # 列出所有会话
    python test_match_recording.py --list
    
    # 导出最新会话为 JSON
    python test_match_recording.py --export
"""

import os
import sys
import time
import argparse

# 添加 src 目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def test_database_init():
    """测试数据库初始化"""
    print("=" * 60)
    print("测试数据库初始化")
    print("=" * 60)
    
    try:
        from ReasonData.data import SetupDatabase, Positions, Outputs, MatchSession
        
        # 初始化数据库
        SetupDatabase()
        print("✓ 数据库初始化成功")
        
        # 检查表是否存在
        tables = [Positions, Outputs, MatchSession]
        for table in tables:
            count = table.select().count()
            print(f"  - {table.__name__}: {count} 条记录")
        
        return True
        
    except Exception as e:
        print(f"✗ 数据库初始化失败: {e}")
        import traceback
        traceback.print_exc()
        return False


def test_mock_recording(duration_seconds: int = 5):
    """使用模拟数据测试记录功能"""
    print("=" * 60)
    print(f"测试模拟数据记录 ({duration_seconds} 秒)")
    print("=" * 60)
    
    try:
        from ReasonData.data import SetupDatabase, Positions, MatchSession
        import math
        
        # 初始化数据库
        SetupDatabase()
        
        # 创建会话
        session_id = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        session = MatchSession.create_session(session_id, description="Mock test recording")
        print(f"✓ 创建会话: {session_id}")
        
        # 模拟记录数据
        start_time = time.time()
        frame_count = 0
        
        print("开始记录模拟数据...")
        
        while time.time() - start_time < duration_seconds:
            t = time.time() - start_time
            
            # 模拟机器人做圆周运动
            radius = 50.0  # cm
            angular_speed = 0.5  # rad/s
            
            x = radius * math.cos(t * angular_speed)
            y = radius * math.sin(t * angular_speed)
            yaw = math.degrees(t * angular_speed) % 360
            
            # 模拟球位置
            ball_x = 20.0 * math.cos(t * angular_speed * 2)
            ball_y = 20.0 * math.sin(t * angular_speed * 2)
            
            # 记录到数据库
            Positions(
                Tick=time.time(),
                RobotId=1,
                SessionId=session_id,
                XPosition=x,
                YPosition=y,
                YawDeg=yaw,
                Vx=radius * angular_speed * (-math.sin(t * angular_speed)),
                Vy=radius * angular_speed * (math.cos(t * angular_speed)),
                Wz=angular_speed,
                BallXPosition=ball_x,
                BallYPosition=ball_y,
                HasBall=(int(t * 10) % 5 == 0),  # 每 0.5 秒持球
                CreatedAt=time.time(),
            ).save()
            
            frame_count += 1
            time.sleep(0.05)  # 20 Hz
        
        # 结束会话
        session.end_session()
        
        print(f"✓ 记录完成: {frame_count} 帧")
        print(f"✓ 会话已结束: {session_id}")
        
        return session_id
        
    except Exception as e:
        print(f"✗ 模拟记录失败: {e}")
        import traceback
        traceback.print_exc()
        return None


def test_list_sessions():
    """列出所有会话"""
    print("=" * 60)
    print("列出所有比赛会话")
    print("=" * 60)
    
    try:
        from ReasonData.data import MatchSession, Positions
        
        sessions = MatchSession.select().order_by(MatchSession.StartTime.desc())
        
        if not sessions:
            print("  没有找到任何会话")
            return
        
        for s in sessions:
            # 统计帧数
            frame_count = Positions.select().where(Positions.SessionId == s.SessionId).count()
            
            duration = 0
            if s.EndTime and s.StartTime:
                duration = s.EndTime - s.StartTime
            
            print(f"\n会话: {s.SessionId}")
            print(f"  描述: {s.Description or '无'}")
            print(f"  帧数: {frame_count}")
            print(f"  时长: {duration:.1f} 秒")
            print(f"  开始: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(s.StartTime))}")
            if s.EndTime:
                print(f"  结束: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(s.EndTime))}")
        
    except Exception as e:
        print(f"✗ 列出会话失败: {e}")
        import traceback
        traceback.print_exc()


def test_export_session(session_id: str = None):
    """导出会话为 JSON"""
    print("=" * 60)
    print("导出会话数据")
    print("=" * 60)
    
    try:
        from ReasonData.data import GetLatestSession, ExportSessionToDict
        import json
        
        if session_id is None:
            session_id = GetLatestSession()
        
        if not session_id:
            print("✗ 没有找到可导出的会话")
            return
        
        print(f"导出会话: {session_id}")
        
        data = ExportSessionToDict(session_id)
        
        output_file = f"replay_{session_id}.json"
        with open(output_file, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        
        print(f"✓ 已导出到: {output_file}")
        print(f"  帧数: {len(data['positions'])}")
        
    except Exception as e:
        print(f"✗ 导出失败: {e}")
        import traceback
        traceback.print_exc()


def test_replay_controller():
    """测试回放控制器"""
    print("=" * 60)
    print("测试回放控制器")
    print("=" * 60)
    
    try:
        from utils.XeleratorReplay import ReplayController
        
        controller = ReplayController()
        
        # 列出会话
        sessions = controller.ListSessions()
        print(f"找到 {len(sessions)} 个会话")
        
        if not sessions:
            print("没有可回放的会话，请先运行 --mock 创建测试数据")
            return
        
        # 加载最新会话
        frame_count = controller.LoadSession()
        
        if frame_count == 0:
            print("✗ 没有加载到任何帧")
            return
        
        info = controller.GetSessionInfo()
        print(f"\n会话信息:")
        print(f"  ID: {info['session_id']}")
        print(f"  帧数: {info['frame_count']}")
        print(f"  时长: {info['duration_seconds']:.1f} 秒")
        print(f"  机器人: {info['robot_ids']}")
        
        # 导出为 JSON
        output_file = controller.ExportToJson()
        print(f"\n✓ 已导出到: {output_file}")
        
    except Exception as e:
        print(f"✗ 回放控制器测试失败: {e}")
        import traceback
        traceback.print_exc()


def main():
    parser = argparse.ArgumentParser(description="测试比赛记录和回放功能")
    parser.add_argument("--init", action="store_true", help="测试数据库初始化")
    parser.add_argument("--mock", action="store_true", help="使用模拟数据测试记录")
    parser.add_argument("--duration", type=int, default=5, help="模拟记录时长（秒）")
    parser.add_argument("--list", action="store_true", help="列出所有会话")
    parser.add_argument("--export", action="store_true", help="导出最新会话为 JSON")
    parser.add_argument("--replay", action="store_true", help="测试回放控制器")
    parser.add_argument("--all", action="store_true", help="运行所有测试")
    
    args = parser.parse_args()
    
    # 如果没有指定任何选项，显示帮助
    if not any([args.init, args.mock, args.list, args.export, args.replay, args.all]):
        parser.print_help()
        return
    
    if args.all:
        args.init = args.mock = args.list = args.export = args.replay = True
    
    success = True
    
    if args.init:
        success = test_database_init() and success
    
    if args.mock:
        session_id = test_mock_recording(args.duration)
        success = (session_id is not None) and success
    
    if args.list:
        test_list_sessions()
    
    if args.export:
        test_export_session()
    
    if args.replay:
        test_replay_controller()
    
    print("\n" + "=" * 60)
    if success:
        print("✓ 所有测试通过")
    else:
        print("✗ 部分测试失败")
    print("=" * 60)


if __name__ == "__main__":
    main()
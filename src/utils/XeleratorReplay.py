"""XeleratorReplay

比赛回放模块 - 从数据库读取历史记录并通过 UDP 发送给 Unity 模拟器。

功能：
- 从数据库读取指定会话的位置数据
- 按时间顺序播放，支持变速播放
- 发送机器人位置和球位置给 Unity

使用方法：
    from utils.XeleratorReplay import ReplayController
    
    replay = ReplayController()
    replay.LoadSession("20260505_150000")
    replay.Play(speed=1.0)
"""

from __future__ import annotations

import json
import socket
import struct
import threading
import time
from pathlib import Path
from typing import Optional, List, Dict, Any, Callable

from dataclasses import dataclass


@dataclass
class ReplayFrame:
    """单帧回放数据"""
    tick: float
    robot_id: int
    x_cm: float
    y_cm: float
    yaw_deg: float
    vx: float
    vy: float
    wz: float
    ball_x_cm: float
    ball_y_cm: float
    has_ball: bool


class ReplayController:
    """比赛回放控制器
    
    从数据库读取历史位置数据，并通过 UDP 发送给 Unity 进行回放。
    """
    
    # Unity 回放协议（扩展 XEL 协议）
    # ReplayPacket: <BBHBBH6f  (32 bytes) + ball extension (8 bytes)
    # 与 StatusPacket 格式相同，但用于回放模式
    VERSION = 1
    TYPE_REPLAY = 3  # 新类型：回放数据
    
    REPLAY_STRUCT = struct.Struct("<BBHBBH6f")  # 32 bytes
    
    def __init__(
        self,
        *,
        unity_ip: str = "127.0.0.1",
        replay_port: int = 20003,  # 回放专用端口
    ):
        self.unity_ip = unity_ip
        self.replay_port = replay_port
        
        self._socket: Optional[socket.socket] = None
        self._frames: List[ReplayFrame] = []
        self._session_id: Optional[str] = None
        self._is_playing: bool = False
        self._play_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._current_frame_idx: int = 0
        self._play_speed: float = 1.0
        self._loop_playback: bool = False
        
    def LoadSession(self, session_id: str = None) -> int:
        """加载比赛会话数据
        
        Args:
            session_id: 会话ID。如果为 None，加载最新的会话。
            
        Returns:
            加载的帧数
        """
        try:
            from ReasonData.data import GetSessionData, GetLatestSession, Positions
        except ImportError:
            print("[REPLAY] Error: ReasonData not available")
            return 0
        
        # 获取会话ID
        if session_id is None:
            session_id = GetLatestSession()
            if session_id is None:
                print("[REPLAY] Error: No sessions found in database")
                return 0
        
        self._session_id = session_id
        
        # 加载数据
        positions = GetSessionData(session_id)
        
        self._frames = []
        for p in positions:
            frame = ReplayFrame(
                tick=p.Tick,
                robot_id=p.RobotId,
                x_cm=p.XPosition,
                y_cm=p.YPosition,
                yaw_deg=p.YawDeg,
                vx=p.Vx,
                vy=p.Vy,
                wz=p.Wz,
                ball_x_cm=p.BallXPosition,
                ball_y_cm=p.BallYPosition,
                has_ball=p.HasBall,
            )
            self._frames.append(frame)
        
        print(f"[REPLAY] Loaded {len(self._frames)} frames from session {session_id}")
        return len(self._frames)
    
    def ListSessions(self) -> List[Dict[str, Any]]:
        """列出所有可用的比赛会话"""
        try:
            from ReasonData.data import MatchSession
            sessions = MatchSession.select().order_by(MatchSession.StartTime.desc())
            return [
                {
                    "session_id": s.SessionId,
                    "start_time": s.StartTime,
                    "end_time": s.EndTime,
                    "description": s.Description,
                }
                for s in sessions
            ]
        except Exception as e:
            print(f"[REPLAY] Error listing sessions: {e}")
            return []
    
    def GetSessionInfo(self) -> Optional[Dict[str, Any]]:
        """获取当前加载会话的信息"""
        if not self._session_id or not self._frames:
            return None
        
        start_tick = self._frames[0].tick if self._frames else 0
        end_tick = self._frames[-1].tick if self._frames else 0
        
        robot_ids = set(f.robot_id for f in self._frames)
        
        return {
            "session_id": self._session_id,
            "frame_count": len(self._frames),
            "duration_seconds": end_tick - start_tick if end_tick > start_tick else 0,
            "robot_ids": list(robot_ids),
        }
    
    def Play(self, speed: float = 1.0, loop: bool = False):
        """开始回放
        
        Args:
            speed: 播放速度（1.0 = 正常，2.0 = 2倍速，0.5 = 半速）
            loop: 是否循环播放
        """
        if not self._frames:
            print("[REPLAY] Error: No frames loaded. Call LoadSession() first.")
            return
        
        if self._is_playing:
            print("[REPLAY] Warning: Already playing. Stop first.")
            return
        
        self._play_speed = max(0.1, min(10.0, speed))
        self._loop_playback = loop
        self._is_playing = True
        self._stop_event.clear()
        
        # 创建 UDP socket
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # 启动播放线程
        self._play_thread = threading.Thread(target=self._play_loop, daemon=True)
        self._play_thread.start()
        
        print(f"[REPLAY] Started playback at {self._play_speed}x speed")
    
    def Pause(self):
        """暂停回放"""
        self._is_playing = False
    
    def Resume(self):
        """恢复回放"""
        if self._frames and not self._is_playing:
            self._is_playing = True
    
    def Stop(self):
        """停止回放"""
        self._is_playing = False
        self._stop_event.set()
        
        if self._socket:
            try:
                self._socket.close()
            except Exception:
                pass
            self._socket = None
        
        if self._play_thread:
            self._play_thread.join(timeout=1.0)
            self._play_thread = None
        
        self._current_frame_idx = 0
        print("[REPLAY] Stopped playback")
    
    def Seek(self, frame_idx: int):
        """跳转到指定帧
        
        Args:
            frame_idx: 帧索引（0-based）
        """
        if 0 <= frame_idx < len(self._frames):
            self._current_frame_idx = frame_idx
        else:
            print(f"[REPLAY] Warning: Invalid frame index {frame_idx}")
    
    def SeekToTime(self, time_seconds: float):
        """跳转到指定时间点
        
        Args:
            time_seconds: 相对于会话开始的时间（秒）
        """
        if not self._frames:
            return
        
        start_tick = self._frames[0].tick
        target_tick = start_tick + time_seconds
        
        # 二分查找最接近的帧
        for i, frame in enumerate(self._frames):
            if frame.tick >= target_tick:
                self._current_frame_idx = i
                return
        
        self._current_frame_idx = len(self._frames) - 1
    
    def GetCurrentFrame(self) -> Optional[ReplayFrame]:
        """获取当前帧"""
        if 0 <= self._current_frame_idx < len(self._frames):
            return self._frames[self._current_frame_idx]
        return None
    
    def GetProgress(self) -> float:
        """获取播放进度（0.0 - 1.0）"""
        if not self._frames:
            return 0.0
        return self._current_frame_idx / len(self._frames)
    
    def _play_loop(self):
        """回放循环（在独立线程中运行）"""
        seq = 0
        
        while not self._stop_event.is_set() and self._is_playing:
            if self._current_frame_idx >= len(self._frames):
                if self._loop_playback:
                    self._current_frame_idx = 0
                else:
                    self._is_playing = False
                    break
            
            frame = self._frames[self._current_frame_idx]
            
            # 发送回放数据
            self._send_replay_frame(frame, seq)
            seq = (seq + 1) & 0xFFFF
            
            # 计算等待时间
            if self._current_frame_idx < len(self._frames) - 1:
                next_frame = self._frames[self._current_frame_idx + 1]
                dt = (next_frame.tick - frame.tick) / self._play_speed
                if dt > 0:
                    # 使用 stop_event.wait 代替 time.sleep，以便能够快速响应停止请求
                    self._stop_event.wait(timeout=dt)
            
            self._current_frame_idx += 1
        
        self._is_playing = False
    
    def _send_replay_frame(self, frame: ReplayFrame, seq: int):
        """发送单帧回放数据到 Unity"""
        if not self._socket:
            return
        
        try:
            # 转换 cm -> m（Unity 单位）
            x_m = frame.x_cm / 100.0
            y_m = frame.y_cm / 100.0
            ball_x_m = frame.ball_x_cm / 100.0
            ball_y_m = frame.ball_y_cm / 100.0
            
            # 构建回放包
            flags = 0x02 if frame.has_ball else 0x00  # 使用 bit 1 表示持球
            
            payload = self.REPLAY_STRUCT.pack(
                self.VERSION,
                self.TYPE_REPLAY,
                seq,
                frame.robot_id,
                flags,
                0,  # reserved
                float(x_m),
                float(y_m),
                float(frame.yaw_deg),
                float(frame.vx),
                float(frame.vy),
                float(frame.wz),
            )
            
            # 添加球位置扩展
            ball_ext = struct.pack("<ff", float(ball_x_m), float(ball_y_m))
            payload = payload + ball_ext
            
            self._socket.sendto(payload, (self.unity_ip, self.replay_port))
            
        except Exception as e:
            print(f"[REPLAY] Error sending frame: {e}")
    
    def ExportToJson(self, output_path: str = None) -> str:
        """导出会话数据为 JSON 文件
        
        Args:
            output_path: 输出文件路径。如果为 None，使用默认路径。
            
        Returns:
            实际保存的文件路径
        """
        if not self._frames:
            print("[REPLAY] Error: No frames to export")
            return ""
        
        if output_path is None:
            output_path = f"replay_{self._session_id}.json"
        
        data = {
            "session_id": self._session_id,
            "frame_count": len(self._frames),
            "frames": [
                {
                    "tick": f.tick,
                    "robot_id": f.robot_id,
                    "x_cm": f.x_cm,
                    "y_cm": f.y_cm,
                    "yaw_deg": f.yaw_deg,
                    "vx": f.vx,
                    "vy": f.vy,
                    "wz": f.wz,
                    "ball_x_cm": f.ball_x_cm,
                    "ball_y_cm": f.ball_y_cm,
                    "has_ball": f.has_ball,
                }
                for f in self._frames
            ]
        }
        
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        
        print(f"[REPLAY] Exported {len(self._frames)} frames to {output_path}")
        return output_path
    
    @staticmethod
    def ImportFromJson(input_path: str) -> 'ReplayController':
        """从 JSON 文件导入会话数据
        
        Args:
            input_path: JSON 文件路径
            
        Returns:
            新的 ReplayController 实例
        """
        with open(input_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        
        controller = ReplayController()
        controller._session_id = data.get("session_id", "imported")
        controller._frames = [
            ReplayFrame(
                tick=frame["tick"],
                robot_id=frame["robot_id"],
                x_cm=frame["x_cm"],
                y_cm=frame["y_cm"],
                yaw_deg=frame["yaw_deg"],
                vx=frame["vx"],
                vy=frame["vy"],
                wz=frame["wz"],
                ball_x_cm=frame["ball_x_cm"],
                ball_y_cm=frame["ball_y_cm"],
                has_ball=frame["has_ball"],
            )
            for frame in data["frames"]
        ]
        
        print(f"[REPLAY] Imported {len(controller._frames)} frames from {input_path}")
        return controller


def main():
    """命令行入口"""
    import argparse
    
    parser = argparse.ArgumentParser(description="Xelerator Match Replay Tool")
    parser.add_argument("--list", action="store_true", help="List available sessions")
    parser.add_argument("--session", "-s", type=str, help="Session ID to replay")
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed (default: 1.0)")
    parser.add_argument("--loop", "-l", action="store_true", help="Loop playback")
    parser.add_argument("--export", "-e", type=str, nargs="?", const=None, help="Export to JSON file")
    parser.add_argument("--import", dest="import_file", type=str, help="Import from JSON file")
    parser.add_argument("--ip", type=str, default="127.0.0.1", help="Unity IP address")
    parser.add_argument("--port", type=int, default=20003, help="Replay port")
    
    args = parser.parse_args()
    
    if args.list:
        controller = ReplayController()
        sessions = controller.ListSessions()
        print("\nAvailable Sessions:")
        print("-" * 60)
        for s in sessions:
            print(f"  {s['session_id']}: {s['description'] or 'No description'}")
        return
    
    controller = ReplayController(unity_ip=args.ip, replay_port=args.port)
    
    if args.import_file:
        controller = ReplayController.ImportFromJson(args.import_file)
        if args.export is not None:
            controller.ExportToJson(args.export)
        return
    
    frame_count = controller.LoadSession(args.session)
    if frame_count == 0:
        print("No data to replay")
        return
    
    info = controller.GetSessionInfo()
    print(f"\nSession: {info['session_id']}")
    print(f"Frames: {info['frame_count']}")
    print(f"Duration: {info['duration_seconds']:.1f} seconds")
    print(f"Robots: {info['robot_ids']}")
    
    if args.export is not None:
        controller.ExportToJson(args.export)
        return
    
    print(f"\nStarting playback at {args.speed}x speed...")
    print("Press Ctrl+C to stop")
    
    try:
        controller.Play(speed=args.speed, loop=args.loop)
        while controller._is_playing:
            time.sleep(0.1)
    except KeyboardInterrupt:
        controller.Stop()
        print("\nPlayback stopped")


if __name__ == "__main__":
    main()
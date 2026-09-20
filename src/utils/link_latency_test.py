#!/usr/bin/env python3
"""双机链路延迟测试：直接收发 0x09/0x0A 原始帧，自动识别长度字段宽度。

写串口后不等 tcdrain：Jetson 的 ttyTHS 上 flush() 一次要 ~10 ms，会把轮询
压到 80 次/秒（去掉后 460 次/秒），而数据本来就由驱动异步发出。

用法（两台机器各跑一份，同时在 --seconds 内收发）:
    python3 link_latency_test.py --rate 20 --seconds 30 --echo
    python3 link_latency_test.py --rate 20 --seconds 30 --echo --pose

时间同步（可选，用于算单向年龄）:
    robot2:   python3 link_latency_test.py --sync-server 50007
    本机:    python3 link_latency_test.py --sync-peer robot2.local:50007 --clock-offset-ms <算出来的值>
"""

import argparse
import glob
import os
import socket
import statistics
import struct
import sys
import threading
import time

import serial

PING = 1
ECHO = 2
MAGIC = b"LT"
CMD_LINK_SEND = 0x09
CMD_LINK_GET = 0x0A
RESP_LINK_SEND = 0x89
RESP_LINK_GET = 0x8A
FLAG_POSE = 0x01


class Link:
    """0x09 / 0x0A 的最小实现，长度字段宽度自动探测。"""

    def __init__(self, port, baudrate=115200, timeout=0.05):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.io_lock = threading.Lock()
        self.len_bytes = 0
        self.resp_header = 0
        self.detect()

    def detect(self):
        probe = MAGIC + bytes([PING]) + struct.pack("<Id", 0, 0.0)
        for width in (1, 2):
            frame = bytes([CMD_LINK_SEND, 0]) + len(probe).to_bytes(width, "little") + probe
            if self.exchange(frame, 1) == bytes([RESP_LINK_SEND]):
                self.len_bytes = width
                self.resp_header = 1 + width
                print(f"链路长度字段 = {width} 字节 (LINK_GET 响应头 {self.resp_header} 字节)", flush=True)
                return
        raise SystemExit("LINK_SEND 无响应：检查串口、STM32 固件与链路配对状态")

    def exchange(self, frame, want):
        with self.io_lock:
            self.ser.reset_input_buffer()
            self.ser.write(frame)
            return self.ser.read(want)

    def send(self, flags, payload):
        frame = bytes([CMD_LINK_SEND, flags]) + len(payload).to_bytes(self.len_bytes, "little") + payload
        return self.exchange(frame, 1) == bytes([RESP_LINK_SEND])

    def poll(self):
        with self.io_lock:
            self.ser.reset_input_buffer()
            self.ser.write(bytes([CMD_LINK_GET]))
            head = self.ser.read(self.resp_header)
            if len(head) != self.resp_header or head[0] != RESP_LINK_GET:
                return None
            length = int.from_bytes(head[1:], "little")
            body = self.ser.read(length) if length else b""
            return body if len(body) == length else None


def make_payload(kind, counter, stamp, pad):
    return MAGIC + bytes([kind]) + struct.pack("<Id", counter, stamp) + bytes(pad)


def parse_payload(data):
    """STM32 回的是整条链路消息：flags [+ 24 字节位姿] + 应用数据。"""
    if not data:
        return None
    offset = 1 + (24 if data[0] & FLAG_POSE else 0)
    body = data[offset:]
    if len(body) < 15 or not body.startswith(MAGIC):
        return None
    kind = body[2]
    counter, stamp = struct.unpack("<Id", body[3:15])
    return kind, counter, stamp


def sync_server(port, rounds=200):
    """回答时间同步请求，返回自己的墙钟。"""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", port))
    srv.listen(1)
    print(f"sync server on :{port}", flush=True)
    conn, _ = srv.accept()
    with conn:
        while True:
            data = conn.recv(64)
            if not data:
                break
            conn.sendall(struct.pack("<d", time.time()))


def sync_client(peer, rounds=200):
    """NTP 式偏移估计：offset = 对端时钟 - 本机时钟。"""
    host, port = peer.rsplit(":", 1)
    sock = socket.create_connection((host, int(port)), timeout=2)
    best = None
    with sock:
        for _ in range(rounds):
            t0 = time.time()
            sock.sendall(b"t")
            remote = struct.unpack("<d", sock.recv(8))[0]
            t1 = time.time()
            rtt = t1 - t0
            offset = remote - (t0 + rtt / 2)
            if best is None or rtt < best[0]:
                best = (rtt, offset)
            time.sleep(0.005)
    print(f"clock offset (peer - local) = {best[1] * 1000:.3f} ms, min rtt = {best[0] * 1000:.3f} ms", flush=True)
    return best[1]


class Runner:
    def __init__(self, args):
        self.args = args
        self.link = Link(args.port, timeout=args.timeout)
        self.offset = args.clock_offset_ms / 1000.0
        self.pad = max(0, args.pad - 15)
        self.sent = {}
        self.received = []
        self.echo_pending = None
        self.sent_count = 0
        self.raw_shown = 0
        self.last_identity = None
        self.polls = 0
        self.poll_bad = 0
        self.lock = threading.Lock()
        self.stop = threading.Event()

    def send_loop(self):
        period = 1.0 / self.args.rate
        next_at = time.monotonic()
        t_end = next_at + self.args.seconds
        while not self.stop.is_set() and time.monotonic() < t_end:
            now = time.time()
            with self.lock:
                self.sent_count += 1
                counter = self.args.counter_base + self.sent_count
            self.sent[counter] = now
            payload = make_payload(PING, counter, now, self.pad)
            if not self.link.send(FLAG_POSE if self.args.pose else 0, payload):
                print("LINK_SEND 未响应", flush=True)
            next_at += period
            delay = next_at - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_at = time.monotonic()

    def poll_loop(self):
        period = 1.0 / self.args.poll_rate
        next_at = time.monotonic()
        while not self.stop.is_set():
            body = self.link.poll()
            now = time.time()
            self.polls += 1
            if body is None:
                self.poll_bad += 1
            if self.args.verbose and body and self.raw_shown < self.args.verbose:
                self.raw_shown += 1
                print(f"raw#{self.raw_shown} len={len(body)} flags={body[0]:#04x} {body[:24].hex(' ')}", flush=True)
            parsed = parse_payload(body)
            if parsed:
                kind, counter, stamp = parsed
                identity = (kind, counter, round(stamp, 6))
                if identity != self.last_identity:
                    self.last_identity = identity
                    with self.lock:
                        self.received.append((now, kind, counter, stamp))
                        index = len(self.received)
                        if kind == PING and self.args.echo:
                            self.echo_pending = counter
                    if self.args.list:
                        age = (now - stamp + self.offset) * 1000
                        print(f"  #{index} kind={kind} counter={counter} age={age:.1f} ms", flush=True)
            if self.echo_pending is not None:
                with self.lock:
                    echo_counter = self.echo_pending
                    self.echo_pending = None
                payload = make_payload(ECHO, echo_counter, time.time(), self.pad)
                self.link.send(FLAG_POSE if self.args.pose else 0, payload)
            next_at += period
            delay = next_at - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_at = time.monotonic()

    def loop_once(self):
        """单线程：一次 LINK_SEND + 一次 LINK_GET，节奏均匀。"""
        period = 1.0 / self.args.rate
        next_at = time.monotonic()
        t_end = next_at + self.args.seconds
        while not self.stop.is_set() and time.monotonic() < t_end:
            now = time.time()
            with self.lock:
                self.sent_count += 1
                counter = self.args.counter_base + self.sent_count
            self.sent[counter] = now
            self.link.send(FLAG_POSE if self.args.pose else 0,
                           make_payload(PING, counter, now, self.pad))
            body = self.link.poll()
            self.polls += 1
            parsed = parse_payload(body)
            if parsed:
                kind, c, stamp = parsed
                identity = (kind, c, round(stamp, 6))
                if identity != self.last_identity:
                    self.last_identity = identity
                    with self.lock:
                        self.received.append((time.time(), kind, c, stamp))
            next_at += period
            delay = next_at - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_at = time.monotonic()

    def run(self):
        if self.args.single_thread:
            self.loop_once()
            self.report()
            return
        threads = [threading.Thread(target=self.poll_loop)]
        if not self.args.no_ping:
            threads.append(threading.Thread(target=self.send_loop))
        for t in threads:
            t.start()
        time.sleep(self.args.seconds)
        self.stop.set()
        for t in threads:
            t.join()
        self.report()

    def report(self):
        with self.lock:
            received = list(self.received)
            sent = dict(self.sent)
            sent_count = self.sent_count

        pings = [r for r in received if r[1] == PING]
        echoes = [r for r in received if r[1] == ECHO]
        gaps = [b[2] - a[2] for a, b in zip(pings, pings[1:])]
        arrivals = [(b[0] - a[0]) * 1000 for a, b in zip(pings, pings[1:])]
        ages = [(r[0] - r[3] + self.offset) * 1000 for r in pings]
        rtts = [(r[0] - sent[r[2]]) * 1000 for r in echoes if r[2] in sent]

        def stats(values, unit="ms"):
            if not values:
                return "n/a"
            values = sorted(values)
            pick = lambda q: values[min(len(values) - 1, int(q * len(values)))]
            return (f"min {values[0]:.1f} p50 {pick(0.5):.1f} p95 {pick(0.95):.1f} "
                    f"max {values[-1]:.1f} {unit} (n={len(values)})")

        print(f"\n=== rate={self.args.rate} Hz poll={self.args.poll_rate} Hz pose={self.args.pose} "
              f"pad={self.args.pad}B echo={self.args.echo} ===", flush=True)
        print(f"发出 ping        : {sent_count}", flush=True)
        print(f"LINK_GET 轮询    : {self.polls} 次，其中新数据 {len(received)} 条，响应异常 {self.poll_bad} 次", flush=True)
        arrived = f"{len(pings) / sent_count * 100:.1f}%" if sent_count else "n/a（本机未发送）"
        print(f"收到 ping        : {len(pings)}   (单向到达率 {arrived})", flush=True)
        print(f"收到 echo        : {len(echoes)}", flush=True)
        print(f"计数器步进       : {stats([float(g) for g in gaps], '包')}", flush=True)
        print(f"到达间隔         : {stats(arrivals)}", flush=True)
        print(f"单向数据年龄     : {stats(ages)}", flush=True)
        print(f"往返 RTT(echo)   : {stats(rtts)}", flush=True)

        if self.args.csv:
            with open(self.args.csv, "w") as fh:
                fh.write("t_local,kind,counter,sender_time,age_ms\n")
                for now, kind, counter, stamp in received:
                    fh.write(f"{now:.6f},{kind},{counter},{stamp:.6f},{(now - stamp + self.offset) * 1000:.3f}\n")
            print(f"CSV: {self.args.csv}", flush=True)


def default_port():
    for pattern in ("/dev/iceloong", "/dev/ttyTHS1", "/dev/serial/by-id/*", "/dev/ttyUSB*"):
        for path in sorted(glob.glob(pattern)):
            return path
    raise SystemExit("找不到串口，请用 --port 指定")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=None, help="STM32 串口，默认自动探测")
    parser.add_argument("--rate", type=float, default=20.0, help="LINK_SEND 频率 Hz")
    parser.add_argument("--counter-base", type=int, default=0,
                        help="计数器起始值，双向同时 ping 时两侧用不同基数避免互相混淆")
    parser.add_argument("--poll-rate", type=float, default=100.0, help="LINK_GET 轮询频率 Hz")
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--pad", type=int, default=15, help="链路 payload 总字节数")
    parser.add_argument("--pose", action="store_true", help="附带本机位姿（+25 字节）")
    parser.add_argument("--echo", action="store_true", help="收到 ping 后立刻回一个 echo")
    parser.add_argument("--no-ping", action="store_true", help="只收不发（可配合 --echo 当应答端）")
    parser.add_argument("--verbose", type=int, default=0, help="打印前 N 条原始 payload")
    parser.add_argument("--list", action="store_true", help="逐条打印收到的消息与年龄")
    parser.add_argument("--single-thread", action="store_true",
                        help="单线程顺序收发（贴近真实上位机节奏）")
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument("--clock-offset-ms", type=float, default=0.0, help="对端时钟 - 本机时钟")
    parser.add_argument("--csv", default=None)
    parser.add_argument("--diag", action="store_true", help="读取 STM32 链路诊断计数（临时命令 0x0D）")
    parser.add_argument("--sync-server", type=int, default=None, help="启动时间同步服务并退出")
    parser.add_argument("--sync-peer", default=None, help="HOST:PORT，估计时钟偏移并退出")
    args = parser.parse_args()

    if args.sync_server:
        sync_server(args.sync_server)
        return
    if args.sync_peer:
        sync_client(args.sync_peer)
        return

    if args.diag:
        link = Link(args.port or default_port())
        with link.io_lock:
            link.ser.reset_input_buffer()
            link.ser.write(bytes([0x0D]))
            resp = link.ser.read(33)
        if len(resp) == 33 and resp[0] == 0x8D:
            names = ("rx_ok", "rx_crc_bad", "rx_len_bad", "rx_type_bad",
                     "poll_tx", "data_tx", "stored", "poll_timeout")
            for name, value in zip(names, struct.unpack("<8I", resp[1:])):
                print(f"{name:14s} {value}")
        else:
            print(f"0x0D 无响应（收到 {len(resp)} 字节）—— STM32 固件没烧诊断版？")
        return

    args.port = args.port or default_port()
    print(f"port={args.port} rate={args.rate}Hz seconds={args.seconds}s pose={args.pose}", flush=True)
    Runner(args).run()


if __name__ == "__main__":
    main()

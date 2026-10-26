#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""车内上行一键采集：回车开始 / 回车结束，每轮自动存一个 CSV。

用法（a-Shell / iSH / 电脑都能跑，纯标准库）：
    python3 cap_menu.py                 # 默认 8086 端口
    python3 cap_menu.py --port 8086

交互式每轮只有三件事：
    回车       -> 开始采集（新文件 wave_YYYYmmdd_HHMMSS.csv）
    回车       -> 停止并保存（打印帧数/时长/丢帧/文件名）
    q + 回车   -> 退出（如果正在采，会先自动保存）

为什么这样设计：iOS 上 a-Shell 切到后台会被挂起，所以要"对着终端按一下"，
不依赖网页/图形界面；socket 在整段会话里只 bind 一次，反复开始结束不会
再撞 "Address already in use"。
"""

import os
import socket
import struct
import sys
import threading
import time

# ---------------------------------------------------------------- 通道表
# 与 KartHostLite 的 Channels.cs 完全同源（名字/单位/顺序），导出的 CSV 能直接
# 丢给电脑端的分析脚本。
NAMES41 = [
    "target_l", "v_l", "u_l", "target_r", "v_r", "u_r", "uff_l", "uff_r",
    "z_l", "z_r", "pos_l", "pos_r", "online", "age_ms", "en_bits", "safety",
    "steer_tgt", "steer_deg", "steer_u", "steer_en", "steer_st", "rc_eng",
    "rc_ch1", "rc_ch2", "rc_frames", "k_v", "k_z", "yaw", "gz",
    "ins_x", "ins_y", "ins_dist", "ins_flags", "rt_cnt", "rt_st", "pb_elat",
    "tar_yaw", "ht_seg", "tick_ms", "ins_frz", "ins_static",
]
UNITS41 = [
    "rps", "rps", "%", "rps", "rps", "%", "%", "%",
    "rps*s", "rps*s", "cnt", "cnt", "-", "ms", "-", "-",
    "deg", "deg", "duty", "-", "-", "-",
    "cnt", "cnt", "-", "-", "-", "deg", "dps",
    "m", "m", "m", "-", "-", "-", "m",
    "deg", "-", "ms", "deg", "-",
]
NAMES18 = ["target_l", "v_l", "u_l", "target_r", "v_r", "u_r",
           "steer_tgt", "steer_deg", "steer_u", "online", "en_bits", "safety",
           "steer_en", "steer_st", "ol_en", "resv0", "k_v", "k_z"]
UNITS18 = ["rps", "rps", "%", "rps", "rps", "%", "deg", "deg", "duty",
           "-", "-", "-", "-", "-", "-", "-", "-", "-"]

TAIL = b"\x00\x00\x80\x7f"


def crc16_modbus(buf):
    c = 0xFFFF
    for x in buf:
        c ^= x
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if (c & 1) else (c >> 1)
    return c & 0xFFFF


class Cap(object):
    """一个常驻的 UDP 接收器 + 可反复开合的 CSV 记录器。"""

    def __init__(self, port):
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # ★ 不开 SO_REUSEADDR：UDP 没有 TIME_WAIT，绑不上就是真有进程还占着端口。
        #   开着的坏处是"旧进程还在收包时新进程也能绑上"，两个进程各拿一半帧、谁都不报错。
        self.sock.bind(("0.0.0.0", port))
        self.sock.settimeout(0.3)

        self.lock = threading.Lock()
        self.stop_flag = threading.Event()
        self.thr = None

        self.reset_stats()

    # ------------------------------------------------------------ 统计
    def reset_stats(self):
        self.f = None
        self.path = None
        self.cnt_expected = None
        self.frames = 0
        self.bad = 0
        self.other = 0          # 非遥测帧（参数表等），只计数
        self.lost = 0
        self.seq_prev = None
        self.ts_first = None
        self.ts_last = None
        self.t_start = None
        self.src = None

    @property
    def active(self):
        return self.f is not None

    # ------------------------------------------------------------ 开/关
    def start(self):
        with self.lock:
            if self.active:
                return None
            self.reset_stats()
            self.path = "wave_%s.csv" % time.strftime("%Y%m%d_%H%M%S")
            self.f = open(self.path, "w", encoding="utf-8-sig", newline="")
            self.t_start = time.time()

            if self.thr is None or not self.thr.is_alive():
                self.stop_flag.clear()
                self.thr = threading.Thread(target=self._rx_loop, daemon=True)
                self.thr.start()

            return self.path

    def stop(self):
        with self.lock:
            if not self.active:
                return None
            try:
                self.f.flush()
                self.f.close()
            except OSError:
                pass
            self.f = None
            return self.path

    # ------------------------------------------------------------ 收
    def _rx_loop(self):
        while not self.stop_flag.is_set():
            try:
                buf, addr = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break

            with self.lock:
                if not self.active:
                    continue
                try:
                    self._feed(buf, addr)
                except (struct.error, ValueError):
                    self.bad += 1

    def _feed(self, b, addr):
        n = len(b)
        if n < 15 or b[0] != 0xA5 or b[1] != 0x5A:
            self.bad += 1
            return

        typ = b[2]
        plen = struct.unpack_from("<H", b, 3)[0]

        if typ != 1:                      # 参数表/ACK/EVENT：本轮不记，只计数
            self.other += 1
            return

        cnt = struct.unpack_from("<H", b, 13)[0]
        if plen != 10 + cnt * 4 or cnt <= 0 or cnt > 64:
            self.bad += 1
            return

        end = 15 + cnt * 4 + 2 + 4
        if end != n or b[end - 4:end] != TAIL:
            self.bad += 1
            return

        crc_rx = struct.unpack_from("<H", b, 15 + cnt * 4)[0]
        if crc16_modbus(b[2:2 + 3 + plen]) != crc_rx:
            self.bad += 1
            return

        seq = struct.unpack_from("<H", b, 5)[0]
        ts = struct.unpack_from("<I", b, 7)[0]
        vals = struct.unpack_from("<%df" % cnt, b, 15)

        # 本轮第一次：定表头（车端档案切换会让 CNT 变，中途变了就丢弃）
        if self.cnt_expected is None:
            self.cnt_expected = cnt
            names = NAMES41 if cnt == 41 else (NAMES18 if cnt == 18 else None)
            units = UNITS41 if cnt == 41 else (UNITS18 if cnt == 18 else None)
            if names is None:
                names = ["ch%d" % k for k in range(cnt)]
                units = ["-"] * cnt
                print("⚠ 未知档案 CNT=%d，按 ch0..chN 记" % cnt)
            self.f.write("t_ms,seq," + ",".join(
                "%s[%s]" % (a, u) for a, u in zip(names, units)) + "\n")
            print("档案：%d 路" % cnt)

        if cnt != self.cnt_expected:
            self.bad += 1
            return

        if self.src is None:
            self.src = addr
            print("首包来自 %s:%d，%d 字节" % (addr[0], addr[1], n))

        # 丢帧统计（seq 是 16 位循环）
        if self.seq_prev is not None:
            d = (seq - self.seq_prev) & 0xFFFF
            if d > 1:
                self.lost += d - 1
        self.seq_prev = seq

        if self.ts_first is None:
            self.ts_first = ts
        self.ts_last = ts

        self.f.write("%d,%d," % (ts, seq) + ",".join("%.6g" % v for v in vals) + "\n")
        self.frames += 1

    # ------------------------------------------------------------ 快照
    def snapshot(self):
        with self.lock:
            secs = (self.ts_last - self.ts_first) / 1000.0 if self.ts_first else 0.0
            wall = (time.time() - self.t_start) if self.t_start else 0.0
            rate = (self.frames / secs) if secs > 0 else 0.0
            return dict(frames=self.frames, secs=secs, wall=wall, rate=rate,
                        lost=self.lost, bad=self.bad, other=self.other,
                        path=self.path)

    def close(self):
        if self.active:
            self.stop()
        self.stop_flag.set()
        try:
            self.sock.close()
        except OSError:
            pass


def _progress(cap):
    """采集中每秒刷一行（用 \\r 原地覆盖，不刷屏）。"""
    while cap.active:
        s = cap.snapshot()
        sys.stdout.write("\r  %4.1fs  %5d 帧  %5.1f 帧/s  丢 %d  坏 %d   按【回车】停止  " %
                         (s["secs"], s["frames"], s["rate"], s["lost"], s["bad"]))
        sys.stdout.flush()
        time.sleep(0.5)


def _stop_and_report(cap):
    p = cap.stop()
    if p is None:
        print("（当前没在采集）")
        return
    s = cap.snapshot()
    print("\r" + " " * 60 + "\r", end="")
    size = os.path.getsize(p) if os.path.exists(p) else 0
    print("已保存 %s" % p)
    print("  帧 %d  时长 %.2f s  %.1f 帧/s  丢 %d  坏 %d  其它帧 %d  文件 %d 字节" %
          (s["frames"], s["secs"], s["rate"], s["lost"], s["bad"], s["other"], size))
    if s["frames"] == 0:
        print("  ⚠ 0 帧：查车端 Host Link 是否 ON、是否停在 Host/Route 页、IP 对不对")


def main():
    port = 8086
    args = sys.argv[1:]
    i = 0
    while i < len(args):
        if args[i] == "--port" and i + 1 < len(args):
            port = int(args[i + 1]); i += 2
        else:
            i += 1

    try:
        cap = Cap(port)
    except OSError as e:
        print("绑定 UDP %d 失败：%s（上一次没退干净？划掉 a-Shell 重开）" % (port, e))
        return 2

    print("监听 UDP 0.0.0.0:%d" % port)


    try:
        while True:
            if not cap.active:
                sys.stdout.write("回车 = 开始采集     q = 退出: ")
                sys.stdout.flush()
            line = sys.stdin.readline()
            if not line:                      # Ctrl-D
                break
            line = line.strip().lower()

            if line == "q":
                if cap.active:
                    _stop_and_report(cap)
                break

            if cap.active:
                _stop_and_report(cap)
            else:
                p = cap.start()
                if p:
                    print("开始采集 -> %s" % p)
                    threading.Thread(target=_progress, args=(cap,), daemon=True).start()
    except KeyboardInterrupt:
        print()
        _stop_and_report(cap)

    cap.close()
    print("退出。已存的文件：")
    for f in sorted(os.listdir(".")):
        if f.startswith("wave_") and f.endswith(".csv"):
            print("  %-32s %9d 字节" % (f, os.path.getsize(f)))
    print("取回电脑：python3 -m http.server 8000  然后在电脑浏览器开 http://<手机IP>:8000/")
    return 0


if __name__ == "__main__":
    sys.exit(main())

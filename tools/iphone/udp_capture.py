#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Karting 上行遥测接收器（iPhone / iSH 版）

用途：车端连上 iPhone 个人热点后，把 41 路遥测收到手机里并存成 CSV。
      不依赖任何 App 的解析功能，只用 Python 标准库。

用法（在 iSH 里）：
    python3 udp_capture.py                 # 存到 ./wave_<时间>.csv
    python3 udp_capture.py --raw           # 额外把原始帧存一份 .bin（排查用）
    python3 udp_capture.py --port 8086     # 端口（要和车端 HOST_TARGET_PORT 一致）
    Ctrl-C 结束，会打印统计

前提（缺一不可）：
  1) 车端 HOST_TARGET_IP = "172.20.10.1"（iOS 热点的固定网关）、HOST_SOCK_TYPE = "UDP"
  2) iPhone 个人热点已开，"最大兼容性"打开（2.4 GHz，逐飞 WiFi 模块才扫得到）
  3) iOS 设置 → iSH → 本地网络 = 开
  4) iSH 保持前台、自动锁屏设"永不"

帧格式（车端 Host_Crtl.c: host_send_telemetry）：
    A5 5A | TYPE=01 | LEN(u16) | SEQ(u16) | TS(u32) | MASK(u16) | CNT(u16) | DATA CNT×f32 | CRC16(u16) | 00 00 80 7F
    LEN = 10 + CNT*4      CRC16-MODBUS 覆盖 [2, 2+3+LEN)
"""

import socket
import struct
import sys
import time

PORT = 8086

# ---- 41 路通道（顺序 = 车端 HOST_CH_*，与 KartHostLite 的 Channels.cs 完全一致）----
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

# ---- 标定档 18 路（tel_sel = 1 时车端只发这些，顺序 = HOST_CT_*）----
NAMES18 = [
    "target_l", "v_l", "u_l", "target_r", "v_r", "u_r",
    "steer_tgt", "steer_deg", "steer_u", "online", "en_bits", "safety",
    "steer_en", "steer_st", "ol_en", "resv0", "k_v", "k_z",
]
UNITS18 = ["rps", "rps", "%", "rps", "rps", "%", "deg", "deg", "duty",
           "-", "-", "-", "-", "-", "-", "-", "-", "-"]

TAIL = b"\x00\x00\x80\x7f"
HDR = 15          # magic2 + type1 + len2 + seq2 + ts4 + mask2 + cnt2


def crc16_modbus(buf: bytes) -> int:
    c = 0xFFFF
    for x in buf:
        c ^= x
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if (c & 1) else (c >> 1)
    return c & 0xFFFF


def parse(buf: bytes):
    """返回 (seq, ts, cnt, floats) 或 None"""
    if len(buf) < HDR + 6 or buf[0] != 0xA5 or buf[1] != 0x5A or buf[2] != 0x01:
        return None

    plen = struct.unpack_from("<H", buf, 3)[0]
    cnt = struct.unpack_from("<H", buf, 13)[0]
    if plen != 10 + cnt * 4:
        return None
    if len(buf) < HDR + cnt * 4 + 2 + 4:
        return None
    if buf[HDR + cnt * 4 + 2: HDR + cnt * 4 + 6] != TAIL:
        return None

    crc_rx = struct.unpack_from("<H", buf, HDR + cnt * 4)[0]
    if crc16_modbus(buf[2:2 + 3 + plen]) != crc_rx:
        return None

    seq = struct.unpack_from("<H", buf, 5)[0]
    ts = struct.unpack_from("<I", buf, 7)[0]
    vals = list(struct.unpack_from("<%df" % cnt, buf, HDR))
    return seq, ts, cnt, vals


def main():
    port = PORT
    want_raw = False
    args = sys.argv[1:]
    for i, a in enumerate(args):
        if a == "--port" and i + 1 < len(args):
            port = int(args[i + 1])
        elif a == "--raw":
            want_raw = True

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(1.0)

    stamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = "wave_%s.csv" % stamp
    raw_path = "wave_%s.bin" % stamp

    print("监听 UDP 0.0.0.0:%d" % port)
    print("写入 %s   （Ctrl-C 结束）" % csv_path)

    f = open(csv_path, "w", encoding="utf-8-sig", newline="")
    rawf = open(raw_path, "wb") if want_raw else None

    header_done = False
    n_ok = n_bad = 0
    t0 = time.time()
    last_tick = None
    dt_min, dt_max = 1e9, 0.0
    src = None

    try:
        while True:
            try:
                buf, addr = sock.recvfrom(4096)
            except socket.timeout:
                if n_ok == 0:
                    print("… 还没收到包（查：热点/最大兼容性/本地网络权限/车端 IP）")
                continue

            if src is None:
                src = addr
                print("首个包来自 %s:%d，%d 字节" % (addr[0], addr[1], len(buf)))

            if rawf is not None:
                rawf.write(struct.pack("<H", len(buf)))
                rawf.write(buf)

            r = parse(buf)
            if r is None:
                n_bad += 1
                continue

            seq, ts, cnt, vals = r
            n_ok += 1

            if not header_done:
                names = NAMES41 if cnt == 41 else (NAMES18 if cnt == 18 else None)
                units = UNITS41 if cnt == 41 else (UNITS18 if cnt == 18 else None)
                if names is None:
                    names = ["ch%d" % i for i in range(cnt)]
                    units = ["-"] * cnt
                    print("⚠ 未知档案 CNT=%d，按 ch0..chN 记" % cnt)
                f.write("t_ms,seq," + ",".join(
                    "%s[%s]" % (n, u) for n, u in zip(names, units)) + "\n")
                header_done = True
                print("档案：%d 路" % cnt)

            # t_ms 用**车端时钟**（TS = Sys_Tick_Ms），不是手机时钟 —— 重放对齐时更可靠
            f.write("%d,%d," % (ts, seq) + ",".join("%.6g" % v for v in vals) + "\n")

            if last_tick is not None:
                d = ts - last_tick
                if 0 < d < 500:
                    dt_min = min(dt_min, d)
                    dt_max = max(dt_max, d)
            last_tick = ts

            if n_ok % 50 == 1:
                f.flush()
                i = (NAMES41.index("v_l") if cnt == 41 else 1)
                s = (NAMES41.index("steer_deg") if cnt == 41 else 7)
                print("%5.1fs  %5d 帧  v=%.2f rps  steer=%.1f deg" %
                      (time.time() - t0, n_ok, vals[i], vals[s]))
    except KeyboardInterrupt:
        pass
    finally:
        f.close()
        if rawf is not None:
            rawf.close()
        el = max(1e-6, time.time() - t0)
        print("\n结束：%d 帧有效 / %d 帧丢弃，%.1f s，平均 %.1f 帧/s" %
              (n_ok, n_bad, el, n_ok / el))
        if n_ok > 1 and dt_max > 0:
            print("车端时间戳间隔：%d ms ~ %d ms（标称 20 ms）" % (dt_min, dt_max))
        print("CSV：%s" % csv_path)
        if want_raw:
            print("原始帧：%s" % raw_path)
        print("导出：在 iSH 里 `ls`，长按文件 → 分享；或 Files App → iSH 文件夹")


if __name__ == "__main__":
    main()

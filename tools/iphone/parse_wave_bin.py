#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把 iPhone 上 `nc -u -l` 抓下来的原始 UDP 流解成 CSV（在**电脑**上跑）。

为什么需要它：iOS 的 iSH 里 `apk add python3` 可能因 DNS 失败装不上，但 busybox 的 `nc`
是自带的。于是可以**不装任何东西**先抓包：

    # iPhone / iSH 里
    nc -u -l -p 8086 > wave.bin          # Ctrl-C 结束

再把 wave.bin 传回电脑，跑：

    python parse_wave_bin.py wave.bin            # -> wave.csv
    python parse_wave_bin.py wave.bin out.csv

注意：`nc` 把 UDP 数据报**首尾相连**写进文件，不保留边界。所以本脚本按帧头 A5 5A 重同步，
      用帧里的 LEN + CRC16 + 帧尾校验；断帧/丢包会跳过并计入统计。

也兼容 `udp_capture.py --raw` 的输出（那种格式每帧前面多了 2 字节长度前缀）——
      两种格式都靠 A5 5A 重同步，自动识别。
"""

import struct
import sys

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
NAMES18 = [
    "target_l", "v_l", "u_l", "target_r", "v_r", "u_r",
    "steer_tgt", "steer_deg", "steer_u", "online", "en_bits", "safety",
    "steer_en", "steer_st", "ol_en", "resv0", "k_v", "k_z",
]
UNITS18 = ["rps", "rps", "%", "rps", "rps", "%", "deg", "deg", "duty",
           "-", "-", "-", "-", "-", "-", "-", "-", "-"]

HDR = 15
TAIL = b"\x00\x00\x80\x7f"


def crc16_modbus(buf):
    c = 0xFFFF
    for x in buf:
        c ^= x
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if (c & 1) else (c >> 1)
    return c & 0xFFFF


# 上行帧类型（Host_Crtl.c / Protocol.cs 同一套）
TYPE_NAME = {1: "telemetry", 2: "param-meta", 3: "param-value",
             4: "ack", 5: "nak", 6: "event"}


def frames(buf):
    """切出所有合法帧 -> (type, seq, ts, cnt, vals)。
    ★ 只 yield type=1（遥测）给 CSV 用；其它类型计数到 frames.stat 里。
    2026-10-24：以前只认 type=1，参数表 META 帧会被当成"垃圾字节"报成几万字节，
    现在分类型统计 —— 实测一次手机抓包里 327 帧 param-meta 就曾被误报为 27 KB 跳过。"""
    i, n = 0, len(buf)
    skipped = 0
    bad = 0
    kinds = {}
    while i + HDR + 6 <= n:
        t = buf[i + 2] if (buf[i] == 0xA5 and buf[i + 1] == 0x5A) else 0
        if t not in TYPE_NAME:
            i += 1
            skipped += 1
            continue

        plen = struct.unpack_from("<H", buf, i + 3)[0]
        if t != 1:
            end = i + 11 + plen
            if end <= n and buf[end - 4:end] == TAIL:
                kinds[t] = kinds.get(t, 0) + 1
                i = end
                continue
            i += 1
            skipped += 1
            bad += 1
            continue

        cnt = struct.unpack_from("<H", buf, i + 13)[0]
        if plen != 10 + cnt * 4 or cnt <= 0 or cnt > 64:
            i += 1
            skipped += 1
            bad += 1
            continue

        end = i + HDR + cnt * 4 + 2 + 4
        if end > n:
            break                       # 尾部截断

        if buf[end - 4:end] != TAIL:
            i += 1
            skipped += 1
            bad += 1
            continue

        crc_rx = struct.unpack_from("<H", buf, i + HDR + cnt * 4)[0]
        if crc16_modbus(buf[i + 2:i + 2 + 3 + plen]) != crc_rx:
            i += 1
            skipped += 1
            bad += 1
            continue

        seq = struct.unpack_from("<H", buf, i + 5)[0]
        ts = struct.unpack_from("<I", buf, i + 7)[0]
        vals = struct.unpack_from("<%df" % cnt, buf, i + HDR)
        kinds[1] = kinds.get(1, 0) + 1
        yield seq, ts, cnt, vals
        i = end

    frames.stat = (skipped, bad, kinds)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1

    src = sys.argv[1]
    dst = sys.argv[2] if len(sys.argv) > 2 else (src.rsplit(".", 1)[0] + ".csv")

    with open(src, "rb") as f:
        buf = f.read()

    print("读入 %s（%d 字节）" % (src, len(buf)))

    out = open(dst, "w", encoding="utf-8-sig", newline="")
    n = 0
    last_ts = None
    dt_min, dt_max = 10 ** 9, 0
    cnt0 = None

    for seq, ts, cnt, vals in frames(buf):
        if cnt0 is None:
            cnt0 = cnt
            names = NAMES41 if cnt == 41 else (NAMES18 if cnt == 18 else None)
            units = UNITS41 if cnt == 41 else (UNITS18 if cnt == 18 else None)
            if names is None:
                names = ["ch%d" % k for k in range(cnt)]
                units = ["-"] * cnt
                print("⚠ 未知档案 CNT=%d，按 ch0..chN 记" % cnt)
            out.write("t_ms,seq," + ",".join("%s[%s]" % (a, b) for a, b in zip(names, units)) + "\n")

        out.write("%d,%d," % (ts, seq) + ",".join("%.6g" % v for v in vals) + "\n")
        n += 1

        if last_ts is not None:
            d = ts - last_ts
            if 0 < d < 500:
                dt_min = min(dt_min, d)
                dt_max = max(dt_max, d)
        last_ts = ts

    out.close()

    skipped, bad, kinds = getattr(frames, "stat", (0, 0, {}))
    print("遥测帧 %d（CNT=%s）  跳过/重同步 %d 字节，坏帧 %d" % (n, cnt0, skipped, bad))
    if kinds:
        print("各类帧: " + ", ".join("%s %d" % (TYPE_NAME.get(k, str(k)), v)
                                     for k, v in sorted(kinds.items())))
    if n > 1:
        print("车端时间戳间隔 %d ~ %d ms（标称 20）" % (dt_min, dt_max))
    print("输出 -> %s" % dst)
    return 0 if n > 0 else 2


if __name__ == "__main__":
    sys.exit(main())

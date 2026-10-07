#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
远程协议测试向量生成器（独立于 C# / C++ 实现的第三份参考实现）。

用途：
  1. 以 struct 按 protocol/PROTOCOL.md 的字节布局生成固定样本帧，写入
     protocol/test_vectors/*.hex。
  2. C# 单元自检（MasterConsole.Protocol.Tests）读取这些 hex 并与自身编码结果逐字节比对；
     C++ 侧后续同样可以读取比对。
  3. 同时校验各帧长度常量，防止出现文档风险 R-09 中“长度不一致”的问题。

运行：python tools/gen_protocol_vectors.py
"""
import hashlib
import hmac
import os
import struct

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "protocol", "test_vectors")

MAGIC = 0x4956          # 字节序列 56 49 = "VI"
VERSION = 1
T_CONTROL, T_HAPTIC, T_STATUS = 1, 2, 3
HEADER_LEN = 16
MAC_LEN = 8
CONTROL_PAYLOAD = 55
HAPTIC_PAYLOAD = 26
STATUS_PAYLOAD = 118

SESSION_KEY = bytes(range(16))   # 固定测试密钥


def header(ftype, session, seq, ts):
    return struct.pack("<HBBIII", MAGIC, VERSION, ftype, session, seq, ts)


def seal(body):
    mac = hmac.new(SESSION_KEY, body, hashlib.sha256).digest()[:MAC_LEN]
    return body + mac


def handle_sample(buttons, valid, enc, joints, vels):
    return struct.pack("<BB2i2f2f", buttons, valid, enc[0], enc[1],
                       joints[0], joints[1], vels[0], vels[1])


def control_frame():
    payload = handle_sample(0x05, 1, (1234, -5678), (0.5, -0.25), (1.5, -2.5))
    payload += handle_sample(0x02, 1, (-1, 2), (1.0, 2.0), (0.0, 0.125))
    payload += struct.pack("<2b", 1, -1)   # 注射器 1/2 方向
    payload += struct.pack("<b", -1)       # 轴4点动方向
    assert len(payload) == CONTROL_PAYLOAD
    return seal(header(T_CONTROL, 0xA1B2C3D4, 100, 123456) + payload)


def haptic_frame():
    payload = struct.pack("<IH", 123450, 3)
    payload += struct.pack("<Bbff", 1, 1, 0.5, 0.01)   # enable, axis, force_n, torque_nm
    payload += struct.pack("<Bbff", 0, 0, 0.0, 0.0)
    assert len(payload) == HAPTIC_PAYLOAD
    return seal(header(T_HAPTIC, 0xA1B2C3D4, 7, 200000) + payload)


def status_frame():
    flags = (1 << 0) | (1 << 3) | (1 << 4) | (1 << 7) | (1 << 9)
    payload = struct.pack("<IHI", 123450, 3, flags)
    payload += struct.pack("<4i", 2, 1, 4, 3)
    payload += struct.pack("<4H", 2000, 10, 2000, 10)
    payload += struct.pack("<B", 0x05)
    payload += struct.pack("<2b", 0, 1)    # injector_active
    payload += struct.pack("<b", 1)        # axis4_active
    payload += struct.pack("<5f", 0.25, 0.01, 0.5, 0.02, 0.3)
    payload += struct.pack("<f", 99.5)
    payload += struct.pack("<7f", 1, 2, 3, 4, 5, 6, 7)
    payload += struct.pack("<7f", 11, 12, 13, 14, 15, 16, 17)
    assert len(payload) == STATUS_PAYLOAD
    return seal(header(T_STATUS, 0xA1B2C3D4, 9, 300000) + payload)


def auth_vectors():
    token = b"vessel-robot-demo-token"
    nc = bytes(range(0, 16))
    ns = bytes(range(16, 32))
    auth = hmac.new(token, b"auth" + nc + ns, hashlib.sha256).digest()
    key = hmac.new(token, b"key" + nc + ns, hashlib.sha256).digest()[:16]
    return token, nc, ns, auth, key


def write_hex(name, data):
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, name), "w", encoding="utf-8", newline="\n") as f:
        f.write(data.hex() + "\n")


def main():
    frames = {
        "control_frame.hex": (control_frame(), HEADER_LEN + CONTROL_PAYLOAD + MAC_LEN),
        "haptic_frame.hex": (haptic_frame(), HEADER_LEN + HAPTIC_PAYLOAD + MAC_LEN),
        "status_frame.hex": (status_frame(), HEADER_LEN + STATUS_PAYLOAD + MAC_LEN),
    }
    for name, (data, expect) in frames.items():
        assert len(data) == expect, (name, len(data), expect)
        write_hex(name, data)
        print(f"{name}: {len(data)} bytes")

    token, nc, ns, auth, key = auth_vectors()
    with open(os.path.join(OUT, "auth.txt"), "w", encoding="utf-8", newline="\n") as f:
        f.write(f"token_ascii={token.decode()}\n")
        f.write(f"nonce_client={nc.hex()}\n")
        f.write(f"nonce_server={ns.hex()}\n")
        f.write(f"auth_mac={auth.hex()}\n")
        f.write(f"session_key={key.hex()}\n")
    write_hex("session_key.hex", SESSION_KEY)
    print("auth.txt written")


if __name__ == "__main__":
    main()

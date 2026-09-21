#!/usr/bin/env python3
"""
【视觉端串口通信测试脚本 - 单次发送+监听版】
专为视觉端（YOLO）设计，使用 PDF 中「视觉与底盘」协议：
帧头 0xAA，帧尾 0x55
"""

import serial
import time
import struct
import sys

# ====================== Modbus CRC16 ======================
def modbus_crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc.to_bytes(2, byteorder='little')


# ====================== 帧构建（视觉端专用） ======================
def build_vision_frame(region: int, mode: int, need_return: int, data_value: int = 0) -> bytes:
    """视觉 → 底盘帧（0xAA ... 0x55）"""
    data_bytes = struct.pack("<h", data_value)
    reserve = b'\x00\x00'
    payload_for_crc = bytes([region, mode, need_return]) + data_bytes + reserve
    crc = modbus_crc16(payload_for_crc)
    
    frame = (
        b'\xAA' +                  # 视觉专用帧头
        payload_for_crc +
        crc +
        b'\x55'                    # 视觉专用帧尾
    )
    return frame


# ====================== 帧解析 ======================
def parse_frame(raw: bytes) -> dict | None:
    if len(raw) < 11:
        return None
    if raw[0] != 0xAA or raw[-1] != 0x55:
        return None

    region = raw[1]
    mode = raw[2]
    task_complete = raw[3]          # 任务完成标志位（最重要）
    data_value = struct.unpack("<h", raw[4:6])[0]
    crc_received = raw[6:8]

    payload = raw[1:6] + b'\x00\x00'
    crc_calc = modbus_crc16(payload)
    crc_ok = crc_calc == crc_received

    return {
        "header": "0xAA",
        "region": region,
        "mode": mode,
        "task_complete": task_complete,   # 1=底盘已完成任务
        "data": data_value,
        "crc_ok": crc_ok,
        "raw": raw.hex()
    }


# ====================== 预定义命令（视觉端常用） ======================
PREDEFINED_COMMANDS = {
    "0": {"name": "启动导航",      "region": 0x01, "mode": 0x00, "need_return": 0, "data": 0},
    "1": {"name": "抓取命令",      "region": 0x01, "mode": 0x01, "need_return": 0, "data": 0},
    "2": {"name": "TF对位成功",    "region": 0x01, "mode": 0x02, "need_return": 0, "data": 0},
    "3": {"name": "二维码确认",    "region": 0x01, "mode": 0x03, "need_return": 0, "data": 0},
    "4": {"name": "流程收尾/继续导航", "region": 0x01, "mode": 0x04, "need_return": 0, "data": 0},
    "5": {"name": "最终退出",      "region": 0x01, "mode": 0x05, "need_return": 0, "data": 0},
    "6": {"name": "抓取失败",      "region": 0x01, "mode": 0x06, "need_return": 0, "data": 0},
    "7": {"name": "特殊二维码200", "region": 0x01, "mode": 0x07, "need_return": 0, "data": 0},
    "8": {"name": "首点同步触发",  "region": 0x01, "mode": 0x08, "need_return": 0, "data": 0},
    "9": {"name": "模式9",         "region": 0x01, "mode": 0x09, "need_return": 0, "data": 0},
}


def main():
    print("=== 视觉端串口通信测试工具（0xAA / 0x55 协议） ===")
    print("串口：/dev/tty   波特率：115200")
    print("规则：输入命令 → 串口只发送一次 → 短时间监听回包\n")

    try:
        ser = serial.Serial(port='/dev/ttyserial', baudrate=115200, timeout=0.5, write_timeout=1.0)
        print("✅ 串口打开成功\n")
    except Exception as e:
        print(f"❌ 串口打开失败: {e}")
        sys.exit(1)

    try:
        while True:
            print("\n" + "="*60)
            print("操作菜单：")
            print("  [1] 发送预定义命令（0/1/2/3/4/5/6/7/8/9）")
            print("  [2] 手动输入帧参数")
            print("  [q] 退出")
            choice = input("请选择 (1/2/q): ").strip().lower()

            if choice == 'q':
                break

            if choice == '1':
                print("可用命令:", list(PREDEFINED_COMMANDS.keys()))
                cmd_key = input("输入命令编号 (如 1 或 5): ").strip()
                if cmd_key not in PREDEFINED_COMMANDS:
                    print("❌ 无效命令")
                    continue
                cfg = PREDEFINED_COMMANDS[cmd_key]
                name = cfg["name"]
                frame = build_vision_frame(cfg["region"], cfg["mode"], cfg["need_return"], cfg["data"])
            elif choice == '2':
                try:
                    region = int(input("区域划分 (默认1): ") or "1")
                    mode = int(input("模式 (默认1): ") or "1")
                    need = int(input("是否需要回传 (0/1，默认1): ") or "1")
                    data_val = int(input("数据值 (默认0): ") or "0")
                    frame = build_vision_frame(region, mode, need, data_val)
                    name = f"手动帧 (mode={mode})"
                except ValueError:
                    print("❌ 输入格式错误")
                    continue
            else:
                continue

            print(f"\n🚀 发送 {name} ...（只发送一次）")
            print("按 Ctrl+C 可强制停止本次测试\n")
            sent_count = 0

            try:
                ser.write(frame)
                sent_count = 1
                print(f"[{time.strftime('%H:%M:%S')}] 已发送 1 次: {frame.hex()}")

                wait_seconds = 8.0
                deadline = time.time() + wait_seconds
                received = False
                while time.time() < deadline:
                    if ser.in_waiting:
                        raw = ser.read(ser.in_waiting)
                        parsed = parse_frame(raw)
                        if parsed:
                            print("\n" + "="*60)
                            print("✅ 收到底盘有效回包！")
                            for k, v in parsed.items():
                                print(f"   {k:15} = {v}")
                            print("="*60)
                            received = True
                            break   # 收到回包后停止本次测试
                    time.sleep(0.05)

                if not received:
                    print(f"⚠️ {wait_seconds:.1f} 秒内未收到有效回包")
            except KeyboardInterrupt:
                print("\n⏹️ 本次测试手动停止")

            print(f"\n本次测试结束（共发送 {sent_count} 次），返回主菜单...\n")

    except KeyboardInterrupt:
        print("\n用户终止测试")
    finally:
        if ser.is_open:
            ser.close()
        print("串口已关闭")


if __name__ == "__main__":
    main()

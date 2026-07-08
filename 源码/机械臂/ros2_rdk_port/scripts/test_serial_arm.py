#!/usr/bin/env python3
"""Direct STM32 serial test (no ROS). Usage: python3 test_serial_arm.py [/dev/ttyACM0]"""
import sys
import time

import serial

FRAME_HEAD = 0xAA
FRAME_TAIL = 0xBB
INIT_ANGLES = [0.0, 0.0, 0.0, -1.57, 0.0, 0.0]
MOVE_ANGLES = [0.0, 0.5, -1.2, -1.2, 0.0, 0.2]


def xor_checksum(data):
    checksum = 0
    for byte in data:
        checksum ^= byte
    return checksum


def build_frame(angles, mode):
    frame = bytearray(16)
    frame[0] = FRAME_HEAD
    for index, angle in enumerate(angles):
        value = int(angle * 1000)
        frame[1 + index * 2] = (value >> 8) & 0xFF
        frame[2 + index * 2] = value & 0xFF
    frame[13] = mode
    frame[14] = xor_checksum(frame[:14])
    frame[15] = FRAME_TAIL
    return frame


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    print(f"Opening {port} @ 115200 (OLED must show ON)")
    ser = serial.Serial(port, 115200, timeout=1)
    ser.reset_input_buffer()
    ser.reset_output_buffer()

    init_frame = build_frame(INIT_ANGLES, mode=2)
    print(f"INIT  : {init_frame.hex(' ')}")
    ser.write(init_frame)
    ser.flush()
    time.sleep(1.0)

    move_frame = build_frame(MOVE_ANGLES, mode=1)
    print(f"MOVE  : {move_frame.hex(' ')}")
    for i in range(50):
        ser.write(move_frame)
        ser.flush()
        time.sleep(0.05)
    print("Done. Arm should lift if port and OLED are correct.")
    ser.close()


if __name__ == "__main__":
    main()

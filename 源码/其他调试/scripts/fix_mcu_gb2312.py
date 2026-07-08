#!/usr/bin/env python3
"""Fix garbled Chinese comments and re-save MCU sources as GB2312 for Keil."""
from __future__ import annotations

import glob
import os
import re

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SCAN_DIRS = ("HARDWARE", "USER", "SYSTEM")

# ST/CMSIS 大文件含特殊字符，保持原编码不转换
SKIP_FILES = {
    "stm32f10x.h",
    "system_stm32f10x.c",
    "system_stm32f10x.h",
    "stm32f10x_conf.h",
}

IMU_C_CONTENT = r'''#include "IMU.h"
#include "USART.h"
#include "Car.h"
#define pi 3.1416f
void IMU_Init(void)
{
    IMU_Struct.yaram_current=0;
    IMU_Struct.yaram_zero=0;
    IMU_Struct.yaram_Target=0;
}

void IMU_Get(void)
{
    if(USART_Data.USART2_RxFlag==1)
    {
        int16_t Temp = 0;
        Temp = (int16_t)(((uint16_t)USART_Data.USART2_RxPacket[6] << 8) |
             (uint16_t)USART_Data.USART2_RxPacket[5]);
        if(USART_Data.USART2_RxPacket[0] == 0x52)
        {
            /* 角速度 Wz：deg/s -> rad/s，取负与 Car.Car_yaw 正方向一致 */
            Car.current_w = -((float)Temp / 32768.0f) * IMU_GYRO_DPS_FS * (pi / 180.0f);
        }
        else if(USART_Data.USART2_RxPacket[0] == 0x53)
        {
            /* 偏航角 Yaw：度 */
            IMU_Struct.yaram_current = ((float)Temp / 32768.0f) * IMU_ANGLE_DEG_FS;
            IMU_Struct.yaram_current=IMU_Struct.yaram_current-IMU_Struct.yaram_zero;
            if(IMU_Struct.yaram_current > 180.0f) // 归一化到 -180~180 度
            {
                IMU_Struct.yaram_current -= 2 * 180.0f;
            }
            else if(IMU_Struct.yaram_current < -180.0f)
            {
                IMU_Struct.yaram_current += 2 * 180.0f;
            }
            Car.Car_yaw=IMU_Struct.yaram_current / 180.0f * pi ;
        }
//        USART1_SendNum((uint32_t)abs((int)IMU_Struct.yaram_zero),5);
//        USART1_SendBits(' ');
        USART_Data.USART2_RxFlag=0;
    }

}
'''


def read_best(path: str) -> str:
    raw = open(path, "rb").read()
    if not raw:
        return ""

    candidates: list[tuple[int, str]] = []
    for enc in ("utf-8-sig", "utf-8", "gbk", "gb2312", "latin-1"):
        try:
            text = raw.decode(enc)
        except UnicodeDecodeError:
            continue
        score = text.count("???") * 100
        score += text.count("\ufffd") * 50
        score += len(re.findall(r"[\u0080-\u00ff]{3,}", text)) * 5
        cn = sum(1 for c in text if "\u4e00" <= c <= "\u9fff")
        score -= cn * 2
        candidates.append((score, text))

    candidates.sort(key=lambda x: x[0])
    return candidates[0][1]


def patch_car_h(text: str) -> str:
    block = """/* ========== 编码器与轮速换算（电机轴 r/min）==========
 * 轮径 65 mm；电机轴编码器 CPR=13 线/圈
 * TIM 编码器模式 TI12 四倍频，每圈脉冲 = 13*4；CAR_ENCODER_QUAD 为四倍频系数
 * 减速比 30：轮子转 1 圈，电机轴转 30 圈
 */
#define CAR_WHEEL_DIAMETER_M          0.065f
#define CAR_WHEEL_RADIUS_M            (CAR_WHEEL_DIAMETER_M * 0.5f)
#define CAR_TRACK_WIDTH_M             0.1847f
#define CAR_GEAR_MOTOR_PER_WHEEL      30.0f

#define CAR_ENCODER_LINES             13
#define CAR_ENCODER_QUAD              4
#define CAR_ENCODER_CNT_PER_MOTOR_REV ((float)((CAR_ENCODER_LINES) * (CAR_ENCODER_QUAD)))

/* TIM6 采样周期（秒），须与 timer.c 中 TIM6 的 PSC/ARR 一致，当前 50 ms */
#define CAR_SPEED_TIM6_PERIOD_S       0.05f

/* 编码器 delta（每 TIM6 周期）-> 电机轴转速 (r/min) */
#define Car_EncoderDelta_To_MotorRpm(delta) \\
    ((float)(delta) * 60.0f / ((CAR_ENCODER_CNT_PER_MOTOR_REV) * (CAR_SPEED_TIM6_PERIOD_S)))

/* 四轮编码器符号：与 TIM 计数方向对齐；W1/W3 与 W2/W4 相反；v=0 时 PID 反馈符号须一致 */
#define CAR_ENCODER_SIGN_W1  (-1)
#define CAR_ENCODER_SIGN_W2  (1)
#define CAR_ENCODER_SIGN_W3  (-1)
#define CAR_ENCODER_SIGN_W4  (1)

/* 角速度 target_w -> 轮速差增益：1.0=理论值，当前 1.5 */
#define CAR_YAW_TO_WHEEL_GAIN  (1.5f)

float Car_WheelLinearV_To_MotorRpm(float v_m_s);

/* 电机轴 r/min（与 PID 同符号）-> 轮缘线速度 m/s */
float Car_MotorRpm_ToWheelLinearV_mps(float motor_rpm_signed);

typedef enum {
    /* 四差速合成：vx=四轮线速平均，vy=0 */
    CAR_KIN_NORMAL_4W = 0,
    /* 麦克纳姆 X 型：含 vy；W1..W4 = 左前/右前/左后/右后 */
    CAR_KIN_MECANUM_X = 1
} Car_KinematicsModel_t;

/* 本车四轮同速差速（非麦轮） */
#define CAR_KIN_DIFF_4W CAR_KIN_NORMAL_4W

/* 四轮电机 r/min -> 车体 vx、vy（m/s）；x 前向，y 左向 */
void Car_FourWheelMotorRpm_ToBodyXY_mps(float rpm1, float rpm2, float rpm3, float rpm4,
    float *vx_out, float *vy_out, Car_KinematicsModel_t model);"""

    m = re.search(
        r"#ifndef __Car_H.*?#include <stdint\.h>\s*\n",
        text,
        re.DOTALL,
    )
    if not m:
        raise RuntimeError("Car.h header not found")
    tail_m = re.search(r"typedef struct\s*\{", text)
    if not tail_m:
        raise RuntimeError("Car.h struct not found")
    tail = text[tail_m.start() :]
    return text[: m.end()] + block + "\n\n" + tail


def patch_imu_h(text: str) -> str:
    return """#ifndef __IMU_H
#define __IMU_H	 
#include "sys.h"

/* WT901/WT9011 系列：角速度满量程（单位：deg/s）；资料若为 2000deg/s 则改宏 */
#ifndef IMU_GYRO_DPS_FS
#define IMU_GYRO_DPS_FS 4000.0f
#endif

/* 角度输出满量程（单位：度），WT901 通常为 +/-180 度 */
#ifndef IMU_ANGLE_DEG_FS
#define IMU_ANGLE_DEG_FS 180.0f
#endif

typedef struct
{
    float yaram_current;
    float yaram_zero;
    float yaram_Target;
}IMU_Structure;

extern IMU_Structure IMU_Struct;
void IMU_Init(void);
void IMU_Get(void);
		 				    
#endif
"""


def patch_nrf(text: str) -> str:
    return text.replace('printf("???");', 'printf("未知");')


def patch_file(path: str, text: str) -> str:
    base = os.path.basename(path)
    if base == "Car.h":
        # Car.h is maintained by repair_car_h.py; only re-encode, do not patch content
        return text
    if base == "IMU.c":
        return IMU_C_CONTENT
    if base == "IMU.h":
        return patch_imu_h(text)
    if base == "nrf24l01.c":
        return patch_nrf(text)
    return text


def to_crlf_gb2312(text: str) -> bytes:
    text = text.replace("\r\n", "\n").replace("\r", "\n").replace("\n", "\r\n")
    return text.encode("gb2312", errors="strict")


def main() -> None:
    paths: list[str] = []
    for d in SCAN_DIRS:
        paths.extend(glob.glob(os.path.join(ROOT, d, "**", "*.c"), recursive=True))
        paths.extend(glob.glob(os.path.join(ROOT, d, "**", "*.h"), recursive=True))
    paths.sort()

    changed = 0
    skipped = 0
    for path in paths:
        base = os.path.basename(path)
        if base in SKIP_FILES:
            skipped += 1
            continue
        rel = os.path.relpath(path, ROOT)
        text = read_best(path)
        new_text = patch_file(path, text)
        encoded = to_crlf_gb2312(new_text)
        old = open(path, "rb").read()
        if encoded != old:
            open(path, "wb").write(encoded)
            changed += 1
            print(f"updated: {rel}")
    print(f"done, {changed} updated, {skipped} skipped, {len(paths)} total")


if __name__ == "__main__":
    main()

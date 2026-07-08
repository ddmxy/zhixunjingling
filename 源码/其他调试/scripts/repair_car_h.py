#!/usr/bin/env python3
"""Repair Car.h: fix broken macro continuation and GB2312 encoding."""
from pathlib import Path

CONTENT = r"""#ifndef __Car_H
#define __Car_H
#include "sys.h"
#include "USART.h"
#include <stdint.h>

/* ========== 编码器与轮速换算（电机轴 r/min）==========
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
#define Car_EncoderDelta_To_MotorRpm(delta) \
    ((float)(delta) * 60.0f / ((CAR_ENCODER_CNT_PER_MOTOR_REV) * (CAR_SPEED_TIM6_PERIOD_S)))

/* 四轮编码器符号：与 TIM 计数方向对齐；W1/W3 与 W2/W4 相反 */
#define CAR_ENCODER_SIGN_W1  (-1)
#define CAR_ENCODER_SIGN_W2  (1)
#define CAR_ENCODER_SIGN_W3  (-1)
#define CAR_ENCODER_SIGN_W4  (1)

/* 角速度 target_w -> 轮速差增益：1.0=理论值，当前 1.5 */
#define CAR_YAW_TO_WHEEL_GAIN       (1.45f)
/* |w|<BOOST_MAX 时额外放大轮速差，克服静摩擦（大角速度不放大） */
#define CAR_YAW_LOW_W_BOOST_MAX     (0.45f)
#define CAR_YAW_LOW_W_BOOST_GAIN    (1.28f)

float Car_WheelLinearV_To_MotorRpm(float v_m_s);
float Car_MotorRpm_ToWheelLinearV_mps(float motor_rpm_signed);
float Car_YawToWheelApply(float target_w);

typedef enum {
    CAR_KIN_NORMAL_4W = 0,
    CAR_KIN_MECANUM_X = 1
} Car_KinematicsModel_t;

#define CAR_KIN_DIFF_4W CAR_KIN_NORMAL_4W

void Car_FourWheelMotorRpm_ToBodyXY_mps(float rpm1, float rpm2, float rpm3, float rpm4,
    float *vx_out, float *vy_out, Car_KinematicsModel_t model);

typedef struct
{
    float current_vx;
    float current_vy;
    float current_w;
    float target_w;
    float target_v;
    float Car_yaw;
    float Car_mindistance;
    uint8_t Send_Yaw_Flag;
    uint8_t Send_Mindistance_Flag;
    uint8_t Update_Speed_Flag;
    uint8_t Time_Flag;
    uint8_t start_Flag;
    int Wheel_1_dirc;
    int Wheel_2_dirc;
    int Wheel_3_dirc;
    int Wheel_4_dirc;
}Car_Struct;

extern Car_Struct Car;

void Car_FloatToBytes(float f, uint8_t *out4);
float Car_FloatFromBytesLast4(const uint8_t *buf, uint32_t len);
void Car_Init(void);
void Car_Encoder_Input(void);
void Car_Send_Current_V(void);
void Car_Send_Yaw(void);
void Car_Remote(void);
#endif
"""

def main() -> None:
    path = Path(__file__).resolve().parent.parent / "HARDWARE" / "Car.h"
    text = CONTENT.strip() + "\r\n"
    path.write_bytes(text.encode("gb2312"))
    print(f"wrote {path} ({path.stat().st_size} bytes)")


if __name__ == "__main__":
    main()

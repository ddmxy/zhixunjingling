#include "Car.h"
#include <string.h>
#include "PID.h"
#include "hu_m40.h"
#include <stdlib.h>
float Car_WheelLinearV_To_MotorRpm(float v_m_s)
{
    const float two_pi_r = 6.2831853f * CAR_WHEEL_RADIUS_M;
    return (v_m_s * 60.0f * CAR_GEAR_MOTOR_PER_WHEEL) / two_pi_r;
}

float Car_MotorRpm_ToWheelLinearV_mps(float motor_rpm_signed)
{
    const float two_pi_r = 6.2831853f * CAR_WHEEL_RADIUS_M;
    return motor_rpm_signed * two_pi_r / (60.0f * CAR_GEAR_MOTOR_PER_WHEEL);
}

float Car_YawToWheelApply(float target_w)
{
    float w_apply;
    float aw;

    w_apply = -target_w;
    aw = w_apply >= 0.0f ? w_apply : -w_apply;
    if (aw > 0.02f && aw < CAR_YAW_LOW_W_BOOST_MAX) {
        w_apply *= CAR_YAW_LOW_W_BOOST_GAIN;
    }
    return w_apply;
}

void Car_FourWheelMotorRpm_ToBodyXY_mps(float rpm1, float rpm2, float rpm3, float rpm4,
    float *vx_out, float *vy_out, Car_KinematicsModel_t model)
{
    float v1, v2, v3, v4;

    if (vx_out == 0 || vy_out == 0)
        return;

    v1 = Car_MotorRpm_ToWheelLinearV_mps(rpm1);
    v2 = Car_MotorRpm_ToWheelLinearV_mps(rpm2);
    v3 = Car_MotorRpm_ToWheelLinearV_mps(rpm3);
    v4 = Car_MotorRpm_ToWheelLinearV_mps(rpm4);

    if (model == CAR_KIN_MECANUM_X) {
        /* Standard X-mecanum; flip signs if rollers/motors wired differently */
        *vx_out = 0.25f * (v1 + v2 + v3 + v4);
        *vy_out = 0.25f * (-v1 + v2 - v3 + v4);
    } else {
        *vx_out = 0.25f * (v1 + v2 + v3 + v4);
        *vy_out = 0.0f;
    }
}

void Car_FloatToBytes(float f, uint8_t *out4)
{
    memcpy(out4, &f, 4);
}

float Car_FloatFromBytesLast4(const uint8_t *buf, uint32_t len)
{
    float f;

    if (buf == 0 || len < 4)
        return 0.0f;
    memcpy(&f, buf + (len - 4), 4);
    return f;
}

void Car_Init(void)
{
    Car.Car_mindistance=0;
    Car.Car_yaw=0;
    Car.current_vx=0;
    Car.current_vy=0;
    Car.current_w=0;
    Car.Send_Mindistance_Flag=0;
    Car.Send_Yaw_Flag=0;
    Car.target_v=0;
    Car.target_w=0;
    Car.Update_Speed_Flag=0;
    Car.Time_Flag=0;
    Car.start_Flag=0;
}

void Car_Send_Current_V(void)
{
    Car_FourWheelMotorRpm_ToBodyXY_mps(
        PID_V[0].Present_Value, PID_V[1].Present_Value,
        PID_V[2].Present_Value, PID_V[3].Present_Value,
        &Car.current_vx, &Car.current_vy, CAR_KIN_DIFF_4W);
    USART1_SendBits(0xff);
    USART1_SendBits(0x03);
    Car_FloatToBytes(Car.current_vx, USART_Data.USART1_TxPacket);
    Car_FloatToBytes(Car.current_vy, USART_Data.USART1_TxPacket + 4);
    USART1_SendArray(USART_Data.USART1_TxPacket, 8);
    USART1_SendBits(0xfe);
}

void Car_Send_Yaw(void)
{
    USART1_SendBits(0xff);
    USART1_SendBits(0x04);
    Car_FloatToBytes(Car.Car_yaw, USART_Data.USART1_TxPacket);
    Car_FloatToBytes(Car.current_w, USART_Data.USART1_TxPacket + 4);
    USART1_SendArray(USART_Data.USART1_TxPacket, 8);
    USART1_SendBits(0xfe);    
}

void Car_Encoder_Input(void)
{
    if(USART_Data.USART1_RxFlag == 1)
    {
        Car.start_Flag=1;
        Car.target_v = Car_FloatFromBytesLast4(&USART_Data.USART1_RxPacket[0], 4);
        Car.target_w = Car_FloatFromBytesLast4(&USART_Data.USART1_RxPacket[4], 4);
        {
            float half_track = CAR_TRACK_WIDTH_M * 0.5f * CAR_YAW_TO_WHEEL_GAIN;
            float w_apply = Car_YawToWheelApply(Car.target_w);
            float v0 = Car.target_v + w_apply * half_track;
            float v1 = Car.target_v - w_apply * half_track;
            float v2 = Car.target_v + w_apply * half_track;
            float v3 = Car.target_v - w_apply * half_track;

            PID_16_Target_Apply(&PID_V[0], Car_WheelLinearV_To_MotorRpm(v0));
            PID_16_Target_Apply(&PID_V[1], Car_WheelLinearV_To_MotorRpm(v1));
            PID_16_Target_Apply(&PID_V[2], Car_WheelLinearV_To_MotorRpm(v2));
            PID_16_Target_Apply(&PID_V[3], Car_WheelLinearV_To_MotorRpm(v3));
        }

        USART_Data.USART1_RxFlag = 0;
    }
}

void Car_Remote(void)
{
    if (hu_m40_read() == 1) 
    {
        uint8_t ly = hu_m40_analog(HU_LY);
        uint8_t lx = hu_m40_analog(HU_LX);
        uint8_t ry = hu_m40_analog(HU_RY);
        uint8_t rx = hu_m40_analog(HU_RX);
        static uint32_t btn_prev_stable = 0;   
        static uint32_t btn_last_sample = 0;   
        static uint8_t  same_cnt = 0;          

        uint32_t btn_sample = 0;
        
        if(ly>128&&abs((int)(lx-128))<50)
        {
            Car.Wheel_1_dirc=-1;
            Car.Wheel_2_dirc=-1;
            Car.Wheel_3_dirc=-1;
            Car.Wheel_4_dirc=-1;                    
        }
        else if(ly<128&&abs((int)(lx-128))<50)
        {
            Car.Wheel_1_dirc=1;
            Car.Wheel_2_dirc=1;
            Car.Wheel_3_dirc=1;
            Car.Wheel_4_dirc=1;                        
        }
        else if(ly==128&&abs((int)(lx-128))<50)
        {
            Car.Wheel_1_dirc=0;
            Car.Wheel_2_dirc=0;
            Car.Wheel_3_dirc=0;
            Car.Wheel_4_dirc=0;                     
        }
        else if(lx<128&&abs((int)(ly-128))<50)
        {
            Car.Wheel_1_dirc=-1;
            Car.Wheel_2_dirc=1;
            Car.Wheel_3_dirc=-1;
            Car.Wheel_4_dirc=1;                        
        }
        else if(lx>128&&abs((int)(ly-128))<50)
        {
            Car.Wheel_1_dirc=1;
            Car.Wheel_2_dirc=-1;
            Car.Wheel_3_dirc=1;
            Car.Wheel_4_dirc=-1;                       
        }
        else if(lx==128&&abs((int)(ly-128))<50) 
        {
            Car.Wheel_1_dirc=0;
            Car.Wheel_2_dirc=0;
            Car.Wheel_3_dirc=0;
            Car.Wheel_4_dirc=0;                      
        }
        
        if(ry<=128)
        {
            PID_V[0].Target_Value=Car.Wheel_1_dirc*4000.0f*(128-ry)/128.0f;
            PID_V[1].Target_Value=Car.Wheel_2_dirc*4000.0f*(128-ry)/128.0f;
            PID_V[2].Target_Value=Car.Wheel_3_dirc*4000.0f*(128-ry)/128.0f;
            PID_V[3].Target_Value=Car.Wheel_4_dirc*4000.0f*(128-ry)/128.0f;                  
        }
        
        if (hu_m40_button(HU_K1)) btn_sample |= HU_K1;
        if (hu_m40_button(HU_K2)) btn_sample |= HU_K2;
        if (hu_m40_button(HU_K3)) btn_sample |= HU_K3;
        if (hu_m40_button(HU_K4)) btn_sample |= HU_K4;
        if (hu_m40_button(HU_K5)) btn_sample |= HU_K5;
        if (hu_m40_button(HU_K6)) btn_sample |= HU_K6;

        if (btn_sample == btn_last_sample) {
            if (same_cnt < 3) same_cnt++;
        } else {
            same_cnt = 0;
            btn_last_sample = btn_sample;
        }

        if (same_cnt >= 2)
        {
            uint32_t btn_stable = btn_sample;
            uint32_t pressed_edge = btn_stable & (~btn_prev_stable); 
            btn_prev_stable = btn_stable;

            if (pressed_edge & HU_K1)  
            {
                PID_V[0].Target_Value=0;
                PID_V[1].Target_Value=0;
                PID_V[2].Target_Value=0;
                PID_V[3].Target_Value=0;
            }
            if (pressed_edge & HU_K2) 
            {
               
            }
            if (pressed_edge & HU_K3) 
            {
                
            }
            if (pressed_edge & HU_K4)  
            {

            }
        }
    }
}

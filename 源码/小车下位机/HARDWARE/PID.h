#ifndef __PID_H
#define __PID_H
#include "stm32f10x.h"    
/* 速度环输出死区（PWM）；PID 内部速度量纲为电机轴 r/min 时需与此无关 */
#define PID_DEAD_VELOCITY  0
/* 角度环输出死区（PWM 或等价量），略大可减抖动 */
#define PID_DEAD_ANGLE     18
typedef struct{
    float Target_Value;
    float Present_Value;
    float Kp;
    float Ki;
    float Kd;
    float Error;           /* 当前误差 */
    float Last_Error;      /* 上一拍误差 */
    float Prev_Error;      /* 上上拍误差 */
	float P_Output;		   /* 比例项输出 */
	float I_Output;        /* 积分项输出 */
	float D_Output;		   /* 微分项输出 */
    int16_t Output;        /* 最终输出（如 PWM） */
	float I_Limit;         /* 积分限幅 */
    float I_Threshold;     /* 积分分离门限 */
    float Output_Limit;    /* 输出限幅 */
}PID_16_TypeDef;


extern PID_16_TypeDef PID_V[4];
extern PID_16_TypeDef PID_A[4];
extern PID_16_TypeDef PID_P[4];

void PID_16_Param_Set(PID_16_TypeDef* pPID, float Kp, float Ki, float Kd);
void PID_16_Target_Set(PID_16_TypeDef* pPID, float Target_Value);
void PID_16_Target_Apply(PID_16_TypeDef* pPID, float Target_Value);
void PID_16_Input_Set(PID_16_TypeDef* pPID, float I_Threshold);
void PID_16_Output_Set(PID_16_TypeDef* pPID, float I_Limit, float Output_Limit);
void PID_16_Limit_int(int16_t* value, int16_t limit);
void PID_16_Limit_float(float* value, float limit);
void PID_16_Input_Update(PID_16_TypeDef* pPID, float Present_Value);
void PID_16_Output_Update(PID_16_TypeDef* pPID);

void PID_Angle_Param_Set(PID_16_TypeDef* pPID, float Kp, float Ki, float Kd);
void PID_Angle_Target_Set(PID_16_TypeDef* pPID, float Target_Value);
void PID_Angle_Input_Set(PID_16_TypeDef* pPID, float I_Threshold);
void PID_Angle_Output_Set(PID_16_TypeDef* pPID, float I_Limit, float Output_Limit);
void PID_Angle_Limit_int(int16_t* value, int16_t limit);
void PID_Angle_Limit_float(float* value, float limit);
void PID_Angle_Input_Update(PID_16_TypeDef* pPID, float Present_Value);
void PID_Angle_Output_Update(PID_16_TypeDef* pPID);
#endif


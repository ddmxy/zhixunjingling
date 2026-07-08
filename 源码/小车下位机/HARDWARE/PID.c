#include "stm32f10x.h"                  // Device header
#include "PID.h"
#define ABS(x)	( (x>0) ? (x) : (-x) )			// 取绝对值

/**
  * @brief     位置式 PID：比例、积分、微分系数设置
  * @param     PID_16_TypeDef* pPID
  * @param     float Kp  比例系数
  * @param     float Ki  积分系数
  * @param     float Kd  微分系数
  * @retval    无
  */
void PID_16_Param_Set(PID_16_TypeDef* pPID, float Kp, float Ki, float Kd)
{
    pPID->Kp = Kp;
    pPID->Ki = Ki;
    pPID->Kd = Kd;
}

/**
  * @brief     位置式 PID：目标值设定（会清零误差历史、积分与输出，用于目标突变或重新起控）
  * @param     PID_16_TypeDef* pPID
  * @param     float Target_Value  目标值（与测量值同一物理量纲，如 m/s 或 r/min）
  * @retval    无
  */
void PID_16_Target_Set(PID_16_TypeDef* pPID, float Target_Value)
{
    pPID->Target_Value = Target_Value;
    pPID->Present_Value = 0;
    pPID->Error = 0;
    pPID->Last_Error = 0;                                                          
    pPID->Prev_Error = 0;
	pPID->P_Output = 0;
	pPID->I_Output = 0;
	pPID->D_Output = 0;
    pPID->Output = 0;
}

/**
  * @brief     仅更新目标值（不清积分、不清历史误差），用于串口周期下发同一指令时保持积分连续
  * @param     PID_16_TypeDef* pPID
  * @param     float Target_Value  与 Present_Value 同一量纲（如电机 r/min）
  * @retval    无
  */
void PID_16_Target_Apply(PID_16_TypeDef* pPID, float Target_Value)
{
    pPID->Target_Value = Target_Value;
}

/**
  * @brief     位置式 PID：积分分离门限设置
  * @param     PID_16_TypeDef* pPID
  * @param     float I_Threshold  积分门限：|误差|小于此值时才在“三拍同号”时累加积分
  * @retval    无
  */
void PID_16_Input_Set(PID_16_TypeDef* pPID, float I_Threshold)
{
    pPID->I_Threshold = I_Threshold;
}

/**
  * @brief     位置式 PID：积分限幅与输出限幅设置
  * @param     PID_16_TypeDef* pPID
  * @param     float I_Limit      积分项限幅
  * @param     float Output_Limit 总输出限幅（PWM 等）
  * @retval    无
  */
void PID_16_Output_Set(PID_16_TypeDef* pPID, float I_Limit, float Output_Limit)
{
    pPID->I_Limit = I_Limit;
    pPID->Output_Limit = Output_Limit;
}

/**
  * @brief     对 int16_t 限幅
  * @param     int16_t* value  待限幅变量
  * @param     int16_t limit   限幅绝对值
  * @retval    无
  */
void PID_16_Limit_int(int16_t* value, int16_t limit)
{
    if(ABS(*value) > limit)
    {
        if(*value >= 0)
        {
            *value = limit;
        }
        else
        {
            *value = -limit;
        }
    }
}

/**
  * @brief     对 float 限幅
  * @param     float* value  待限幅变量
  * @param     float limit   限幅绝对值
  * @retval    无
  */
void PID_16_Limit_float(float* value, float limit)
{
    if(ABS(*value) > limit)
    {
        if(*value >= 0)
        {
            *value = limit;
        }
        else
        {
            *value = -limit;
        }
    }
}

/**
  * @brief     位置式 PID：输入更新（带积分分离，减小积分饱和）
  * @param     PID_16_TypeDef* pPID
  * @param     float Present_Value  当前测量值（与目标值量纲一致）
  * @retval    无
  */
void PID_16_Input_Update(PID_16_TypeDef* pPID, float Present_Value)
{
    pPID->Prev_Error = pPID->Last_Error; // 上上拍误差
    pPID->Last_Error = pPID->Error;       // 上一拍误差
    pPID->Present_Value = Present_Value;  // 当前测量值
    pPID->Error = pPID->Target_Value - pPID->Present_Value; // 当前误差
    /* 积分分离：仅当误差连续三拍同号时考虑积分，否则衰减积分（防止来回振荡时积分乱累积） */
    if((pPID->Error > 0  && pPID->Last_Error > 0 && pPID->Prev_Error > 0) || (pPID->Error < 0  && pPID->Last_Error < 0 && pPID->Prev_Error < 0))
    {
        /* 误差同向：小误差时累加积分；大误差时清零积分（避免启动或大偏差时积分过猛） */
        if(ABS(pPID->Error) < pPID->I_Threshold)
        {
            pPID->I_Output += pPID->Ki * pPID->Error; // 积分项累加
        }
        else
        {
            pPID->I_Output = 0; // 大偏差下不使用积分项
        }
    }
    else
    {
        /* 误差变号或穿过零区：积分减半，减轻超调与反向时的积分拖尾 */
        pPID->I_Output *= 0.5f;
    }
}

/**
  * @brief     位置式 PID：输出更新（P+I+D，限幅与速度环死区）
  * @param     PID_16_TypeDef* pPID
  * @retval    无
  */
void PID_16_Output_Update(PID_16_TypeDef* pPID)
{
    float out=0 ;
    pPID->P_Output = pPID->Kp * pPID->Error;
    pPID->D_Output = pPID->Kd * (pPID->Error - pPID->Last_Error);

    PID_16_Limit_float(&pPID->I_Output, pPID->I_Limit);

    out = pPID->P_Output + pPID->I_Output + pPID->D_Output;

    PID_16_Limit_float(&out, pPID->Output_Limit);   // 输出限幅
    pPID->Output = (int16_t)out;                    // 转为整型给 PWM
    if(pPID->Output>-PID_DEAD_VELOCITY&&pPID->Output<PID_DEAD_VELOCITY)
    {
        pPID->Output=0;
    }
}

/* ==================== 角度环 PID（结构与速度环相同，死区宏不同） ==================== */

/**
  * @brief     角度环 PID：比例、积分、微分系数设置
  * @param     PID_16_TypeDef* pPID
  * @param     float Kp
  * @param     float Ki
  * @param     float Kd
  * @retval    无
  */
void PID_Angle_Param_Set(PID_16_TypeDef* pPID, float Kp, float Ki, float Kd)
{
    pPID->Kp = Kp;
    pPID->Ki = Ki;
    pPID->Kd = Kd;
}

/**
  * @brief     角度环 PID：目标值设定（清零历史状态）
  * @param     PID_16_TypeDef* pPID
  * @param     float Target_Value
  * @retval    无
  */
void PID_Angle_Target_Set(PID_16_TypeDef* pPID, float Target_Value)
{
    pPID->Target_Value = Target_Value;
    pPID->Present_Value = 0;
    pPID->Error = 0;
    pPID->Last_Error = 0;                                                          
    pPID->Prev_Error = 0;
	pPID->P_Output = 0;
	pPID->I_Output = 0;
	pPID->D_Output = 0;
    pPID->Output = 0;
}

/**
  * @brief     角度环 PID：积分分离门限设置
  * @param     PID_16_TypeDef* pPID
  * @param     float I_Threshold
  * @retval    无
  */
void PID_Angle_Input_Set(PID_16_TypeDef* pPID, float I_Threshold)
{
    pPID->I_Threshold = I_Threshold;
}

/**
  * @brief     角度环 PID：积分限幅与输出限幅设置
  * @param     PID_16_TypeDef* pPID
  * @param     float I_Limit
  * @param     float Output_Limit
  * @retval    无
  */
void PID_Angle_Output_Set(PID_16_TypeDef* pPID, float I_Limit, float Output_Limit)
{
    pPID->I_Limit = I_Limit;
    pPID->Output_Limit = Output_Limit;
}

/**
  * @brief     对 int16_t 限幅（角度环）
  * @param     int16_t* value
  * @param     int16_t limit
  * @retval    无
  */
void PID_Angle_Limit_int(int16_t* value, int16_t limit)
{
    if(ABS(*value) > limit)
    {
        if(*value >= 0)
        {
            *value = limit;
        }
        else
        {
            *value = -limit;
        }
    }
}

/**
  * @brief     对 float 限幅（角度环）
  * @param     float* value
  * @param     float limit
  * @retval    无
  */
void PID_Angle_Limit_float(float* value, float limit)
{
    if(ABS(*value) > limit)
    {
        if(*value >= 0)
        {
            *value = limit;
        }
        else
        {
            *value = -limit;
        }
    }
}

/**
  * @brief     角度环 PID：输入更新（积分分离策略同速度环）
  * @param     PID_16_TypeDef* pPID
  * @param     float Present_Value
  * @retval    无
  */
void PID_Angle_Input_Update(PID_16_TypeDef* pPID, float Present_Value)
{
    pPID->Prev_Error = pPID->Last_Error;
    pPID->Last_Error = pPID->Error;
    pPID->Present_Value = Present_Value;
    pPID->Error = pPID->Target_Value - pPID->Present_Value;
    if((pPID->Error > 0  && pPID->Last_Error > 0 && pPID->Prev_Error > 0) || (pPID->Error < 0  && pPID->Last_Error < 0 && pPID->Prev_Error < 0))
    {
        if(ABS(pPID->Error) < pPID->I_Threshold)
        {
            pPID->I_Output += pPID->Ki * pPID->Error;
        }
        else
        {
            pPID->I_Output = 0;
        }
    }
    else
    {
        pPID->I_Output *= 0.5f;
    }
}

/**
  * @brief     角度环 PID：输出更新（含角度死区 PID_DEAD_ANGLE）
  * @param     PID_16_TypeDef* pPID
  * @retval    无
  */
void PID_Angle_Output_Update(PID_16_TypeDef* pPID)
{
    float out=0 ;
    pPID->P_Output = pPID->Kp * pPID->Error;
    pPID->D_Output = pPID->Kd * (pPID->Error - pPID->Last_Error);

    PID_Angle_Limit_float(&pPID->I_Output, pPID->I_Limit);

    out = pPID->P_Output + pPID->I_Output + pPID->D_Output;

    PID_Angle_Limit_float(&out, pPID->Output_Limit);
    pPID->Output = (int16_t)out;
    if(pPID->Output>-PID_DEAD_ANGLE&&pPID->Output<PID_DEAD_ANGLE)
    {
        pPID->Output=0;
    }
}


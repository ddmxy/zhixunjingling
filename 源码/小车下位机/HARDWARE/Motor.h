#ifndef __Motor_H
#define __Motor_H	 
#include "sys.h"

/* TB6612 逻辑输入：四路电机 IN1/IN2（与 PWM 在 main 中配合） */
#define Motor1_IN1 PAout(4)
#define Motor1_IN2 PCout(4)
#define Motor2_IN1 PCout(2)
#define Motor2_IN2 PAout(5)
#define Motor3_IN1 PFout(15)
#define Motor3_IN2 PFout(13)
#define Motor4_IN1 PFout(9)
#define Motor4_IN2 PFout(11)

/* 编码器：TIM 本周期计数值与增量（与 main 中 TIM1/2/3/8 对应关系见 Motor.c） */
typedef struct
{
    int32_t Wheel_1_dirc;
    uint32_t Wheel_1_current;
    uint32_t Wheel_1_last;
    int16_t Wheel_1_delta;
    
    int32_t Wheel_2_dirc;
    uint32_t Wheel_2_current;
    uint32_t Wheel_2_last;
    int16_t Wheel_2_delta;
    
    int32_t Wheel_3_dirc;
    uint32_t Wheel_3_current;
    uint32_t Wheel_3_last;
    int16_t Wheel_3_delta;
    
    int32_t Wheel_4_dirc;
    uint32_t Wheel_4_current;
    uint32_t Wheel_4_last;
    int16_t Wheel_4_delta;
}Struct_Encoder;
extern Struct_Encoder Car_Encoder;

void Encoder_TIM_Init(TIM_TypeDef* TIMx, uint16_t period);
void Encoder_TIM_F_Init(TIM_TypeDef* TIMx, uint16_t period);
void GPIO_Config_InputPullUp(GPIO_TypeDef* GPIOx, uint16_t pins);
void Encoder_GPIO_Init(void);
void Encoder_InitAll(void);
void Encoder_Struct_Init(void);

void STM32_PWM_Configuration(uint16_t arr, uint16_t psc);
void TB6612_GPIO_Init(void);
void Turn_F(void);
void Turn_B(void);
void Turn_R(void);
void Turn_L(void);
		 				    
#endif
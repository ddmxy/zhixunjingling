#include "Motor.h"
#include "PID.h"

/* TB6612 四路电机 PWM：TIM4 的 CH1~CH4，全重映射到 PD12~PD15 */
void STM32_PWM_Configuration(uint16_t arr, uint16_t psc)
{
    GPIO_InitTypeDef        GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;

    /* 1) 使能 AFIO、GPIOD、TIM4 时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    /* 2) TIM4 全重映射：CH1~CH4 对应 PD12~PD15（依库选择 Remap 宏） */
#ifdef GPIO_Remap_TIM4
    GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE);
#elif defined(GPIO_FullRemap_TIM4)
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM4, ENABLE);
#else
#warning "No TIM4 remap macro found. Check your StdPeriph library for TIM4 remap macro name."
#endif

    /* 3) PD12~PD15 复用推挽输出，接 TB6612 PWM 输入 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* 4) TIM4 时基：arr、psc 决定 PWM 频率与占空比分辨率 */
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler     = psc;
    TIM_TimeBaseStructure.TIM_Period        = arr;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    /* 5) 四通道 PWM1，初始比较值为 0 */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 0;
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;

    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_OC3Init(TIM4, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

    TIM_OC4Init(TIM4, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

    /* 6) ARR 预装载并使能 TIM4 */
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

/* TB6612 方向脚：PA/PC/PF 四路 IN1、IN2，与 Motor.h 中宏对应 */
void TB6612_GPIO_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOF, ENABLE);

     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
     GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_SetBits(GPIOA,GPIO_Pin_4|GPIO_Pin_5);

     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
     GPIO_Init(GPIOC, &GPIO_InitStructure);
     GPIO_SetBits(GPIOC,GPIO_Pin_4|GPIO_Pin_2);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
     GPIO_Init(GPIOF, &GPIO_InitStructure);
    GPIO_SetBits(GPIOF,GPIO_Pin_9|GPIO_Pin_11|GPIO_Pin_13|GPIO_Pin_15);
}

/* 测试：四轮同向前进；在 main 里关 PID 时可与 IN 配合试电机 */
void Turn_F(void)
{
    Motor1_IN1=1;
    Motor1_IN2=0;
    Motor2_IN1=0;
    Motor2_IN2=1;
    Motor3_IN1=1;
    Motor3_IN2=0;
    Motor4_IN1=0;
    Motor4_IN2=1;
}

void Turn_B(void)
{
    Motor1_IN1=0;
    Motor1_IN2=1;
    Motor2_IN1=1;
    Motor2_IN2=0;
    Motor3_IN1=0;
    Motor3_IN2=1;
    Motor4_IN1=1;
    Motor4_IN2=0;
}

void Turn_R(void)
{
    Motor1_IN1=1;
    Motor1_IN2=0;
    Motor2_IN1=1;
    Motor2_IN2=0;
    Motor3_IN1=1;
    Motor3_IN2=0;
    Motor4_IN1=1;
    Motor4_IN2=0;
}

void Turn_L(void)
{
    Motor1_IN1=0;
    Motor1_IN2=1;
    Motor2_IN1=0;
    Motor2_IN2=1;
    Motor3_IN1=0;
    Motor3_IN2=1;
    Motor4_IN1=0;
    Motor4_IN2=1;
}

void Encoder_TIM_Init(TIM_TypeDef* TIMx, uint16_t period)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = period;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

    /* 编码器接口 TI12：A/B 相，四倍频计数 */
    TIM_EncoderInterfaceConfig(TIMx,
                               TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising,
                               TIM_ICPolarity_Rising);

    /* 输入滤波 0~15，此处取 6 */
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 6;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInit(TIMx, &TIM_ICInitStructure);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIMx, &TIM_ICInitStructure);

    TIM_SetCounter(TIMx, 0);
    TIM_Cmd(TIMx, ENABLE);
}

/*
 * 重映射后编码器引脚（与 Encoder_GPIO_Init 一致）：
 * TIM1: PE9 / PE11    TIM2: PA15 / PB3
 * TIM3: PA6 / PA7     TIM8: PC6 / PC7
 */

void GPIO_Config_InputPullUp(GPIO_TypeDef* GPIOx, uint16_t pins)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin   = pins;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode  = GPIO_Mode_IPU;
    GPIO_Init(GPIOx, &gpio);
}

/* 编码器 GPIO：重映射 + 上拉，对应 TIM1/2/3/8 */
void Encoder_GPIO_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO |
                           RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_GPIOC |
                           RCC_APB2Periph_GPIOE, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE); /* TIM1 -> PE9/PE11 */
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE); /* TIM2 -> PA15/PB3 */
    GPIO_Config_InputPullUp(GPIOE, GPIO_Pin_9 | GPIO_Pin_11);
    GPIO_Config_InputPullUp(GPIOA, GPIO_Pin_15);
    GPIO_Config_InputPullUp(GPIOB, GPIO_Pin_3);
    GPIO_Config_InputPullUp(GPIOA, GPIO_Pin_6 | GPIO_Pin_7);
    GPIO_Config_InputPullUp(GPIOC, GPIO_Pin_6 | GPIO_Pin_7);
}

void Encoder_InitAll(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM8, ENABLE);

    Encoder_GPIO_Init();

    Encoder_TIM_Init(TIM2, 0xFFFF);
    Encoder_TIM_Init(TIM1, 0xFFFF);
    Encoder_TIM_Init(TIM3, 0xFFFF);
    Encoder_TIM_Init(TIM8, 0xFFFF);
}

/* 编码器数据结构清零，并初始化速度环 PID（目标/反馈量纲为 r/min） */
void Encoder_Struct_Init(void)
{
    Car_Encoder.Wheel_1_dirc=0;
    Car_Encoder.Wheel_1_current=0;
    Car_Encoder.Wheel_1_last=0;
    Car_Encoder.Wheel_1_delta=0;

    Car_Encoder.Wheel_2_dirc=0;
    Car_Encoder.Wheel_2_current=0;
    Car_Encoder.Wheel_2_last=0;
    Car_Encoder.Wheel_2_delta=0;

    Car_Encoder.Wheel_3_dirc=0;
    Car_Encoder.Wheel_3_current=0;
    Car_Encoder.Wheel_3_last=0;
    Car_Encoder.Wheel_3_delta=0;

    Car_Encoder.Wheel_4_dirc=0;
    Car_Encoder.Wheel_4_current=0;
    Car_Encoder.Wheel_4_last=0;
    Car_Encoder.Wheel_4_delta=0;

    /* Kp/Ki/Kd 与 r/min 量纲匹配；Ki=0 即纯比例；可按响应再改大或改小 */
    PID_16_Param_Set(&PID_V[0], 2.5f, 0.02f, 0.04f);
    PID_16_Param_Set(&PID_V[1], 2.5f, 0.02f, 0.04f);
    PID_16_Param_Set(&PID_V[2], 2.5f, 0.02f, 0.04f);
    PID_16_Param_Set(&PID_V[3], 2.5f, 0.02f, 0.04f);

    PID_16_Target_Set(&PID_V[0], 0);
    PID_16_Target_Set(&PID_V[1], 0);
    PID_16_Target_Set(&PID_V[2], 0);
    PID_16_Target_Set(&PID_V[3], 0);

    /* 积分门限（r/min）：|误差|小于该值时才按同向规则累加积分 */
    PID_16_Input_Set(&PID_V[0], 800.0f);
    PID_16_Input_Set(&PID_V[1], 800.0f);
    PID_16_Input_Set(&PID_V[2], 800.0f);
    PID_16_Input_Set(&PID_V[3], 800.0f);

    PID_16_Output_Set(&PID_V[0],2000, 7000);
    PID_16_Output_Set(&PID_V[1],2000, 7000);
    PID_16_Output_Set(&PID_V[2],2000, 7000);
    PID_16_Output_Set(&PID_V[3],2000, 7000);
}

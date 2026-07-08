#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "timer.h"
#include "Motor.h"
#include "USART.h"
#include "Car.h"
#include "PID.h"
#include "stdio.h"
#include <stdint.h>
#include "IMU.h"
#include "hu_m40.h"
//static void USART1_SendRpm_int(float rpm)
//{
//    int32_t x = (int32_t)(rpm + (rpm >= 0.0f ? 0.5f : -0.5f));
//    if (x < 0) {
//        USART1_SendBits('-');
//        x = -x;
//    }
//    if (x > 999999) {
//        x = 999999;
//    }
//    USART1_SendNum((uint32_t)x, 6);
//}

PID_16_TypeDef PID_V[4];
USART_Data_TypeDef USART_Data;
Struct_Encoder Car_Encoder;
Car_Struct Car;
uint8_t arr1[5]={0xff,0xaa,0x69,0x88,0xb5};
uint8_t arr2[5]={0xff,0xaa,0x01,0x08,0x00};
uint8_t arr3[5]={0xff,0xaa,0x00,0x00,0x00};
IMU_Structure IMU_Struct;
 int main(void)
 {		
    delay_init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    USART1_Init(115200);
    USART2_Init(115200);
    STM32_PWM_Configuration(7199, 0);
    TB6612_GPIO_Init();
    Encoder_InitAll();
    Encoder_Struct_Init();
    Car_Init();
    TIM6_Init();
    TIM5_Init();
    IMU_Init();
    USART2_SendArray(arr1,5);
    delay_ms(200);
    USART2_SendArray(arr2,5);
    delay_ms(3000);
    USART2_SendArray(arr3,5);
    delay_ms(100);  
//    hu_m40_init();
//    Turn_F();
//    TIM_SetCompare1(TIM4, 2000);
//    TIM_SetCompare2(TIM4, 2000);
//    TIM_SetCompare3(TIM4, 600);
//    TIM_SetCompare4(TIM4, 600);
    USART1_SendBits(0x01);
    while(1)
    {
        IMU_Get();

        if(Car.Send_Mindistance_Flag==1&&Car.start_Flag==1)
        {
            Car_Send_Current_V();
            Car.Send_Mindistance_Flag=0;
        }
        if(Car.Send_Yaw_Flag==1&&Car.start_Flag==1)
        {
            Car_Send_Yaw();
            Car.Send_Yaw_Flag=0;
        }        
        if(Car.Update_Speed_Flag==1)
        {
            PID_16_Output_Update(&PID_V[0]);
            PID_16_Output_Update(&PID_V[1]);
            PID_16_Output_Update(&PID_V[2]);
            PID_16_Output_Update(&PID_V[3]);
            
            if(PID_V[0].Output>=0)
            {
                Motor1_IN1=1;
                Motor1_IN2=0;                
                TIM_SetCompare1(TIM4,PID_V[0].Output);
            }
            else 
            {
                Motor1_IN1=0;
                Motor1_IN2=1;                
                TIM_SetCompare1(TIM4,PID_V[0].Output*(-1));                
            }
            
            if(PID_V[1].Output>=0)
            {
                Motor2_IN1=0;
                Motor2_IN2=1;                
                TIM_SetCompare2(TIM4,PID_V[1].Output);
            }
            else 
            {
                Motor2_IN1=1;
                Motor2_IN2=0;                
                TIM_SetCompare2(TIM4,PID_V[1].Output*(-1));                
            }
            
            if(PID_V[2].Output>=0)
            {
                Motor3_IN1=1;
                Motor3_IN2=0;                
                TIM_SetCompare3(TIM4,PID_V[2].Output);
            }
            else 
            {
                Motor3_IN1=0;
                Motor3_IN2=1;                
                TIM_SetCompare3(TIM4,PID_V[2].Output*(-1));                
            }
            
            if(PID_V[3].Output>=0)
            {
                Motor4_IN1=0;
                Motor4_IN2=1;               
                TIM_SetCompare4(TIM4,PID_V[3].Output);
            }
            else 
            {
                Motor4_IN1=1;
                Motor4_IN2=0;                
                TIM_SetCompare4(TIM4,PID_V[3].Output*(-1));                
            }
            Car.Update_Speed_Flag=0;
        }
        Car_Encoder_Input();
        //Car_Remote();
    }	 

 
}	 
 
void USART1_IRQHandler(void)
{
	static uint8_t RX1State=0;
	static uint8_t pRX1Packet=0;
	if(USART_GetITStatus(USART1,USART_IT_RXNE) == SET)   
	{
		uint8_t RX1Data = USART_ReceiveData(USART1);
        if(RX1State==0)
        {
            if(RX1Data == 0xFF)
            {
                RX1State =1;
            }
            else 
            {
                RX1State=0;
            }
        }
        else if(RX1State==1)
        {
            if(RX1Data==0x02)
            {
                RX1State=2;
                pRX1Packet=0;
            }
            else
            {
                RX1State=0;
            }                
        }
        else if(RX1State == 2)		
        {
            USART_Data.USART1_RxPacket[pRX1Packet] = RX1Data;
            pRX1Packet++;
            if(pRX1Packet>=8)						
            {
                RX1State=3;
            }
        }
        else if(RX1State == 3)						
        {
            if(RX1Data == 0xFE)
            {
                RX1State = 0;
                USART_Data.USART1_RxFlag = 1;
            }
            else
            {
                RX1State = 0;
            }
        }    
        USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	}
}


void TIM6_IRQHandler(void) 
{
    if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
    {

        Car_Encoder.Wheel_1_last=Car_Encoder.Wheel_1_current;
        Car_Encoder.Wheel_2_last=Car_Encoder.Wheel_2_current;
        Car_Encoder.Wheel_3_last=Car_Encoder.Wheel_3_current;
        Car_Encoder.Wheel_4_last=Car_Encoder.Wheel_4_current;
        
        Car_Encoder.Wheel_1_current=TIM_GetCounter(TIM2);
        Car_Encoder.Wheel_2_current=TIM_GetCounter(TIM1);
        Car_Encoder.Wheel_3_current=TIM_GetCounter(TIM3);
        Car_Encoder.Wheel_4_current=TIM_GetCounter(TIM8);
        
        Car_Encoder.Wheel_1_delta=(int16_t)(Car_Encoder.Wheel_1_current-Car_Encoder.Wheel_1_last);
        Car_Encoder.Wheel_2_delta=(int16_t)(Car_Encoder.Wheel_2_current-Car_Encoder.Wheel_2_last);
        Car_Encoder.Wheel_3_delta=(int16_t)(Car_Encoder.Wheel_3_current-Car_Encoder.Wheel_3_last);
        Car_Encoder.Wheel_4_delta=(int16_t)(Car_Encoder.Wheel_4_current-Car_Encoder.Wheel_4_last);
        
             
        PID_V[0].Present_Value = Car_EncoderDelta_To_MotorRpm(Car_Encoder.Wheel_1_delta * CAR_ENCODER_SIGN_W1);
        PID_V[1].Present_Value = Car_EncoderDelta_To_MotorRpm(Car_Encoder.Wheel_2_delta * CAR_ENCODER_SIGN_W2);
        PID_V[2].Present_Value = Car_EncoderDelta_To_MotorRpm(Car_Encoder.Wheel_3_delta * CAR_ENCODER_SIGN_W3);
        PID_V[3].Present_Value = Car_EncoderDelta_To_MotorRpm(Car_Encoder.Wheel_4_delta * CAR_ENCODER_SIGN_W4);
        
        PID_16_Input_Update(&PID_V[0],PID_V[0].Present_Value);
        PID_16_Input_Update(&PID_V[1],PID_V[1].Present_Value);
        PID_16_Input_Update(&PID_V[2],PID_V[2].Present_Value);
        PID_16_Input_Update(&PID_V[3],PID_V[3].Present_Value);
        Car.Update_Speed_Flag = 1;
    }
    TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
}

void TIM5_IRQHandler(void)
{    
    if(TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
    {
        Car.Time_Flag++;
        if(Car.Time_Flag==1)
        {
            Car.Send_Mindistance_Flag=1;
        }
        else if(Car.Time_Flag==2)
        {
            Car.Send_Yaw_Flag=1;
            Car.Time_Flag=0;
        }
        TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
    }
}

void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
        uint8_t data = USART_ReceiveData(USART2);
		if(USART_Data.USART2_State == 0)
		{
			if(data == 0x55)					
			{
				USART_Data.USART2_State = 1;
			}
            else 
            {
                USART_Data.USART2_State = 0;
            }
		}
		else if(USART_Data.USART2_State == 1)
		{
			if(data == 0x53 || data == 0x52)				
			{
				USART_Data.USART2_State = 2;
                USART_Data.USART2_pPacket = 0;
                USART_Data.USART2_RxPacket[USART_Data.USART2_pPacket]=data;
                USART_Data.USART2_pPacket++;
			}
            else 
            {
                USART_Data.USART2_State = 0;
            }
		}
		else if(USART_Data.USART2_State == 2)
		{
			USART_Data.USART2_RxPacket[USART_Data.USART2_pPacket] = data;
			USART_Data.USART2_pPacket++;
			if(USART_Data.USART2_pPacket >= 10)	
			{
				USART_Data.USART2_State = 0;
				USART_Data.USART2_pPacket = 0;
				USART_Data.USART2_RxFlag = 1;
			}
		}
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

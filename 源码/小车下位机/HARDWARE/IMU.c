#include "IMU.h"
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

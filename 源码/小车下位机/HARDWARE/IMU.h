#ifndef __IMU_H
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

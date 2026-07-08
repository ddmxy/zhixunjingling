
/***********************************************************************
 * 
 * 文件名   hu_m40.h
 * 作者     hche
 * E-mail   
 * 日期     2023/07/08
 * 版本     V1.1.0
 * 说明     HU遥控手柄解码
 * 
 ************************************************************************
 * 金华市核芯风暴智能科技有限公司
 ************************************************************************/

#ifndef __HU_M40
#define __HU_M40


#include <stdint.h>


#define HU_M40_ADDR   1  // 地址码，有冲突时修改此地址


#define HU_K1   0x0001
#define HU_K2   0x0002
#define HU_K3   0x0004
#define HU_K4   0x0008
#define HU_K5   0x0010
#define HU_K6   0x0020


#define HU_RX 4
#define HU_RY 5
#define HU_LX 6
#define HU_LY 7

extern uint8_t* g_hu_m40_tx_addr;
void hu_m40_rx_mode(uint8_t* txaddr, uint8_t* rxaddr);
void hu_m40_tx_mode(uint8_t* txaddr, uint8_t* rxaddr);


void hu_m40_write(uint8_t rx, uint8_t ry, uint8_t lx, uint8_t ly, uint8_t key);
uint8_t hu_m40_init(void);
uint8_t hu_m40_read(void);
extern uint8_t hu_m40_rx_data[16];

uint8_t hu_m40_button(uint16_t button);                //will be TRUE if button is being pressed
uint8_t hu_m40_analog(uint8_t button);

#endif

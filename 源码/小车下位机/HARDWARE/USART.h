#ifndef __USART_H__
#define __USART_H__
#include "sys.h"

typedef struct{
	uint8_t USART1_RxPacket[30];
	uint8_t USART1_TxPacket[30];
	uint8_t USART1_RxFlag;
	uint8_t USART1_State;
	uint8_t USART1_pPacket;
	uint8_t USART1_Length;
    
	uint8_t USART2_RxPacket[30];
	uint8_t USART2_TxPacket[30];
	uint8_t USART2_RxFlag;
	uint8_t USART2_State;
	uint8_t USART2_pPacket;
    uint8_t USART2_Length;
	
	uint8_t USART3_RxPacket[20];
	uint8_t USART3_TxPacket[20];
	uint8_t USART3_RxFlag;
	uint8_t USART3_State;
	uint8_t USART3_pPacket;
    uint8_t USART3_Length;
    
    uint8_t USART4_RxPacket[20];
	uint8_t USART4_TxPacket[20];
	uint8_t USART4_RxFlag;
	uint8_t USART4_State;
	uint8_t USART4_pPacket;
    uint8_t USART4_Length;
}USART_Data_TypeDef;	

extern USART_Data_TypeDef USART_Data;

void USART1_Init(u32 BaudRate);
void USART1_SendBits(uint8_t data);
void USART1_SendArray(uint8_t *Array,uint8_t Length);
void USART1_SendString(uint8_t *String);
void USART1_SendNum(uint32_t Number,uint8_t Length);

void USART2_Init(u32 BaudRate);
void USART2_SendBits(uint8_t data);
void USART2_SendArray(uint8_t *Array,uint8_t Length);
void USART2_SendString(uint8_t *String);
void USART2_SendNum(uint32_t Number,uint8_t Length);

void USART3_Init(u32 BaudRate);
void USART3_SendBits(uint8_t data);
void USART3_SendArray(uint8_t *Array,uint8_t Length);
void USART3_SendString(uint8_t *String);
void USART3_SendNum(uint32_t Number,uint8_t Length);

void UART4_Init(uint32_t baudrate);
void UART4_SendBits(uint8_t data);
void UART4_SendArray(uint8_t *Array, uint8_t Length);
void UART4_SendString(uint8_t *String);
void UART4_SendNum(uint32_t Number, uint8_t Length);
#endif


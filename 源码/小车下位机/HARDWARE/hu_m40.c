
/***********************************************************************
 * 
 * 文件名   g_hu_m40.c
 * 作者     hche
 * E-mail   
 * 日期     2023/07/08
 * 版本     V1.1.0
 * 说明     HU遥控手柄解码
 * 
 ************************************************************************
 * 金华市核芯风暴智能科技有限公司
 ***********************************************************************
 
 使用方法：
 
 // 初始化
 void setup() {
   hu_m40_init();
 }
 
 // 轮询
 void loop() {
   if (hu_m40_read() == 1) // ==1有收到数据
   {
     if( hu_m40_analog(HU_LX) > 200 ) // 左摇杆X值> 200
	 {
       // TODO
	 }
	 
     if (hu_m40_button(HU_K1))  // KEY1 按下
     {
       // TODO
     }
   }
 }
 
 */

#include "hu_m40.h"
#include "nRF24L01.h"


#define PAYLOAD_WIDTH    16


/*-------------------------------------------

         以下根据不同平台自行实现  
		 
----------------------------------------------*/

#define  HU_M40_TIMEOUT_CNT     19999     // 根据不同平台配置通信超时时间，一般配置成2秒

/********************************************************
* 函数名    hu_m40_init_hw
* 输入      无
* 输出      无
* 返回      无
* 说明      硬件初始化
********************************************************/
void hu_m40_init_hw(void)
{
  // TODO，初始化硬件
  // 参数：
  // 通道25、、、
  // 开启自动应答
  // 开启CRC，CRC16
  // 数据长度16
  // 
  
  NRF24L01_Init(); //
}

/********************************************************
* 函数名    hu_m40_tx_mode
* 输入      txaddr：发送端点地址
*          rxaddr: 接收端点地址
* 输出      无
* 返回      无
* 说明      发送模式
********************************************************/
void hu_m40_tx_mode(uint8_t* txaddr, uint8_t* rxaddr)
{
  // TODO，实现发送模式
  NRF24L01_TX_Mode( txaddr, rxaddr);
}

/********************************************************
* 函数名    hu_m40_rx_mode
* 输入      txaddr：发送端点地址
*          rxaddr: 接收端点地址
* 输出      无
* 返回      无
* 说明      接收模式
********************************************************/
void hu_m40_rx_mode(uint8_t* txaddr, uint8_t* rxaddr)
{
  // TODO，实现接收模式
  NRF24L01_RX_Mode( txaddr, rxaddr);
}

/********************************************************
* 函数名    hu_m40_send_data
* 输入      buf：数据
*          len: 数据长度
* 输出      无
* 返回      无
* 说明      发送数据
********************************************************/
void hu_m40_send_data(uint8_t* buf, uint8_t len)
{
  // TODO，实现发送数据
  
  uint8_t status = 0x00;
  
  len = len > PAYLOAD_WIDTH? PAYLOAD_WIDTH : len;
  NRF24L01_Write_From_Buf(NRF24L01_CMD_TX_PLOAD_W, buf, len);
  
  while( 1 )
  {
    status = NRF24L01_Read_Reg(NRF24L01_REG_STATUS);
    if(status & NRF24L01_FLAG_TX_DSENT) {
      NRF24L01_ClearIRQFlag(NRF24L01_FLAG_TX_DSENT);
      
      break;

    } else if(status & NRF24L01_FLAG_MAX_RT) {
      NRF24L01_FlushTX();
      NRF24L01_ClearIRQFlag(NRF24L01_FLAG_MAX_RT);
      
      break;

    }
	
  }
}

/********************************************************
* 函数名    hu_m40_received_data
* 输入      buf：数据
*          len: 数据长度
* 输出      无
* 返回      数据长度
* 说明      接收数据
********************************************************/
uint8_t hu_m40_received_data(uint8_t* buf, uint8_t len)
{
  // TODO，实现接收数据
  uint8_t status;
  
  status = NRF24L01_Read_Reg(NRF24L01_REG_STATUS);
  if(status & NRF24L01_FLAG_RX_DREADY) {
    NRF24L01_Read_To_Buf(NRF24L01_CMD_RX_PLOAD_R, buf, len);
    NRF24L01_ClearIRQFlag(NRF24L01_FLAG_RX_DREADY);
    
    return 1;
  }
  
  return 0;
}


/*-------------------------------------------

         一般情况下以下不用修改
		 
----------------------------------------------*/

const uint8_t HU_M40_ADDR_DEF[5] = "HXFB0";

uint8_t g_hu_m40_rx_data[16];
uint32_t g_hu_m40_last_buttons;
uint32_t g_hu_m40_buttons;

uint8_t g_hu_m40_rx_addr[5] = "HXFB1";

uint8_t* g_hu_m40_tx_addr;

uint8_t hu_m40_check_sum(uint8_t* buf, uint8_t len);
void hu_m40_ack(uint8_t* buf, uint8_t len);

/********************************************************
* 函数名    hu_m40_button
* 输入      button：可选
*          HU_K1
*          HU_K2
*          HU_K3
*          HU_K4
*          HU_K5
*          HU_K6
*           
* 输出      无
* 返回      1：按下
*           0: 松开
* 说明      获取按键状态
********************************************************/
uint8_t hu_m40_button(uint16_t button)
{
  return ((g_hu_m40_buttons & button) > 0);
}

/********************************************************
* 函数名    hu_m40_analog
* 输入      button：可选
*          HU_RX
*          HU_RY
*          HU_LX
*          HU_LY
* 输出      无
* 返回      模拟值
* 说明      获取摇杆状态
********************************************************/
uint8_t hu_m40_analog(uint8_t button)
{
   return g_hu_m40_rx_data[button];
}

/********************************************************
* 函数名    g_hu_m40_init
* 输入      无
* 输出      无
* 返回      1：初始化成功
*           0: 初始化失败
* 说明      初始化
********************************************************/
uint8_t hu_m40_init(void)
{
  uint8_t res;
  uint8_t i;
  
  hu_m40_init_hw();
  
  hu_m40_rx_mode( (uint8_t*)HU_M40_ADDR_DEF, (uint8_t*)HU_M40_ADDR_DEF );
  g_hu_m40_tx_addr = (uint8_t*)HU_M40_ADDR_DEF;
  
  for( i=0;i<16;i++)
  {
    g_hu_m40_rx_data[i] = 0;
  }

  g_hu_m40_rx_data[HU_RX] = 0x80;
  g_hu_m40_rx_data[HU_RY] = 0x80;
  g_hu_m40_rx_data[HU_LX] = 0x80;
  g_hu_m40_rx_data[HU_LY] = 0x80;

  g_hu_m40_buttons = 0;
  
  return res;
}

/********************************************************
* 函数名    read
* 输入      无
* 输出      无
* 返回      1：成功
*           0: 失败
* 说明      读取
********************************************************/
uint8_t hu_m40_read(void)
{
  static uint32_t ticks=0;
  uint16_t ticks1=500;
  uint8_t dat[PAYLOAD_WIDTH]={0};
  uint8_t i;
  static uint8_t res=1;
  
  ticks++;
  
  if( ticks > HU_M40_TIMEOUT_CNT ) // 通信超时
  {
    ticks = 0;

    hu_m40_init(); // 重新初始化

    res = 0;
  }
  
  while(ticks1--);
  
  if( hu_m40_received_data( dat, PAYLOAD_WIDTH ) )
  {
    if( dat[0] == 1 && dat[1] == 3 && dat[15] == hu_m40_check_sum( dat, 15 ) ) // 校验成功
    {
      ticks = 0;
      for( i=0;i<16;i++)
      {
        g_hu_m40_rx_data[i] = dat[i];
      }

      g_hu_m40_buttons = g_hu_m40_rx_data[8];
      
      res = 1;
      
      // 返回数据到遥控器
      dat[0] = 0x01; // 固定为1
      dat[1] = 0x83; // 功能码
      dat[2] = dat[2]; // 序号
      dat[3] = 11; // 长度
      dat[4] = g_hu_m40_rx_addr[0];
      dat[5] = g_hu_m40_rx_addr[1];
      dat[6] = g_hu_m40_rx_addr[2];
      dat[7] = g_hu_m40_rx_addr[3];
      dat[8] = g_hu_m40_rx_addr[4];
      dat[15] = hu_m40_check_sum( dat, 15 );
      
      hu_m40_tx_mode( g_hu_m40_tx_addr, g_hu_m40_tx_addr );
      hu_m40_send_data(dat, PAYLOAD_WIDTH);         //发送数据
	  
      hu_m40_rx_mode( g_hu_m40_rx_addr, g_hu_m40_rx_addr );
      g_hu_m40_tx_addr = g_hu_m40_rx_addr;
    }
  }
  
  return res;
}

uint8_t hu_m40_check_sum(uint8_t* buf, uint8_t len)
{
  uint8_t i, sum;
  sum = 0;
  for( i=0;i<len;i++)
  {
    sum += buf[i];
  }

  return sum;
}

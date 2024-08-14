/************************************************ 
* WKS Mini GD32开发板
* USB-hw_config 代码	   
* 版本：V1.0								  
************************************************/	

#ifndef __HWCONFIG_H
#define __HWCONFIG_H
#include <stdint.h>
#include "sys.h"

#define USBD_CTL                    REG32(USBD_BASE + 0x40U)

#define USB_USART_TXFIFO_SIZE   1024	//USB虚拟串口发送FIFO大小		
#define USB_USART_REC_LEN	    	200		//USB串口接收缓冲区最大字节数

extern uint8_t  USB_USART_RX_BUF[USB_USART_REC_LEN]; //接收缓冲,最大USB_USART_REC_LEN个字节.末字节为换行符 
extern uint16_t USB_USART_RX_STA;   				     	   //接收状态标记

void Error_Handler(void);
void Enter_LowPowerMode(void);
void usbd_port_config(uint8_t state);
uint8_t USB_GetStatus(void);

void USB_USART_SendData(uint8_t *data);
void USB_Printf(char* fmt,...);

void USB_To_USART_Send_Data(uint8_t* data_buffer, uint32_t Nb_bytes);
#endif


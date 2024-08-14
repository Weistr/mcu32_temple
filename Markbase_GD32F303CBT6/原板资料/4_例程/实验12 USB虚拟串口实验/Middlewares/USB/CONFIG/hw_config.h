/************************************************ 
* WKS Mini GD32������
* USB-hw_config ����	   
* �汾��V1.0								  
************************************************/	

#ifndef __HWCONFIG_H
#define __HWCONFIG_H
#include <stdint.h>
#include "sys.h"

#define USBD_CTL                    REG32(USBD_BASE + 0x40U)

#define USB_USART_TXFIFO_SIZE   1024	//USB���⴮�ڷ���FIFO��С		
#define USB_USART_REC_LEN	    	200		//USB���ڽ��ջ���������ֽ���

extern uint8_t  USB_USART_RX_BUF[USB_USART_REC_LEN]; //���ջ���,���USB_USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern uint16_t USB_USART_RX_STA;   				     	   //����״̬���

void Error_Handler(void);
void Enter_LowPowerMode(void);
void usbd_port_config(uint8_t state);
uint8_t USB_GetStatus(void);

void USB_USART_SendData(uint8_t *data);
void USB_Printf(char* fmt,...);

void USB_To_USART_Send_Data(uint8_t* data_buffer, uint32_t Nb_bytes);
#endif


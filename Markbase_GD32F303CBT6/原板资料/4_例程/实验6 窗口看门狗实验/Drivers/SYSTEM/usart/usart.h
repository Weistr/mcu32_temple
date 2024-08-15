#ifndef __USART_H
#define __USART_H

#include "stdio.h"
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//����0��ʼ��
////////////////////////////////////////////////////////////////////////////////// 
//����봮���жϽ��գ��벻Ҫע�����º궨��
#define USART_REC_LEN               200         /* �����������ֽ��� 200 */
#define USART_EN_RX                 1           /* ʹ�ܣ�1��/��ֹ��0������0���� */

extern uint8_t  USART_RX_BUF[USART_REC_LEN];    /*���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з�*/ 
extern uint16_t USART_RX_STA;         		      /*����״̬���*/	

void usart_init(uint32_t bound);                /* ���ڳ�ʼ������ */

#endif



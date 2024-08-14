/************************************************ 
* WKS Mini GD32������
* �������� ��������	   
* �汾��V1.0								  
************************************************/	

#ifndef _KEY_H
#define _KEY_H
#include "sys.h"


#define KEY0        gpio_input_bit_get(GPIOA,GPIO_PIN_13)  //��ȡKEY0����
#define KEY1        gpio_input_bit_get(GPIOA,GPIO_PIN_15)  //��ȡKEY1����
#define WK_UP       gpio_input_bit_get(GPIOA,GPIO_PIN_0)   //��ȡWKUP����


#define KEY0_PRES 	1             //KEY0����
#define KEY1_PRES	  2             //KEY1����
#define WKUP_PRES   3             //KEY_UP����(��WK_UP)

void KEY_Init(void);              //������ʼ������
uint8_t KEY_Scan(uint8_t mode);   //����ɨ�躯��
#endif

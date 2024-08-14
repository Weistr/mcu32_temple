#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"
#include "key.h"

int main(void)
{ 
	  delay_init(120);                          //��ʼ����ʱ���� 
	  LED_Init();							                  //��ʼ��LED
	  KEY_Init();                               //��ʼ������
	  LED1(0);
    while(1)
		{		
			if( KEY_Scan() == KEY_ON )		// �����������
			{
				LED1_TOGGLE();	// ��תLED״̬
			}
		}
}

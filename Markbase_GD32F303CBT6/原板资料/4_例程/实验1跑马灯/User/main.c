#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"


int main(void)
{ 
	  delay_init(120);                          //��ʼ����ʱ���� 
	  LED_Init();							                  //��ʼ��LED
    while(1)
		{
			LED1(1);                                // LED1 �� 
			delay_ms(100);
			LED1(0);                                // LED1 �� 
			delay_ms(100);
		}
}

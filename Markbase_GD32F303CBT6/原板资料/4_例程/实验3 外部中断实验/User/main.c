#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"
#include "key.h"
#include "exti.h" 

int main(void)
{ 
	  delay_init(120);                          //��ʼ����ʱ���� 
	  usart_init(115200);                       //��ʼ������
	  LED_Init();							                  //��ʼ��LED
	  EXTI_Init();						                  //��ʼ���ⲿ�ж�
    while(1)
		{
        printf("OK\r\n");
        delay_ms(1000);
		}
}

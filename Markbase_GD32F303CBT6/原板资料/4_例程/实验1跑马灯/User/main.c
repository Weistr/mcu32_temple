#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"


int main(void)
{ 
	  delay_init(120);                          //初始化延时函数 
	  LED_Init();							                  //初始化LED
    while(1)
		{
			LED1(1);                                // LED1 灭 
			delay_ms(100);
			LED1(0);                                // LED1 亮 
			delay_ms(100);
		}
}

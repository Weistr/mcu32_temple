#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"
#include "key.h"
#include "exti.h" 

int main(void)
{ 
	  delay_init(120);                          //初始化延时函数 
	  usart_init(115200);                       //初始化串口
	  LED_Init();							                  //初始化LED
	  EXTI_Init();						                  //初始化外部中断
    while(1)
		{
        printf("OK\r\n");
        delay_ms(1000);
		}
}

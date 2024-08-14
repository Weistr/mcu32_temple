#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"
#include "timer.h"

int main(void)
{ 
	  delay_init(120);                          //初始化延时函数 
	  usart_init(115200);                       //初始化串口
	  LED_Init();							                  //初始化LED
	  TIM2_Int_Init(5000-1,12000-1);            //设置10Khz的计数频率，计数5000次为500ms
    while(1)
		{
      printf("OK\n");
      delay_ms(200);
		}
}

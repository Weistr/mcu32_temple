#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"
#include "rtc.h" 	


int main(void)
{ 	
	  delay_init(120);                 //初始化延时函数 
	  usart_init(115200);              //初始化串口
	  LED_Init();							         //初始化LED
	
		while(RTC_Init())		//RTC初始化，检测时钟是否工作正常
	  { 
			printf("RTC ERROR!");
			delay_ms(800);
			printf("RTC Trying...");

	  }		  

  	while(1) 
	  {		 

			printf("%d:%d:%d\r\n",calendar.hour,calendar.min,calendar.sec);	// 打印时分秒

			
		LED1_TOGGLE();                                //翻转一次LED0 
		                                      
		delay_ms(999); 
	 } 
}

#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"
#include "rtc.h" 	


int main(void)
{ 	
	  delay_init(120);                 //��ʼ����ʱ���� 
	  usart_init(115200);              //��ʼ������
	  LED_Init();							         //��ʼ��LED
	
		while(RTC_Init())		//RTC��ʼ�������ʱ���Ƿ�������
	  { 
			printf("RTC ERROR!");
			delay_ms(800);
			printf("RTC Trying...");

	  }		  

  	while(1) 
	  {		 

			printf("%d:%d:%d\r\n",calendar.hour,calendar.min,calendar.sec);	// ��ӡʱ����

			
		LED1_TOGGLE();                                //��תһ��LED0 
		                                      
		delay_ms(999); 
	 } 
}

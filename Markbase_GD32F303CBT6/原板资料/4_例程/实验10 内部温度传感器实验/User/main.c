#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "usmart.h"
#include "led.h"

#include "adc.h" 	


int main(void)
{ 	
    short temp;
	
	  delay_init(120);                 //初始化延时函数 
	  usart_init(115200);              //初始化串口
	  LED_Init();							         //初始化LED
    adc_temperature_init();          //初始化ADC内部温度传感器

	

	
  	while(1) 
	  {		 
				temp=Get_Temprate();	//得到温度值 
			
				if(temp<0)
				{
					temp=-temp;
				}else printf("");	//无符号
				
				printf("%d.",temp/100);
				printf("%d°C",temp%100);
				printf("\n");
				
				
				LED1_TOGGLE();    //LED0闪烁,提示程序运行 
				delay_ms(500);	
	 } 
}



#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"
#include "usmart.h"


//LED状态设置函数 
void led_set(uint8_t sta)
{
    LED1(sta);
}

//函数参数调用测试函数 
void test_fun(void(*ledset)(uint8_t), uint8_t sta)
{
    ledset(sta);
}

int main(void)
{ 	
	  delay_init(120);                 //初始化延时函数 
	  usart_init(115200);              //初始化串口
	  usmart_init(120);	               //初始化USMART
	  LED_Init();							         //初始化LED

  	while(1) 
	 {		 

	 } 
}

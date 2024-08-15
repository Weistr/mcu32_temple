#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"
#include "key.h"

int main(void)
{ 
	  delay_init(120);                          //初始化延时函数 
	  LED_Init();							                  //初始化LED
	  KEY_Init();                               //初始化按键
	  LED1(0);
    while(1)
		{		
			if( KEY_Scan() == KEY_ON )		// 如果按键按下
			{
				LED1_TOGGLE();	// 翻转LED状态
			}
		}
}

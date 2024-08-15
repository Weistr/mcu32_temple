#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"
#include "key.h"
#include "fwdgt.h"

int main(void)
{ 	
	  delay_init(120);                          //初始化延时函数 
	  usart_init(115200);                       //初始化串口
	  LED_Init();							                  //初始化LED
	  KEY_Init();							                  //初始化按键
	  delay_ms(100);                  	        //延时100ms再初始化看门狗,LED0的变化"可见"
	  FWDGT_Init(FWDGT_PSC_DIV64,625);  	      //分频数为64,重载值为625,溢出时间为1s	
	  LED1(0);                                  //点亮LED0
    while(1)
		{
        if(KEY_Scan()==KEY_ON)  	        //如果WK_UP按下，喂狗
        {
            FWDGT_Feed();    			            //喂狗
        }
        delay_ms(10); 
		}
}



#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h"
#include "wwdgt.h"

int main(void)
{ 	
	  delay_init(120);                            //初始化延时函数 
	  usart_init(115200);                         //初始化串口
	  LED_Init();							                    //初始化LED
	  LED1(0);                                    //点亮LED1
	  delay_ms(300);                  	          //延时300ms再初始化看门狗,LED1的变化"可见"
	  WWDGT_Init(0X7F, 0X5F, WWDGT_CFG_PSC_DIV8); //计数器值为7F，窗口值为5F，预分频值为8
    while(1)
		{
       LED1(1);                                 //关闭LED1  
		}
}



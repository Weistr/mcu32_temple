/************************************************
 GenBotter Mini GD32开发板
 Template工程模板-新建工程使用
************************************************/

#include "sys.h"
#include "usart.h"		
#include "delay.h"	


int main(void)
{
	  delay_init(120);                     //初始化延时函数 
	  usart_init(115200);                  //初始化串口
		rcu_periph_clock_enable(RCU_GPIOC);  //GPIOA时钟使能
		gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13); //设置PC13推挽输出
    while(1)
			{
				gpio_bit_reset(GPIOC, GPIO_PIN_13);  //PC13置0
				delay_ms(500);
				gpio_bit_set(GPIOC, GPIO_PIN_13);  //PC13置1
				delay_ms(500);
			}
}



/************************************************ 
* WKS Mini GD32开发板
* 外部中断 驱动代码	   
* 版本：V1.0								  
************************************************/	

#include "exti.h"
#include "delay.h"
#include "led.h"
#include "key.h"
#include "usart.h"

//外部中断初始化
void EXTI_Init(void)
{   
    rcu_periph_clock_enable(RCU_AF); //使能AF时钟	
	
    KEY_Init();                      //初始化按键对应的IO口
	    
		gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOA, GPIO_PIN_SOURCE_15);  //选择PA0作为EXTI源

		
		exti_init(EXTI_15, EXTI_INTERRUPT, EXTI_TRIG_RISING);   //初始化EXTI_0

		
	  nvic_irq_enable(EXTI10_15_IRQn, 2, 0);         //使能外部中断线EXTI0中断请求并设置优先级，抢占优先级为2，响应优先级为0


		exti_interrupt_flag_clear(EXTI_15);         //清除EXTI Line15上的中断标志位
}


//外部中断13服务程序
void EXTI10_15_IRQHandler(void)
{

	   if (exti_flag_get(EXTI_15) != RESET)
	{
		 delay_ms(10);
		 LED1_TOGGLE();
	}
			
	  exti_interrupt_flag_clear(EXTI_15);  //清除EXTI Line0上的中断标志位  
}






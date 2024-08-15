#include "wwdgt.h"
#include "led.h"
#include "usart.h"	

//初始化窗口看门狗 	
//tr   :T[6:0],计数器值 
//wr   :W[6:0],窗口值 
//fprer:分频系数,范围:WWDGT_CFG_PSC_DIV1 ~WWDGT_CFG_PSC_DIV8
//Fwwdg=PCLK1/(4096*2^fprer). 一般PCLK1=60Mhz
void WWDGT_Init(uint8_t tr,uint8_t wr,uint32_t fprer)
{
  rcu_periph_clock_enable(RCU_WWDGT);         //使能WWDGT时钟
	
	wwdgt_config(tr, wr, fprer);                //设置WWDGT计数器值、窗口值和预分频值 ;
	
	wwdgt_enable();                             //使能窗口看门狗定时器
	
	wwdgt_flag_clear();                         //清除WWDGT提前唤醒中断标志位状态
	
	wwdgt_interrupt_enable();                   //使能窗口看门狗提前唤醒中断
	
	nvic_irq_enable(WWDGT_IRQn, 2, 3);          //抢占优先级2，响应优先级为3
}
   
//窗口看门狗中断服务函数
void WWDGT_IRQHandler(void)
{ 
  wwdgt_counter_update(127);                  //更新窗口看门狗值
	
	wwdgt_flag_clear();                         //清除WWDGT提前唤醒中断标志位状态
	
	LED1_TOGGLE();                              //LED1闪烁
}


#include "led.h"


//LED IO初始化
void LED_Init(void)
{
    rcu_periph_clock_enable(RCU_GPIOC);  //GPIOC时钟使能
	
		gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13); //设置PC13推挽输出

	  LED1(1);                             //关闭LED1
}
//翻转IO口状态
void gpio_togglepin(uint32_t gpio_periph, uint32_t pin)
{
  uint32_t octl;

  octl = GPIO_OCTL(gpio_periph);

  GPIO_BOP(gpio_periph) = ((octl & pin) << 16u) | (~octl & pin);
}

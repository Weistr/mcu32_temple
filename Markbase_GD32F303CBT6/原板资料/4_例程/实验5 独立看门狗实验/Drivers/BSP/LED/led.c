#include "led.h"


//LED IO��ʼ��
void LED_Init(void)
{
    rcu_periph_clock_enable(RCU_GPIOC);  //GPIOCʱ��ʹ��
	
		gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13); //����PC13�������

	  LED1(1);                             //�ر�LED1
}
//��תIO��״̬
void gpio_togglepin(uint32_t gpio_periph, uint32_t pin)
{
  uint32_t octl;

  octl = GPIO_OCTL(gpio_periph);

  GPIO_BOP(gpio_periph) = ((octl & pin) << 16u) | (~octl & pin);
}

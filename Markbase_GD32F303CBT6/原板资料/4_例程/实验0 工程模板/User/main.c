/************************************************
 GenBotter Mini GD32������
 Template����ģ��-�½�����ʹ��
************************************************/

#include "sys.h"
#include "usart.h"		
#include "delay.h"	


int main(void)
{
	  delay_init(120);                     //��ʼ����ʱ���� 
	  usart_init(115200);                  //��ʼ������
		rcu_periph_clock_enable(RCU_GPIOC);  //GPIOAʱ��ʹ��
		gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13); //����PC13�������
    while(1)
			{
				gpio_bit_reset(GPIOC, GPIO_PIN_13);  //PC13��0
				delay_ms(500);
				gpio_bit_set(GPIOC, GPIO_PIN_13);  //PC13��1
				delay_ms(500);
			}
}



/************************************************ 
* WKS Mini GD32������
* �ⲿ�ж� ��������	   
* �汾��V1.0								  
************************************************/	

#include "exti.h"
#include "delay.h"
#include "led.h"
#include "key.h"
#include "usart.h"

//�ⲿ�жϳ�ʼ��
void EXTI_Init(void)
{   
    rcu_periph_clock_enable(RCU_AF); //ʹ��AFʱ��	
	
    KEY_Init();                      //��ʼ��������Ӧ��IO��
	    
		gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOA, GPIO_PIN_SOURCE_15);  //ѡ��PA0��ΪEXTIԴ

		
		exti_init(EXTI_15, EXTI_INTERRUPT, EXTI_TRIG_RISING);   //��ʼ��EXTI_0

		
	  nvic_irq_enable(EXTI10_15_IRQn, 2, 0);         //ʹ���ⲿ�ж���EXTI0�ж������������ȼ�����ռ���ȼ�Ϊ2����Ӧ���ȼ�Ϊ0


		exti_interrupt_flag_clear(EXTI_15);         //���EXTI Line15�ϵ��жϱ�־λ
}


//�ⲿ�ж�13�������
void EXTI10_15_IRQHandler(void)
{

	   if (exti_flag_get(EXTI_15) != RESET)
	{
		 delay_ms(10);
		 LED1_TOGGLE();
	}
			
	  exti_interrupt_flag_clear(EXTI_15);  //���EXTI Line0�ϵ��жϱ�־λ  
}






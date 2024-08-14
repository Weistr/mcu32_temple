#include "wwdgt.h"
#include "led.h"
#include "usart.h"	

//��ʼ�����ڿ��Ź� 	
//tr   :T[6:0],������ֵ 
//wr   :W[6:0],����ֵ 
//fprer:��Ƶϵ��,��Χ:WWDGT_CFG_PSC_DIV1 ~WWDGT_CFG_PSC_DIV8
//Fwwdg=PCLK1/(4096*2^fprer). һ��PCLK1=60Mhz
void WWDGT_Init(uint8_t tr,uint8_t wr,uint32_t fprer)
{
  rcu_periph_clock_enable(RCU_WWDGT);         //ʹ��WWDGTʱ��
	
	wwdgt_config(tr, wr, fprer);                //����WWDGT������ֵ������ֵ��Ԥ��Ƶֵ ;
	
	wwdgt_enable();                             //ʹ�ܴ��ڿ��Ź���ʱ��
	
	wwdgt_flag_clear();                         //���WWDGT��ǰ�����жϱ�־λ״̬
	
	wwdgt_interrupt_enable();                   //ʹ�ܴ��ڿ��Ź���ǰ�����ж�
	
	nvic_irq_enable(WWDGT_IRQn, 2, 3);          //��ռ���ȼ�2����Ӧ���ȼ�Ϊ3
}
   
//���ڿ��Ź��жϷ�����
void WWDGT_IRQHandler(void)
{ 
  wwdgt_counter_update(127);                  //���´��ڿ��Ź�ֵ
	
	wwdgt_flag_clear();                         //���WWDGT��ǰ�����жϱ�־λ״̬
	
	LED1_TOGGLE();                              //LED1��˸
}


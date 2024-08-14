#include "timer.h"
#include "led.h"

//ͨ�ö�ʱ��2�жϳ�ʼ��
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
//APB1ʱ��Ϊ60MHz��TIMER2ʱ��ѡ��ΪAPB1��2������ˣ�TIMER2ʱ��Ϊ120MHz
void TIM2_Int_Init(uint16_t arr,uint16_t psc)
{
	timer_parameter_struct timer_initpara;               //timer_initpara���ڴ�Ŷ�ʱ���Ĳ���

  //ʹ��RCU���ʱ�� 
  rcu_periph_clock_enable(RCU_TIMER2);                 //ʹ��TIMER2��ʱ��

  //��λTIMER2
  timer_deinit(TIMER2);                                //��λTIMER2
  timer_struct_para_init(&timer_initpara);             //��ʼ��timer_initparaΪĬ��ֵ

  //����TIMER2
  timer_initpara.prescaler         = psc;              //����Ԥ��Ƶֵ
  timer_initpara.counterdirection  = TIMER_COUNTER_UP; //�������ϼ���ģʽ
  timer_initpara.period            = arr;              //�����Զ���װ��ֵ
  timer_initpara.clockdivision     = TIMER_CKDIV_DIV1; //����ʱ�ӷ�Ƶ����
  timer_init(TIMER2, &timer_initpara);                 //���ݲ�����ʼ����ʱ��

  //ʹ�ܶ�ʱ�������ж�
  timer_interrupt_enable(TIMER2, TIMER_INT_UP);        //ʹ�ܶ�ʱ���ĸ����ж�
  nvic_irq_enable(TIMER2_IRQn, 1, 3);                  //����NVIC�������ȼ�����ռ���ȼ�1����Ӧ���ȼ�3
  timer_enable(TIMER2);                                //ʹ�ܶ�ʱ��TIMER2
}

//��ʱ��2�жϷ������
void TIMER2_IRQHandler(void)
{
  if(timer_interrupt_flag_get(TIMER2, TIMER_INT_FLAG_UP) == SET)   //�ж϶�ʱ�������ж��Ƿ���
  {
		LED1_TOGGLE();                                                 //LED1��ת
    timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);         //�����ʱ�������жϱ�־
  }
}















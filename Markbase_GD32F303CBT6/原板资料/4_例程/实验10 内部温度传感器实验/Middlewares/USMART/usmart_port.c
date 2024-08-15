/************************************************ 
* USMART ���ڵ������						  
************************************************/	

#include "usmart.h"
#include "usmart_port.h"



//��ȡ����������(�ַ���)
//USMARTͨ�������ú������ص��ַ����Ի�ȡ����������������Ϣ
//����ֵ:0,û�н��յ�����
//����,�������׵�ַ(������0)
char *usmart_get_input_string(void)
{
    uint8_t len;
    char *pbuf = 0;

    if (USART_RX_STA & 0x8000)        //���ڽ�����ɣ� 
    {
        len = USART_RX_STA & 0x3fff;  //�õ��˴ν��յ������ݳ��� 
        USART_RX_BUF[len] = '\0';     //��ĩβ���������. 
        pbuf = (char*)USART_RX_BUF;
        USART_RX_STA = 0;             //������һ�ν��� 
    }

    return pbuf;
}

//���ʹ���˶�ʱ��ɨ��, ����Ҫ�������º��� 
#if USMART_ENTIMX_SCAN==1

////////////////////////////////////////////////////////////////////////////////////////
//��ֲע��:��������GD32Ϊ��,���Ҫ��ֲ������mcu,������Ӧ�޸�.
//usmart_reset_runtime,�����������ʱ��,��ͬ��ʱ���ļ����Ĵ����Լ���־λһ������.��������װ��ֵΪ���,������޶ȵ��ӳ���ʱʱ��.
//usmart_get_runtime,��ȡ��������ʱ��,ͨ����ȡCNTֵ��ȡ,����usmart��ͨ���жϵ��õĺ���,���Զ�ʱ���жϲ�����Ч,��ʱ����޶�
//ֻ��ͳ��2��CNT��ֵ,Ҳ���������+���һ��,���������2��,û������,���������ʱ,������:2*������CNT*0.1ms.
//������:TIM4_IRQHandler��Timer4_Init,��Ҫ����MCU�ص������޸�.ȷ������������Ƶ��Ϊ:10Khz����.����,��ʱ����Ҫ�����Զ���װ�ع���!!


//��λruntime
//��Ҫ��������ֲ����MCU�Ķ�ʱ�����������޸�
void usmart_reset_runtime(void)
{
     timer_interrupt_flag_clear(TIMER3, TIMER_INT_FLAG_UP); //����жϱ�־λ 
     timer_autoreload_value_config(TIMER3,0XFFFF);          //����װ��ֵ���õ����
     timer_counter_value_config(TIMER3, 0);                 //��ն�ʱ���ļ�����
     usmart_dev.runtime=0;	
}

//���runtimeʱ��
//����ֵ:ִ��ʱ��,��λ:0.1ms,�����ʱʱ��Ϊ��ʱ��CNTֵ��2��*0.1ms
//��Ҫ��������ֲ����MCU�Ķ�ʱ�����������޸�
uint32_t usmart_get_runtime(void)
{
	if(timer_interrupt_flag_get(TIMER3, TIMER_INT_FLAG_UP) == SET)//�������ڼ�,�����˶�ʱ�����
	{
		usmart_dev.runtime+=0XFFFF;
	}
	usmart_dev.runtime+=timer_counter_read(TIMER3);
	return usmart_dev.runtime;		//���ؼ���ֵ
}  

//��ʱ��3�жϷ������	 
void TIMER3_IRQHandler(void)
{ 		    		  			       
    if(timer_interrupt_flag_get(TIMER3, TIMER_INT_FLAG_UP) == SET)//����ж�
    {
        usmart_dev.scan();	                          //ִ��usmartɨ��
        timer_counter_value_config(TIMER3, 0);        //��ն�ʱ���ļ�����
        timer_autoreload_value_config(TIMER3, 1000);  //�ָ�ԭ��������
    }
    timer_interrupt_flag_clear(TIMER3, TIMER_INT_FLAG_UP);//����жϱ�־λ
}

//��ʱ����ʼ������
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//ʹ�ܶ�ʱ��3,ʹ���ж�.
void Timer3_Init(uint16_t arr,uint16_t psc)
{ 
	timer_parameter_struct timer_initpara;               //timer_initpara���ڴ�Ŷ�ʱ���Ĳ���

  //ʹ��RCU���ʱ�� 
  rcu_periph_clock_enable(RCU_TIMER3);                 //ʹ��TIMER3��ʱ��

  //��λTIMER3
  timer_deinit(TIMER3);                                //��λTIMER3
  timer_struct_para_init(&timer_initpara);             //��ʼ��timer_initparaΪĬ��ֵ

  //����TIMER3
  timer_initpara.prescaler         = psc;              //����Ԥ��Ƶֵ
  timer_initpara.counterdirection  = TIMER_COUNTER_UP; //�������ϼ���ģʽ
  timer_initpara.period            = arr;              //�����Զ���װ��ֵ
  timer_initpara.clockdivision     = TIMER_CKDIV_DIV1; //����ʱ�ӷ�Ƶ����
  timer_init(TIMER3, &timer_initpara);                 //���ݲ�����ʼ����ʱ��

  //ʹ�ܶ�ʱ�������ж�
  timer_interrupt_enable(TIMER3, TIMER_INT_UP);        //ʹ�ܶ�ʱ���ĸ����ж�
  nvic_irq_enable(TIMER3_IRQn, 3, 3);                  //����NVIC�������ȼ�����ռ���ȼ�3����Ӧ���ȼ�3
  timer_enable(TIMER3);                                //ʹ�ܶ�ʱ��TIMER3			 
}
 

#endif

















#include "sys.h"
#include "usart.h"


/* ���ʹ��os,����������ͷ�ļ�����. */
#if SYS_SUPPORT_OS
#include "includes.h" /* os ʹ�� */
#endif

/******************************************************************************************/
/* �������´���, ֧��printf����, ������Ҫѡ��use MicroLIB */

#if 1

#if (__ARMCC_VERSION >= 6010050)            /* ʹ��AC6������ʱ */
__asm(".global __use_no_semihosting\n\t");  /* ������ʹ�ð�����ģʽ */
__asm(".global __ARM_use_no_argv \n\t");    /* AC6����Ҫ����main����Ϊ�޲�����ʽ�����򲿷����̿��ܳ��ְ�����ģʽ */

#else
/* ʹ��AC5������ʱ, Ҫ�����ﶨ��__FILE �� ��ʹ�ð�����ģʽ */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

#endif

/* ��ʹ�ð�����ģʽ��������Ҫ�ض���_ttywrch\_sys_exit\_sys_command_string����,��ͬʱ����AC6��AC5ģʽ */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* ����_sys_exit()�Ա���ʹ�ð�����ģʽ */
void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}


/* FILE �� stdio.h���涨��. */
FILE __stdout;

/* MDK����Ҫ�ض���fputc����, printf�������ջ�ͨ������fputc����ַ��������� */
int fputc(int ch, FILE *f)
{
	while(RESET == usart_flag_get(USART0, USART_FLAG_TC));       /* �ȴ���һ���ַ�������� */
	
	usart_data_transmit(USART0, (uint8_t)ch);                    /* ��Ҫ���͵��ַ� ch д�뵽DR�Ĵ��� */  
  return ch;
}
#endif
/******************************************************************************************/

#if USART_EN_RX /*���ʹ���˽���*/

/* ���ջ���, ���USART_REC_LEN���ֽ�. */
uint8_t USART_RX_BUF[USART_REC_LEN];

/*  ����״̬
 *  bit15��      ������ɱ�־
 *  bit14��      ���յ�0x0d
 *  bit13~0��    ���յ�����Ч�ֽ���Ŀ
*/
uint16_t USART_RX_STA = 0;



//����0��ʼ������
//bound: ������, �����Լ���Ҫ���ò�����ֵ
void usart_init(uint32_t bound)
{
	//ʹ��GPIOʱ�Ӻ͸���ʱ��
  rcu_periph_clock_enable(RCU_GPIOA);     //ʹ��GPIOAʱ��
  rcu_periph_clock_enable(RCU_AF);        //ʹ�ܸ���ʱ��
  rcu_periph_clock_enable(RCU_USART0);    //ʹ�ܴ���ʱ��

  //����TX��GPIO
  gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

  //����RX��GPIO
  gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);

  //����USART�Ĳ���
  usart_deinit(USART0);                                 //RCU���ûָ�Ĭ��ֵ
  usart_baudrate_set(USART0, bound);                    //���ò�����
  usart_stop_bit_set(USART0, USART_STB_1BIT);           //һ��ֹͣλ
  usart_word_length_set(USART0, USART_WL_8BIT);         //�ֳ�Ϊ8λ���ݸ�ʽ
  usart_parity_config(USART0, USART_PM_NONE);           //����żУ��λ
  usart_receive_config(USART0, USART_RECEIVE_ENABLE);   //ʹ�ܽ���
  usart_transmit_config(USART0, USART_TRANSMIT_ENABLE); //ʹ�ܷ���
  usart_interrupt_enable(USART0, USART_INT_RBNE);       //ʹ�ܽ��ջ������ǿ��ж�
  //����NVIC���������ж����ȼ�
  nvic_irq_enable(USART0_IRQn, 3, 3);                   //��ռ���ȼ�3�������ȼ�3

  //ʹ�ܴ���
  usart_enable(USART0);	
}

void USART0_IRQHandler(void)
{
	uint8_t Res;
#if SYSTEM_SUPPORT_OS 		                              //���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntEnter();    
#endif
		if(usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
		{
		Res =usart_data_receive(USART0);	//��ȡ���յ�������
		
		if((USART_RX_STA&0x8000)==0)//����δ���
			{
			if(USART_RX_STA&0x4000)//���յ���0x0d
				{
				if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
				else USART_RX_STA|=0x8000;	//��������� 
				}
			else //��û�յ�0X0D
				{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
					{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
					}		 
				}
			}   		 
     } 
#if SYSTEM_SUPPORT_OS 	//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntExit();  											 
#endif
}
#endif

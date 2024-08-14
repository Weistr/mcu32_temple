#include "fwdgt.h"


//��ʼ���������Ź�
//prer:��Ƶ��:FWDGT_PSC_DIV4~FWDGT_PSC_DIV256
//rlr:�Զ���װ��ֵ,0~0XFFF.
//ʱ�����(���):Tout=((4*2^prer)*rlr)/40 (ms).
void FWDGT_Init(uint8_t prer,uint16_t rlr)
{
	  //���ö������Ź���ʱ��
  fwdgt_write_enable();          //ʹ�ܶԼĴ�����д����
  
  fwdgt_config(rlr, prer);       //������װ��ֵ��Ԥ��Ƶֵ
    
  fwdgt_counter_reload();        //��FWDGT_RLD�Ĵ�����ֵ��װ��FWDGT������
  
  fwdgt_enable();                //ʹ�ܶ������Ź���ʱ��

}
   
//ι�������Ź�
void FWDGT_Feed(void)
{   
  fwdgt_counter_reload();        //ι��
}

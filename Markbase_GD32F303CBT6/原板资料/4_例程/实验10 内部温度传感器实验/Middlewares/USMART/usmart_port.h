/************************************************ 
* USMART ���ڵ������						  
************************************************/	
 
#ifndef __USMART_PORT_H
#define __USMART_PORT_H

#include "sys.h"
#include "usart.h"




/******************************************************************************************/
/* �û����ò��� */


#define MAX_FNAME_LEN           30      //��������󳤶ȣ�Ӧ������Ϊ��С����������ĳ��ȡ� 
#define MAX_PARM                10      //���Ϊ10������ ,�޸Ĵ˲���,�����޸�usmart_exe��֮��Ӧ. 
#define PARM_LEN                200     //���в���֮�͵ĳ��Ȳ�����PARM_LEN���ֽ�,ע�⴮�ڽ��ղ���Ҫ��֮��Ӧ(��С��PARM_LEN) 


#define USMART_ENTIMX_SCAN      1       //ʹ��TIM�Ķ�ʱ�ж���ɨ��SCAN����,�������Ϊ0,��Ҫ�Լ�ʵ�ָ�һ��ʱ��ɨ��һ��scan����.
                                        //ע��:���Ҫ��runtimeͳ�ƹ���,��������USMART_ENTIMX_SCANΪ1!!!!
                                         

#define USMART_USE_HELP         1       //ʹ�ð�������ֵ��Ϊ0�����Խ�ʡ��700���ֽڣ����ǽ������޷���ʾ������Ϣ�� 
#define USMART_USE_WRFUNS       1       //ʹ�ö�д����,ʹ������,���Զ�ȡ�κε�ַ��ֵ,������д�Ĵ�����ֵ. 

#define USMART_PRINTF           printf  //����printf��� 




/* ���û�ж���uint32_t,���� */
#ifndef uint32_t
typedef unsigned           char uint8_t;
typedef unsigned short     int  uint16_t;
typedef unsigned           int  uint32_t;
#endif



char *usmart_get_input_string(void);           //��ȡ���������� 
void usmart_reset_runtime(void);               //��λ����ʱ�� 
uint32_t usmart_get_runtime(void);             //��ȡ����ʱ�� 
void Timer3_Init(uint16_t arr,uint16_t psc);   //��ʼ����ʱ�� 

#endif




























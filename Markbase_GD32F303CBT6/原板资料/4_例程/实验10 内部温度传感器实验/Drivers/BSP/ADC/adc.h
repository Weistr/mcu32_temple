#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
							  	 	    
#define ADC_CHX_TEMPSENSOR  16 	  //ADC�ڲ��¶ȴ�����ͨ��		 	    
	   									   
void Adc_Init(void); 				      //��ʼ��ADC
uint16_t Get_Adc(uint8_t ch) ; 		//���ĳ��ͨ��ת����Ľ�� 
uint16_t Get_Adc_Average(uint8_t ch,uint8_t times); //�õ�ĳ��ͨ��times�β�����ƽ��ֵ 	  
void adc_temperature_init(void);  //ADC�¶Ȳɼ���ʼ������
short Get_Temprate(void);         //��ȡ�ڲ��¶ȴ������¶�ֵ 

#endif 
















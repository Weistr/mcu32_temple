#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"
							  	 	    
#define ADC_CHX_TEMPSENSOR  16 	  //ADC内部温度传感器通道		 	    
	   									   
void Adc_Init(void); 				      //初始化ADC
uint16_t Get_Adc(uint8_t ch) ; 		//获得某个通道转换后的结果 
uint16_t Get_Adc_Average(uint8_t ch,uint8_t times); //得到某个通道times次采样的平均值 	  
void adc_temperature_init(void);  //ADC温度采集初始化函数
short Get_Temprate(void);         //获取内部温度传感器温度值 

#endif 
















#ifndef __RTC_H
#define __RTC_H	  
#include "sys.h"

												    
//ʱ��ṹ��,������������ʱ�������Ϣ
typedef struct 
{
	uint8_t hour;       //ʱ
	uint8_t min;        //��
	uint8_t sec;			  //��
	//������������
	uint16_t year;      //��
	uint8_t  month;     //��
	uint8_t  date;      //��
	uint8_t  week;	    //��
}_calendar_obj;					 
extern _calendar_obj calendar;				//ʱ��ṹ��

//��̬���� 
static uint8_t Is_Leap_Year(uint16_t year);					//�жϵ�ǰ����ǲ������� 
static long rtc_date2sec(uint16_t syear, uint8_t smon, uint8_t sday, uint8_t hour, uint8_t min, uint8_t sec); //��������ʱ����ת����������

uint8_t RTC_Init(void);        					  //��ʼ��RTC
uint8_t RTC_Set_Time(uint16_t syear,uint8_t smon,uint8_t sday,uint8_t hour,uint8_t min,uint8_t sec);	//����ʱ��	
uint8_t RTC_Alarm_Set(uint16_t syear,uint8_t smon,uint8_t sday,uint8_t hour,uint8_t min,uint8_t sec);	//��������	ʱ��
void RTC_Get_Time(void);         					//��ȡʱ��   
uint8_t RTC_Get_Week(uint16_t year,uint8_t month,uint8_t day); //���������ջ�ȡ���ڼ� 

#endif




















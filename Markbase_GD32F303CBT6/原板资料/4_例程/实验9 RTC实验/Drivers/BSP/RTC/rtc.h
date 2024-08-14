#ifndef __RTC_H
#define __RTC_H	  
#include "sys.h"

												    
//时间结构体,包括年月日周时分秒等信息
typedef struct 
{
	uint8_t hour;       //时
	uint8_t min;        //分
	uint8_t sec;			  //秒
	//公历年月日周
	uint16_t year;      //年
	uint8_t  month;     //月
	uint8_t  date;      //日
	uint8_t  week;	    //周
}_calendar_obj;					 
extern _calendar_obj calendar;				//时间结构体

//静态函数 
static uint8_t Is_Leap_Year(uint16_t year);					//判断当前年份是不是闰年 
static long rtc_date2sec(uint16_t syear, uint8_t smon, uint8_t sday, uint8_t hour, uint8_t min, uint8_t sec); //将年月日时分秒转换成秒钟数

uint8_t RTC_Init(void);        					  //初始化RTC
uint8_t RTC_Set_Time(uint16_t syear,uint8_t smon,uint8_t sday,uint8_t hour,uint8_t min,uint8_t sec);	//设置时间	
uint8_t RTC_Alarm_Set(uint16_t syear,uint8_t smon,uint8_t sday,uint8_t hour,uint8_t min,uint8_t sec);	//设置闹钟	时间
void RTC_Get_Time(void);         					//获取时间   
uint8_t RTC_Get_Week(uint16_t year,uint8_t month,uint8_t day); //根据年月日获取星期几 

#endif




















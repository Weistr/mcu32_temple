#include "delay.h"
#include "usart.h"
#include "led.h"
#include "rtc.h" 		    


_calendar_obj calendar; //ʱ�ӽṹ�� 

//ʵʱʱ������
//��ʼ��RTCʱ��,ͬʱ���ʱ���Ƿ�������""
//Ĭ�ϳ���ʹ��CK_LXTAL,��CK_LXTAL����ʧ�ܺ�,�л�ΪCK_IRC40K.
//ͨ��BKP�Ĵ���0��ֵ,�����ж�RTCʹ�õ���CK_LXTAL/CK_IRC40K:
//��BKP0==0X5050ʱ,ʹ�õ���CK_LXTAL
//��BKP0==0X5051ʱ,ʹ�õ���CK_IRC40K
//ע��:�л�CK_LXTAL/CK_IRC40K������ʱ��/���ڶ�ʧ,�л�������������.
//����0:����;1,�����ʼ��ģʽʧ��
uint8_t RTC_Init(void)
{
		//����ǲ��ǵ�һ������ʱ�� 
		uint16_t bkpflag = 0;
		uint16_t retry = 200;
		uint32_t tempreg = 0;
		uint32_t clockfreq = 0;
	
		rcu_periph_clock_enable(RCU_PMU);        //ʹ��PMUʱ��
		rcu_periph_clock_enable(RCU_BKPI);       //ʹ��BKPIʱ��
		pmu_backup_write_enable();               //������дʹ��
	
		bkpflag = bkp_read_data(BKP_DATA_0);     //��ȡBKP0��ֵ

		if (bkpflag != 0X6060)                   //֮ǰʹ�õĲ���CK_LXTAL
		{
			bkp_deinit();                          //��λ������
			rcu_osci_on(RCU_LXTAL);                //ʹ���ⲿ����ʱ��
		
			if ((rcu_osci_stab_wait(RCU_LXTAL) == ERROR))              //����CK_LXTALʧ��?
			{ 
					rcu_osci_on(RCU_IRC40K);           //ʹ��CK_IRC40K 
					if ((rcu_osci_stab_wait(RCU_IRC40K) == SUCCESS))       //�ȴ�CK_IRC40K׼����
					{
							rcu_rtc_clock_config(RCU_RTCSRC_IRC40K); //ѡ��CK_IRC40Kʱ����ΪRTC��ʱ��Դ
							bkp_write_data(BKP_DATA_0, 0x6061);      //����Ѿ���ʼ������,ʹ��CK_IRC40K
							clockfreq = 40000 - 1;  //CK_IRC40KƵ��Լ40Khz
					}
			}	
			else
			{ 
				 rcu_rtc_clock_config(RCU_RTCSRC_LXTAL);       //ѡ��RCU_LXTALʱ����ΪRTC��ʱ��Դ
				 bkp_write_data(BKP_DATA_0, 0x6060);           //����Ѿ���ʼ������,ʹ��RCU_LXTAL
				 clockfreq = 32768 - 1;       //RCU_LXTALƵ��Ϊ32.768Khz
			}
			rcu_periph_clock_enable(RCU_RTC);      //ʹ��RTCʱ��
			
			rtc_lwoff_wait();                      //�ȴ����һ�ζ�RTC�Ĵ�����д�������		
			rtc_register_sync_wait();              //�ȴ�RTC�Ĵ�����APBʱ��ͬ��		
			rtc_interrupt_enable(RTC_INT_SECOND);  //ʹ��RTC���ж�
			rtc_lwoff_wait();                      //�ȴ����һ�ζ�RTC�Ĵ�����д�������
			
			rtc_prescaler_set(clockfreq);          //Ƶ��Ϊ1Hz		
			rtc_lwoff_wait();                      //�ȴ����һ�ζ�RTC�Ĵ�����д�������
		
			if (bkpflag != 0X6061)          //BKP0�����ݼȲ���0X5050,Ҳ����0X5051,˵���ǵ�һ������,��Ҫ����ʱ������
			{
					RTC_Set_Time(2023, 8, 28, 23, 58, 55);   //����ʱ�� 
			}		
		}
		else   //RTC������ʱ 
		{
			retry = 30;     //���⿨�� 
			while (((RESET == (RTC_CTL & RTC_CTL_RSYNF))&& retry))   //�ȴ�RTC�Ĵ�����APBʱ��ͬ��
			{
					delay_ms(5);
					retry--;
			}
			retry = 100;    //���RCU_LXTAL/CK_IRC40K�Ƿ��������� 
			tempreg = rtc_divider_get();            //��ȡRTC��Ƶֵ 
			while (retry)
			{
					delay_ms(5);
					retry--;			
					if (tempreg != rtc_divider_get())   //�Ա�RTC��Ƶֵ��tempreg,����в���,���˳� 
					{
							break;                          //��Ƶֵ!= tempreg, ��RTC�ڼ���,˵������û���� 
					}
			}
			if (retry == 0)
			{
					bkp_write_data(BKP_DATA_0, 0xFFFF);    //��Ǵ����ֵ 
					bkp_deinit();                          //��λ������
					return 1;                              //��ʼ��ʧ�� 
			}
			else
			{
					rtc_interrupt_enable(RTC_INT_SECOND);  //ʹ��RTC���ж�
					rtc_lwoff_wait();                      //�ȴ����һ�ζ�RTC�Ĵ�����д�������
			}	
		}
		nvic_irq_enable(RTC_IRQn, 0, 0);             //�����ж����ȼ�
		RTC_Get_Time();      //����ʱ�� 
		return 0;            //ok
}


//RTCʱ���ж�
//�����жϷ�����,˳���������ӱ�־  	 
void RTC_IRQHandler(void)
{		 
		if(RESET != rtc_flag_get(RTC_FLAG_SECOND))  //���ж�
		{
			rtc_flag_clear(RTC_FLAG_SECOND); 		      //������жϱ�־
			RTC_Get_Time();				     //����ʱ��   
			 printf("sec:%d\r\n", calendar.sec);
		}		

		if(RESET != rtc_flag_get(RTC_FLAG_ALARM))  //�����ж�
		{
			rtc_flag_clear(RTC_FLAG_ALARM); 	       //��������жϱ�־   
			printf("ALARM A!\r\n");
		}	
		rtc_flag_clear(RTC_CTL_OVIF); 		//������  	  
		rtc_lwoff_wait();                 //�ȴ����һ�ζ�RTC�Ĵ�����д�������	
}

//�ж��Ƿ������꺯��
//�·�   1  2  3  4  5  6  7  8  9  10 11 12
//����   31 29 31 30 31 30 31 31 30 31 30 31
//������ 31 28 31 30 31 30 31 31 30 31 30 31
//year:���
//����ֵ:������ǲ�������.1,��.0,����
static uint8_t Is_Leap_Year(uint16_t year)
{			  
		if(year%4==0) //�����ܱ�4����
		{ 
			if(year%100==0) 
			{ 
				if(year%400==0)return 1;//�����00��β,��Ҫ�ܱ�400���� 	   
				else return 0;   
			}else return 1;   
		}else return 0;	
}	


//�·����ݱ�											 
uint8_t const table_week[12]={0,3,3,6,1,4,6,2,5,0,3,5}; //���������ݱ�	  
//ƽ����·����ڱ�
const uint8_t mon_table[12]={31,28,31,30,31,30,31,31,30,31,30,31};

//����ʱ��, ����������ʱ����
//��1970��1��1��Ϊ��׼, �����ۼ�ʱ��
//�Ϸ���ݷ�ΧΪ: 1970 ~ 2105��
//syear,smon,sday,hour,min,sec��������ʱ����
//����ֵ�����ý����0���ɹ���1��ʧ�ܡ�
uint8_t RTC_Set_Time(uint16_t syear,uint8_t smon,uint8_t sday,uint8_t hour,uint8_t min,uint8_t sec)
{
		uint32_t seccount=0;
		
		if(syear<1970||syear>2099)return 1;	   

		seccount = rtc_date2sec(syear, smon, sday, hour, min, sec); //��������ʱ����ת������������ 

		//����ʱ��
		rcu_periph_clock_enable(RCU_PMU);        //ʹ��PMUʱ��
		rcu_periph_clock_enable(RCU_BKPI);       //ʹ��BKPIʱ��
		pmu_backup_write_enable();               //������дʹ��
		//���������Ǳ����!
		
		rtc_counter_set(seccount);               //����ʱ��
		
		rtc_lwoff_wait();                        //�ȴ����һ�ζ�RTC�Ĵ�����д�������	
		
		RTC_Get_Time();                          //������֮�����һ��ʱ�� 	
		
		return 0;	    
}
//��������, ���嵽������ʱ����
//��1970��1��1��Ϊ��׼, �����ۼ�ʱ��
//�Ϸ���ݷ�ΧΪ: 1970 ~ 2105��
//syear,smon,sday,hour,min,sec�����ӵ�������ʱ����   
//����ֵ:0,�ɹ�;����:�������.
uint8_t RTC_Alarm_Set(uint16_t syear,uint8_t smon,uint8_t sday,uint8_t hour,uint8_t min,uint8_t sec)
{
		uint32_t seccount=0;
		
		if(syear<1970||syear>2099)return 1;	   

		seccount = rtc_date2sec(syear, smon, sday, hour, min, sec); //��������ʱ����ת������������ 

		//����ʱ��
		rcu_periph_clock_enable(RCU_PMU);        //ʹ��PMUʱ��
		rcu_periph_clock_enable(RCU_BKPI);       //ʹ��BKPIʱ��
		pmu_backup_write_enable();               //������дʹ��
		//���������Ǳ����!

		rtc_alarm_config(seccount);              //��������ʱ��
		
		rtc_lwoff_wait();                        //�ȴ����һ�ζ�RTC�Ĵ�����д�������	
			
		return 0;	    
}
//�õ���ǰ��ʱ��
//�ú�����ֱ�ӷ���ʱ��, ʱ�����ݱ�����calendar�ṹ������
void RTC_Get_Time(void)
{
		static uint16_t daycnt=0;
		uint32_t seccount=0; 
		uint32_t temp=0;
		uint16_t temp1=0;	  
		
		seccount = rtc_counter_get();	 //��ȡRTC��������ֵ(������)

		temp=seccount/86400;   //�õ�����(��������Ӧ��)
		
		if(daycnt!=temp)//����һ����
		{	  
			daycnt=temp;
			temp1=1970;	  //��1970�꿪ʼ
			while(temp>=365)
			{				 
				if(Is_Leap_Year(temp1))  //������
				{
					if(temp>=366)temp-=366;//�����������
					else break;  
				}
				else temp-=365;	    //ƽ�� 
				temp1++;  
			}   
			calendar.year=temp1;//�õ����
			temp1=0;
			while(temp>=28)//������һ����
			{
				if(Is_Leap_Year(calendar.year)&&temp1==1)//�����ǲ�������/2�·�
				{
					if(temp>=29)temp-=29;//�����������
					else break; 
				}
				else 
				{
					if(temp>=mon_table[temp1])temp-=mon_table[temp1];//ƽ��
					else break;
				}
				temp1++;  
			}
			calendar.month=temp1+1;	  //�õ��·�
			calendar.date=temp+1;  	  //�õ����� 
		}
		temp=seccount%86400;     		//�õ�������   	   
		calendar.hour=temp/3600;     	//Сʱ
		calendar.min=(temp%3600)/60; 	//����	
		calendar.sec=(temp%3600)%60; 	//����
		calendar.week=RTC_Get_Week(calendar.year,calendar.month,calendar.date);//��ȡ����   
}	 
//������������ڼ�
//��������:���빫�����ڵõ�����(ֻ����1901-2099��)
//year,month,day������������ 
//����ֵ�����ں�; 0, ������; 1 ~ 6: ����һ ~ ������																						 
uint8_t RTC_Get_Week(uint16_t year,uint8_t month,uint8_t day)
{	
//	uint16_t temp2;
//	uint8_t yearH,yearL;
//	
//	yearH=year/100;	yearL=year%100; 
//	// ���Ϊ21����,�������100  
//	if (yearH>19)yearL+=100;
//	// ����������ֻ��1900��֮���  
//	temp2=yearL+yearL/4;
//	temp2=temp2%7; 
//	temp2=temp2+day+table_week[month-1];
//	if (yearL%4==0&&month<3)temp2--;
//	return(temp2%7);
	
		uint8_t week = 0;

		if (month < 3)
		{
				month += 12;
				--year;
		}

		week = (day + 1 + 2 * month + 3 * (month + 1) / 5 + year + (year >> 2) - year / 100 + year / 400) % 7;
		return week;
}

//��������ʱ����ת����������
//��1970��1��1��Ϊ��׼, 1970��1��1��, 0ʱ0��0��, ��ʾ��0����
//����ʾ��2105��, ��Ϊuint32_t����ʾ136���������(����������)!
//syear,smon,sday,hour,min,sec�����ӵ�������ʱ����   
//����ֵ:ת�����������.
static long rtc_date2sec(uint16_t syear, uint8_t smon, uint8_t sday, uint8_t hour, uint8_t min, uint8_t sec)
{
		uint16_t t;
		uint32_t seccount=0;
		for(t=1970;t<syear;t++)	//��������ݵ��������
		{
			if(Is_Leap_Year(t))seccount+=31622400;//�����������
			else seccount+=31536000;			  //ƽ���������
		}
		smon-=1;
		for(t=0;t<smon;t++)	   //��ǰ���·ݵ����������
		{
			seccount+=(uint32_t)mon_table[t]*86400;//�·����������
			if(Is_Leap_Year(syear)&&t==1)seccount+=86400;//����2�·�����һ���������	   
		}
		seccount+=(uint32_t)(sday-1)*86400;//��ǰ�����ڵ���������� 
		seccount+=(uint32_t)hour*3600;//Сʱ������
		seccount+=(uint32_t)min*60;	 //����������
		seccount+=sec;//�������Ӽ���ȥ

		return seccount;
}















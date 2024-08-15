#include "delay.h"
#include "usart.h"
#include "led.h"
#include "rtc.h" 		    


_calendar_obj calendar; //时钟结构体 

//实时时钟配置
//初始化RTC时钟,同时检测时钟是否工作正常""
//默认尝试使用CK_LXTAL,当CK_LXTAL启动失败后,切换为CK_IRC40K.
//通过BKP寄存器0的值,可以判断RTC使用的是CK_LXTAL/CK_IRC40K:
//当BKP0==0X5050时,使用的是CK_LXTAL
//当BKP0==0X5051时,使用的是CK_IRC40K
//注意:切换CK_LXTAL/CK_IRC40K将导致时间/日期丢失,切换后需重新设置.
//返回0:正常;1,进入初始化模式失败
uint8_t RTC_Init(void)
{
		//检查是不是第一次配置时钟 
		uint16_t bkpflag = 0;
		uint16_t retry = 200;
		uint32_t tempreg = 0;
		uint32_t clockfreq = 0;
	
		rcu_periph_clock_enable(RCU_PMU);        //使能PMU时钟
		rcu_periph_clock_enable(RCU_BKPI);       //使能BKPI时钟
		pmu_backup_write_enable();               //备份域写使能
	
		bkpflag = bkp_read_data(BKP_DATA_0);     //读取BKP0的值

		if (bkpflag != 0X6060)                   //之前使用的不是CK_LXTAL
		{
			bkp_deinit();                          //复位备份域
			rcu_osci_on(RCU_LXTAL);                //使能外部低速时钟
		
			if ((rcu_osci_stab_wait(RCU_LXTAL) == ERROR))              //开启CK_LXTAL失败?
			{ 
					rcu_osci_on(RCU_IRC40K);           //使能CK_IRC40K 
					if ((rcu_osci_stab_wait(RCU_IRC40K) == SUCCESS))       //等待CK_IRC40K准备好
					{
							rcu_rtc_clock_config(RCU_RTCSRC_IRC40K); //选择CK_IRC40K时钟作为RTC的时钟源
							bkp_write_data(BKP_DATA_0, 0x6061);      //标记已经初始化过了,使用CK_IRC40K
							clockfreq = 40000 - 1;  //CK_IRC40K频率约40Khz
					}
			}	
			else
			{ 
				 rcu_rtc_clock_config(RCU_RTCSRC_LXTAL);       //选择RCU_LXTAL时钟作为RTC的时钟源
				 bkp_write_data(BKP_DATA_0, 0x6060);           //标记已经初始化过了,使用RCU_LXTAL
				 clockfreq = 32768 - 1;       //RCU_LXTAL频率为32.768Khz
			}
			rcu_periph_clock_enable(RCU_RTC);      //使能RTC时钟
			
			rtc_lwoff_wait();                      //等待最近一次对RTC寄存器的写操作完成		
			rtc_register_sync_wait();              //等待RTC寄存器与APB时钟同步		
			rtc_interrupt_enable(RTC_INT_SECOND);  //使能RTC秒中断
			rtc_lwoff_wait();                      //等待最近一次对RTC寄存器的写操作完成
			
			rtc_prescaler_set(clockfreq);          //频率为1Hz		
			rtc_lwoff_wait();                      //等待最近一次对RTC寄存器的写操作完成
		
			if (bkpflag != 0X6061)          //BKP0的内容既不是0X5050,也不是0X5051,说明是第一次配置,需要设置时间日期
			{
					RTC_Set_Time(2023, 8, 28, 23, 58, 55);   //设置时间 
			}		
		}
		else   //RTC继续计时 
		{
			retry = 30;     //避免卡死 
			while (((RESET == (RTC_CTL & RTC_CTL_RSYNF))&& retry))   //等待RTC寄存器与APB时钟同步
			{
					delay_ms(5);
					retry--;
			}
			retry = 100;    //检测RCU_LXTAL/CK_IRC40K是否正常工作 
			tempreg = rtc_divider_get();            //获取RTC分频值 
			while (retry)
			{
					delay_ms(5);
					retry--;			
					if (tempreg != rtc_divider_get())   //对比RTC分频值和tempreg,如果有差异,则退出 
					{
							break;                          //分频值!= tempreg, 即RTC在计数,说明晶振没问题 
					}
			}
			if (retry == 0)
			{
					bkp_write_data(BKP_DATA_0, 0xFFFF);    //标记错误的值 
					bkp_deinit();                          //复位备份域
					return 1;                              //初始化失败 
			}
			else
			{
					rtc_interrupt_enable(RTC_INT_SECOND);  //使能RTC秒中断
					rtc_lwoff_wait();                      //等待最近一次对RTC寄存器的写操作完成
			}	
		}
		nvic_irq_enable(RTC_IRQn, 0, 0);             //设置中断优先级
		RTC_Get_Time();      //更新时间 
		return 0;            //ok
}


//RTC时钟中断
//秒钟中断服务函数,顺带处理闹钟标志  	 
void RTC_IRQHandler(void)
{		 
		if(RESET != rtc_flag_get(RTC_FLAG_SECOND))  //秒中断
		{
			rtc_flag_clear(RTC_FLAG_SECOND); 		      //清除秒中断标志
			RTC_Get_Time();				     //更新时间   
			 printf("sec:%d\r\n", calendar.sec);
		}		

		if(RESET != rtc_flag_get(RTC_FLAG_ALARM))  //闹钟中断
		{
			rtc_flag_clear(RTC_FLAG_ALARM); 	       //清除闹钟中断标志   
			printf("ALARM A!\r\n");
		}	
		rtc_flag_clear(RTC_CTL_OVIF); 		//清除溢出  	  
		rtc_lwoff_wait();                 //等待最近一次对RTC寄存器的写操作完成	
}

//判断是否是闰年函数
//月份   1  2  3  4  5  6  7  8  9  10 11 12
//闰年   31 29 31 30 31 30 31 31 30 31 30 31
//非闰年 31 28 31 30 31 30 31 31 30 31 30 31
//year:年份
//返回值:该年份是不是闰年.1,是.0,不是
static uint8_t Is_Leap_Year(uint16_t year)
{			  
		if(year%4==0) //必须能被4整除
		{ 
			if(year%100==0) 
			{ 
				if(year%400==0)return 1;//如果以00结尾,还要能被400整除 	   
				else return 0;   
			}else return 1;   
		}else return 0;	
}	


//月份数据表											 
uint8_t const table_week[12]={0,3,3,6,1,4,6,2,5,0,3,5}; //月修正数据表	  
//平年的月份日期表
const uint8_t mon_table[12]={31,28,31,30,31,30,31,31,30,31,30,31};

//设置时间, 包括年月日时分秒
//以1970年1月1日为基准, 往后累加时间
//合法年份范围为: 1970 ~ 2105年
//syear,smon,sday,hour,min,sec：年月日时分秒
//返回值：设置结果。0，成功；1，失败。
uint8_t RTC_Set_Time(uint16_t syear,uint8_t smon,uint8_t sday,uint8_t hour,uint8_t min,uint8_t sec)
{
		uint32_t seccount=0;
		
		if(syear<1970||syear>2099)return 1;	   

		seccount = rtc_date2sec(syear, smon, sday, hour, min, sec); //将年月日时分秒转换成总秒钟数 

		//设置时钟
		rcu_periph_clock_enable(RCU_PMU);        //使能PMU时钟
		rcu_periph_clock_enable(RCU_BKPI);       //使能BKPI时钟
		pmu_backup_write_enable();               //备份域写使能
		//上面三步是必须的!
		
		rtc_counter_set(seccount);               //设置时间
		
		rtc_lwoff_wait();                        //等待最近一次对RTC寄存器的写操作完成	
		
		RTC_Get_Time();                          //设置完之后更新一下时间 	
		
		return 0;	    
}
//设置闹钟, 具体到年月日时分秒
//以1970年1月1日为基准, 往后累加时间
//合法年份范围为: 1970 ~ 2105年
//syear,smon,sday,hour,min,sec：闹钟的年月日时分秒   
//返回值:0,成功;其他:错误代码.
uint8_t RTC_Alarm_Set(uint16_t syear,uint8_t smon,uint8_t sday,uint8_t hour,uint8_t min,uint8_t sec)
{
		uint32_t seccount=0;
		
		if(syear<1970||syear>2099)return 1;	   

		seccount = rtc_date2sec(syear, smon, sday, hour, min, sec); //将年月日时分秒转换成总秒钟数 

		//设置时钟
		rcu_periph_clock_enable(RCU_PMU);        //使能PMU时钟
		rcu_periph_clock_enable(RCU_BKPI);       //使能BKPI时钟
		pmu_backup_write_enable();               //备份域写使能
		//上面三步是必须的!

		rtc_alarm_config(seccount);              //设置闹钟时间
		
		rtc_lwoff_wait();                        //等待最近一次对RTC寄存器的写操作完成	
			
		return 0;	    
}
//得到当前的时间
//该函数不直接返回时间, 时间数据保存在calendar结构体里面
void RTC_Get_Time(void)
{
		static uint16_t daycnt=0;
		uint32_t seccount=0; 
		uint32_t temp=0;
		uint16_t temp1=0;	  
		
		seccount = rtc_counter_get();	 //获取RTC计数器的值(秒钟数)

		temp=seccount/86400;   //得到天数(秒钟数对应的)
		
		if(daycnt!=temp)//超过一天了
		{	  
			daycnt=temp;
			temp1=1970;	  //从1970年开始
			while(temp>=365)
			{				 
				if(Is_Leap_Year(temp1))  //是闰年
				{
					if(temp>=366)temp-=366;//闰年的秒钟数
					else break;  
				}
				else temp-=365;	    //平年 
				temp1++;  
			}   
			calendar.year=temp1;//得到年份
			temp1=0;
			while(temp>=28)//超过了一个月
			{
				if(Is_Leap_Year(calendar.year)&&temp1==1)//当年是不是闰年/2月份
				{
					if(temp>=29)temp-=29;//闰年的秒钟数
					else break; 
				}
				else 
				{
					if(temp>=mon_table[temp1])temp-=mon_table[temp1];//平年
					else break;
				}
				temp1++;  
			}
			calendar.month=temp1+1;	  //得到月份
			calendar.date=temp+1;  	  //得到日期 
		}
		temp=seccount%86400;     		//得到秒钟数   	   
		calendar.hour=temp/3600;     	//小时
		calendar.min=(temp%3600)/60; 	//分钟	
		calendar.sec=(temp%3600)%60; 	//秒钟
		calendar.week=RTC_Get_Week(calendar.year,calendar.month,calendar.date);//获取星期   
}	 
//获得现在是星期几
//功能描述:输入公历日期得到星期(只允许1901-2099年)
//year,month,day：公历年月日 
//返回值：星期号; 0, 星期天; 1 ~ 6: 星期一 ~ 星期六																						 
uint8_t RTC_Get_Week(uint16_t year,uint8_t month,uint8_t day)
{	
//	uint16_t temp2;
//	uint8_t yearH,yearL;
//	
//	yearH=year/100;	yearL=year%100; 
//	// 如果为21世纪,年份数加100  
//	if (yearH>19)yearL+=100;
//	// 所过闰年数只算1900年之后的  
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

//将年月日时分秒转换成秒钟数
//以1970年1月1日为基准, 1970年1月1日, 0时0分0秒, 表示第0秒钟
//最大表示到2105年, 因为uint32_t最大表示136年的秒钟数(不包括闰年)!
//syear,smon,sday,hour,min,sec：闹钟的年月日时分秒   
//返回值:转换后的秒钟数.
static long rtc_date2sec(uint16_t syear, uint8_t smon, uint8_t sday, uint8_t hour, uint8_t min, uint8_t sec)
{
		uint16_t t;
		uint32_t seccount=0;
		for(t=1970;t<syear;t++)	//把所有年份的秒钟相加
		{
			if(Is_Leap_Year(t))seccount+=31622400;//闰年的秒钟数
			else seccount+=31536000;			  //平年的秒钟数
		}
		smon-=1;
		for(t=0;t<smon;t++)	   //把前面月份的秒钟数相加
		{
			seccount+=(uint32_t)mon_table[t]*86400;//月份秒钟数相加
			if(Is_Leap_Year(syear)&&t==1)seccount+=86400;//闰年2月份增加一天的秒钟数	   
		}
		seccount+=(uint32_t)(sday-1)*86400;//把前面日期的秒钟数相加 
		seccount+=(uint32_t)hour*3600;//小时秒钟数
		seccount+=(uint32_t)min*60;	 //分钟秒钟数
		seccount+=sec;//最后的秒钟加上去

		return seccount;
}















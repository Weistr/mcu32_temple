#include "adc.h"
#include "delay.h"					   

	   
//初始化ADC																   
void  Adc_Init(void)
{    
		rcu_periph_clock_enable(RCU_GPIOC);                             //使能GPIOC时钟
		gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_4); //AD采集引脚(PC4)模式设置,模拟输入 
		 
		rcu_periph_clock_enable(RCU_ADC0);   //使能ADC0时钟
		 
		adc_deinit(ADC0);   //复位ADC0
		
		//ADC时钟来自APB2,频率为120Mhz
		//使用6分频,得到APB2/6 = 20Mhz的ADC时钟
		rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV6);   //配置ADC时钟
		
		adc_mode_config(ADC_MODE_FREE);   //ADC独立工作模式              
		
		adc_special_function_config(ADC0, ADC_SCAN_MODE, DISABLE);        //非扫描模式	
		adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, DISABLE);  //禁止连续模式 
		
		adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);             //数据右对齐
			
		adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);                               //常规序列使能外部触发
		adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE); //常规序列使用软件触发
		
		adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, 1);   //规则序列长度，1个转换在规则序列中，也就是只转换规则序列1 

		adc_enable(ADC0);   //使能ADC0

		adc_calibration_enable(ADC0);   //使能ADC0校准复位
	
}

//获得ADC转换后的结果
//ch:通道值 0~17
//返回值:转换结果
uint16_t Get_Adc(uint8_t ch)   
{
	  uint16_t adc_value = 0; 
	
	  adc_regular_channel_config(ADC0, 0, ch, ADC_SAMPLETIME_239POINT5);//配置ADC规则通道组,选择采样时间为239.5周期,提高采样时间可以提高精确度
	
    adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);   //软件触发使能，常规序列转换开始
	
	  while(SET != adc_flag_get(ADC0,ADC_FLAG_EOC));   //等待转换结束	
	   
	  adc_value = adc_regular_data_read(ADC0);         //读ADC规则组数据寄存器
	
    return  adc_value;   //返回最近一次ADC0的转换结果
}

//获取通道ch的转换值，取times次,然后平均 
//ch:通道编号
//times:获取次数
//返回值:通道ch的times次转换结果平均值
uint16_t Get_Adc_Average(uint8_t ch,uint8_t times)
{
		uint32_t temp_val=0;
		uint8_t t;
		for(t=0;t<times;t++)
		{
			temp_val+=Get_Adc(ch);  //获取times次数据 
			delay_ms(5);
		}
		return temp_val/times;    //返回平均值
} 
	 
//ADC内部温度传感器初始化
//注意: GD32内部温度传感器只连接在ADC0的通道16上, 其他ADC无法进行转换.
void adc_temperature_init(void)
{
    Adc_Init();                        //先初始化ADC 	
		adc_tempsensor_vrefint_enable();   //TSVREN = 1,使能内部温度传感器和Vrefint通道
}

//获取内部温度传感器温度值
//返回值:温度值(扩大了100倍,单位:℃.)
short Get_Temprate(void)
{
		uint32_t adcx;
		short result;
		double temperate;
		
		adcx=Get_Adc_Average(ADC_CHX_TEMPSENSOR,20);//读取内部温度传感器通道,20次取平均
		temperate=(float)adcx*(3.3/4096);				    //转化为电压值 
		temperate=(1.45-temperate)/0.0041+25;			  //计算温度 	 
		result=temperate*=100;							        //扩大100倍.
		return result;
}









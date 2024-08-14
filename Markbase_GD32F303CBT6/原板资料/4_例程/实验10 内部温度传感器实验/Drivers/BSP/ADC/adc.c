#include "adc.h"
#include "delay.h"					   

	   
//��ʼ��ADC																   
void  Adc_Init(void)
{    
		rcu_periph_clock_enable(RCU_GPIOC);                             //ʹ��GPIOCʱ��
		gpio_init(GPIOC, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_4); //AD�ɼ�����(PC4)ģʽ����,ģ������ 
		 
		rcu_periph_clock_enable(RCU_ADC0);   //ʹ��ADC0ʱ��
		 
		adc_deinit(ADC0);   //��λADC0
		
		//ADCʱ������APB2,Ƶ��Ϊ120Mhz
		//ʹ��6��Ƶ,�õ�APB2/6 = 20Mhz��ADCʱ��
		rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV6);   //����ADCʱ��
		
		adc_mode_config(ADC_MODE_FREE);   //ADC��������ģʽ              
		
		adc_special_function_config(ADC0, ADC_SCAN_MODE, DISABLE);        //��ɨ��ģʽ	
		adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, DISABLE);  //��ֹ����ģʽ 
		
		adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);             //�����Ҷ���
			
		adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);                               //��������ʹ���ⲿ����
		adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE); //��������ʹ���������
		
		adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, 1);   //�������г��ȣ�1��ת���ڹ��������У�Ҳ����ֻת����������1 

		adc_enable(ADC0);   //ʹ��ADC0

		adc_calibration_enable(ADC0);   //ʹ��ADC0У׼��λ
	
}

//���ADCת����Ľ��
//ch:ͨ��ֵ 0~17
//����ֵ:ת�����
uint16_t Get_Adc(uint8_t ch)   
{
	  uint16_t adc_value = 0; 
	
	  adc_regular_channel_config(ADC0, 0, ch, ADC_SAMPLETIME_239POINT5);//����ADC����ͨ����,ѡ�����ʱ��Ϊ239.5����,��߲���ʱ�������߾�ȷ��
	
    adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);   //�������ʹ�ܣ���������ת����ʼ
	
	  while(SET != adc_flag_get(ADC0,ADC_FLAG_EOC));   //�ȴ�ת������	
	   
	  adc_value = adc_regular_data_read(ADC0);         //��ADC���������ݼĴ���
	
    return  adc_value;   //�������һ��ADC0��ת�����
}

//��ȡͨ��ch��ת��ֵ��ȡtimes��,Ȼ��ƽ�� 
//ch:ͨ�����
//times:��ȡ����
//����ֵ:ͨ��ch��times��ת�����ƽ��ֵ
uint16_t Get_Adc_Average(uint8_t ch,uint8_t times)
{
		uint32_t temp_val=0;
		uint8_t t;
		for(t=0;t<times;t++)
		{
			temp_val+=Get_Adc(ch);  //��ȡtimes������ 
			delay_ms(5);
		}
		return temp_val/times;    //����ƽ��ֵ
} 
	 
//ADC�ڲ��¶ȴ�������ʼ��
//ע��: GD32�ڲ��¶ȴ�����ֻ������ADC0��ͨ��16��, ����ADC�޷�����ת��.
void adc_temperature_init(void)
{
    Adc_Init();                        //�ȳ�ʼ��ADC 	
		adc_tempsensor_vrefint_enable();   //TSVREN = 1,ʹ���ڲ��¶ȴ�������Vrefintͨ��
}

//��ȡ�ڲ��¶ȴ������¶�ֵ
//����ֵ:�¶�ֵ(������100��,��λ:��.)
short Get_Temprate(void)
{
		uint32_t adcx;
		short result;
		double temperate;
		
		adcx=Get_Adc_Average(ADC_CHX_TEMPSENSOR,20);//��ȡ�ڲ��¶ȴ�����ͨ��,20��ȡƽ��
		temperate=(float)adcx*(3.3/4096);				    //ת��Ϊ��ѹֵ 
		temperate=(1.45-temperate)/0.0041+25;			  //�����¶� 	 
		result=temperate*=100;							        //����100��.
		return result;
}









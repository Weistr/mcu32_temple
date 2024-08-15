/************************************************ 
* USMART 串口调试组件						  
************************************************/	

#include "usmart.h"
#include "usmart_port.h"



//获取输入数据流(字符串)
//USMART通过解析该函数返回的字符串以获取函数名及参数等信息
//返回值:0,没有接收到数据
//其他,数据流首地址(不能是0)
char *usmart_get_input_string(void)
{
    uint8_t len;
    char *pbuf = 0;

    if (USART_RX_STA & 0x8000)        //串口接收完成？ 
    {
        len = USART_RX_STA & 0x3fff;  //得到此次接收到的数据长度 
        USART_RX_BUF[len] = '\0';     //在末尾加入结束符. 
        pbuf = (char*)USART_RX_BUF;
        USART_RX_STA = 0;             //开启下一次接收 
    }

    return pbuf;
}

//如果使能了定时器扫描, 则需要定义如下函数 
#if USMART_ENTIMX_SCAN==1

////////////////////////////////////////////////////////////////////////////////////////
//移植注意:本例是以GD32为例,如果要移植到其他mcu,请做相应修改.
//usmart_reset_runtime,清除函数运行时间,连同定时器的计数寄存器以及标志位一起清零.并设置重装载值为最大,以最大限度的延长计时时间.
//usmart_get_runtime,获取函数运行时间,通过读取CNT值获取,由于usmart是通过中断调用的函数,所以定时器中断不再有效,此时最大限度
//只能统计2次CNT的值,也就是清零后+溢出一次,当溢出超过2次,没法处理,所以最大延时,控制在:2*计数器CNT*0.1ms.
//其他的:TIM4_IRQHandler和Timer4_Init,需要根据MCU特点自行修改.确保计数器计数频率为:10Khz即可.另外,定时器不要开启自动重装载功能!!


//复位runtime
//需要根据所移植到的MCU的定时器参数进行修改
void usmart_reset_runtime(void)
{
     timer_interrupt_flag_clear(TIMER3, TIMER_INT_FLAG_UP); //清除中断标志位 
     timer_autoreload_value_config(TIMER3,0XFFFF);          //将重装载值设置到最大
     timer_counter_value_config(TIMER3, 0);                 //清空定时器的计数器
     usmart_dev.runtime=0;	
}

//获得runtime时间
//返回值:执行时间,单位:0.1ms,最大延时时间为定时器CNT值的2倍*0.1ms
//需要根据所移植到的MCU的定时器参数进行修改
uint32_t usmart_get_runtime(void)
{
	if(timer_interrupt_flag_get(TIMER3, TIMER_INT_FLAG_UP) == SET)//在运行期间,产生了定时器溢出
	{
		usmart_dev.runtime+=0XFFFF;
	}
	usmart_dev.runtime+=timer_counter_read(TIMER3);
	return usmart_dev.runtime;		//返回计数值
}  

//定时器3中断服务程序	 
void TIMER3_IRQHandler(void)
{ 		    		  			       
    if(timer_interrupt_flag_get(TIMER3, TIMER_INT_FLAG_UP) == SET)//溢出中断
    {
        usmart_dev.scan();	                          //执行usmart扫描
        timer_counter_value_config(TIMER3, 0);        //清空定时器的计数器
        timer_autoreload_value_config(TIMER3, 1000);  //恢复原来的设置
    }
    timer_interrupt_flag_clear(TIMER3, TIMER_INT_FLAG_UP);//清除中断标志位
}

//定时器初始化函数
//arr：自动重装值。
//psc：时钟预分频数
//使能定时器3,使能中断.
void Timer3_Init(uint16_t arr,uint16_t psc)
{ 
	timer_parameter_struct timer_initpara;               //timer_initpara用于存放定时器的参数

  //使能RCU相关时钟 
  rcu_periph_clock_enable(RCU_TIMER3);                 //使能TIMER3的时钟

  //复位TIMER3
  timer_deinit(TIMER3);                                //复位TIMER3
  timer_struct_para_init(&timer_initpara);             //初始化timer_initpara为默认值

  //配置TIMER3
  timer_initpara.prescaler         = psc;              //设置预分频值
  timer_initpara.counterdirection  = TIMER_COUNTER_UP; //设置向上计数模式
  timer_initpara.period            = arr;              //设置自动重装载值
  timer_initpara.clockdivision     = TIMER_CKDIV_DIV1; //设置时钟分频因子
  timer_init(TIMER3, &timer_initpara);                 //根据参数初始化定时器

  //使能定时器及其中断
  timer_interrupt_enable(TIMER3, TIMER_INT_UP);        //使能定时器的更新中断
  nvic_irq_enable(TIMER3_IRQn, 3, 3);                  //配置NVIC设置优先级，抢占优先级3，响应优先级3
  timer_enable(TIMER3);                                //使能定时器TIMER3			 
}
 

#endif

















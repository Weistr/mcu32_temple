#include "fwdgt.h"


//初始化独立看门狗
//prer:分频数:FWDGT_PSC_DIV4~FWDGT_PSC_DIV256
//rlr:自动重装载值,0~0XFFF.
//时间计算(大概):Tout=((4*2^prer)*rlr)/40 (ms).
void FWDGT_Init(uint8_t prer,uint16_t rlr)
{
	  //配置独立看门狗定时器
  fwdgt_write_enable();          //使能对寄存器的写操作
  
  fwdgt_config(rlr, prer);       //设置重装载值和预分频值
    
  fwdgt_counter_reload();        //将FWDGT_RLD寄存器的值重装载FWDGT计数器
  
  fwdgt_enable();                //使能独立看门狗定时器

}
   
//喂独立看门狗
void FWDGT_Feed(void)
{   
  fwdgt_counter_reload();        //喂狗
}

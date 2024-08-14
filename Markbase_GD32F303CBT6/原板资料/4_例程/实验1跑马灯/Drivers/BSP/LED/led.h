#ifndef _LED_H
#define _LED_H
#include "sys.h"

/* LED配置宏定义 */
#define LED1_PIN             GPIO_PIN_13   // LED1 引脚      
#define LED1_PORT            GPIOC                  // LED1 GPIO端口     
#define LED1_CLK             RCU_GPIOC	 // LED1 GPIO端口时钟


/* LED端口定义 */
#define LED1(x)   do{ x ? \
                      gpio_bit_write(LED1_PORT, LED1_PIN, SET) : \
                      gpio_bit_write(LED1_PORT, LED1_PIN, RESET); \
                  }while(0)     

/* LED取反定义 */
#define LED1_TOGGLE()   do{ gpio_togglepin(LED1_PORT, LED1_PIN); }while(0)        /* 翻转LED1 */

void LED_Init(void);                                      //初始化LED
void gpio_togglepin(uint32_t gpio_periph, uint32_t pin);  //翻转IO口状态
#endif



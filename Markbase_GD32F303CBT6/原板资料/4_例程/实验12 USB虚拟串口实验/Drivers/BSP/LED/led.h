#ifndef _LED_H
#define _LED_H
#include "sys.h"

/* LED端口定义 */
#define LED1(x)   do{ x ? \
                      gpio_bit_write(GPIOC, GPIO_PIN_13, SET) : \
                      gpio_bit_write(GPIOC, GPIO_PIN_13, RESET); \
                  }while(0)     

/* LED取反定义 */
#define LED1_TOGGLE()   do{ gpio_togglepin(GPIOC, GPIO_PIN_13); }while(0)        /* 翻转LED1 */

void LED_Init(void);                                      //初始化LED
void gpio_togglepin(uint32_t gpio_periph, uint32_t pin);  //翻转IO口状态
#endif



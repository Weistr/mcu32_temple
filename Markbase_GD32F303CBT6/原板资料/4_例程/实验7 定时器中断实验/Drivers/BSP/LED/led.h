#ifndef _LED_H
#define _LED_H
#include "sys.h"

/* LED�˿ڶ��� */
#define LED1(x)   do{ x ? \
                      gpio_bit_write(GPIOC, GPIO_PIN_13, SET) : \
                      gpio_bit_write(GPIOC, GPIO_PIN_13, RESET); \
                  }while(0)     

/* LEDȡ������ */
#define LED1_TOGGLE()   do{ gpio_togglepin(GPIOC, GPIO_PIN_13); }while(0)        /* ��תLED1 */

void LED_Init(void);                                      //��ʼ��LED
void gpio_togglepin(uint32_t gpio_periph, uint32_t pin);  //��תIO��״̬
#endif



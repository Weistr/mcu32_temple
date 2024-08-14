#ifndef _LED_H
#define _LED_H
#include "sys.h"

/* LED���ú궨�� */
#define LED1_PIN             GPIO_PIN_13   // LED1 ����      
#define LED1_PORT            GPIOC                  // LED1 GPIO�˿�     
#define LED1_CLK             RCU_GPIOC	 // LED1 GPIO�˿�ʱ��


/* LED�˿ڶ��� */
#define LED1(x)   do{ x ? \
                      gpio_bit_write(LED1_PORT, LED1_PIN, SET) : \
                      gpio_bit_write(LED1_PORT, LED1_PIN, RESET); \
                  }while(0)     

/* LEDȡ������ */
#define LED1_TOGGLE()   do{ gpio_togglepin(LED1_PORT, LED1_PIN); }while(0)        /* ��תLED1 */

void LED_Init(void);                                      //��ʼ��LED
void gpio_togglepin(uint32_t gpio_periph, uint32_t pin);  //��תIO��״̬
#endif



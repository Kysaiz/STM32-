#ifndef __LED_H
#define __LED_H

#include "stm32f4xx.h"

void init_led(void);
void led_on(uint8_t led_num);
void led_off(uint8_t led_num);

#endif


#ifndef __BEEP_H
#define __BEEP_H

#include "stm32f4xx.h"

void beep_pwm_init(uint32_t freq);
void tim13_set_freq(uint32_t freq);
void tim13_set_duty(uint32_t duty);

#endif


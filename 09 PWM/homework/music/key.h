#ifndef __KEY_H
#define __KEY_H

#include "stm32f4xx.h"


void init_key(void);
uint8_t key_scan(uint8_t key_num);

#endif


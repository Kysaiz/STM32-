#ifndef __BIT_H
#define __BIT_H

#include "stm32f4xx.h"

//寄存器的位带别名 = 0x42000000 + （寄存器的地址-0x40000000）*8*4 + 引脚编号*4
#define PAout(x)            *(volatile uint32_t *)(0x42000000 + ( (uint32_t)&GPIOA->ODR - 0x40000000 )*32 + (x*4))
#define PBout(x)            *(volatile uint32_t *)(0x42000000 + ( (uint32_t)&GPIOB->ODR - 0x40000000 )*32 + (x*4))
#define PCout(x)            *(volatile uint32_t *)(0x42000000 + ( (uint32_t)&GPIOC->ODR - 0x40000000 )*32 + (x*4))
#define PDout(x)            *(volatile uint32_t *)(0x42000000 + ( (uint32_t)&GPIOD->ODR - 0x40000000 )*32 + (x*4))
#define PEout(x)            *(volatile uint32_t *)(0x42000000 + ( (uint32_t)&GPIOE->ODR - 0x40000000 )*32 + (x*4))
#define PFout(x)            *(volatile uint32_t *)(0x42000000 + ( (uint32_t)&GPIOF->ODR - 0x40000000 )*32 + (x*4))
#define PGout(x)            *(volatile uint32_t *)(0x42000000 + ( (uint32_t)&GPIOG->ODR - 0x40000000 )*32 + (x*4))

#define PAin(x)             *(volatile uint32_t *)(0x42000000 + ( (uint32_t)&GPIOA->IDR - 0x40000000 )*32 + (x*4))
#define PBin(x)             *(volatile uint32_t *)(0x42000000 + ( (uint32_t)&GPIOB->IDR - 0x40000000 )*32 + (x*4))
#define PCin(x)             *(volatile uint32_t *)(0x42000000 + ( (uint32_t)&GPIOC->IDR - 0x40000000 )*32 + (x*4))
#define PDin(x)             *(volatile uint32_t *)(0x42000000 + ( (uint32_t)&GPIOD->IDR - 0x40000000 )*32 + (x*4))
#define PEin(x)             *(volatile uint32_t *)(0x42000000 + ( (uint32_t)&GPIOE->IDR - 0x40000000 )*32 + (x*4))
#define PFin(x)             *(volatile uint32_t *)(0x42000000 + ( (uint32_t)&GPIOF->IDR - 0x40000000 )*32 + (x*4))
#define PGin(x)             *(volatile uint32_t *)(0x42000000 + ( (uint32_t)&GPIOG->IDR - 0x40000000 )*32 + (x*4))


#endif


#include "stm32f4xx.h"

/*
跑马灯程序
使用位带操作、通过KEY1-3来切换时钟
*/

GPIO_InitTypeDef GPIO_InitStruct;

#define		PFout(x)	*(volatile uint32_t *)(0x42000000 + (GPIOF_BASE + 0x14 - 0x40000000)*32 + x*4)
#define		PEout(x)	*(volatile uint32_t *)(0x42000000 + (GPIOE_BASE + 0x14 - 0x40000000)*32 + x*4)
#define		PEin(x)		*(volatile uint32_t *)(0x42000000 + (GPIOE_BASE + 0x10 - 0x40000000)*32 + x*4)
	
void delay(uint32_t delaytime)
{
	for(;delaytime>0;delaytime--)
	{
		// KEY1 -> PLL
		if (PEin(2) == 0)
		{
			RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
			RCC->CFGR |= RCC_CFGR_SW_PLL;
		}
		
		// KEY2 -> HSI
		if (PEin(3) == 0)
		{
			RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
			RCC->CFGR |= RCC_CFGR_SW_HSI;
		}
		
		// KEY3 -> HSE
		if (PEin(4) == 0)
		{
			RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
			RCC->CFGR |= RCC_CFGR_SW_HSE;
		}
	}
}

int main(void)
{
	// 打开端口F的硬件时钟，等同于对端口F供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	
	// 打开端口E的硬件时钟，等同于对端口E供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; // 配置为输出模式
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; // 配置引脚的响应时间=1/100MHz，从高电平切换到低电平
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // 推挽输出模式 
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // 不使能内部上拉电阻
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; // 指定第9、10根引脚
	GPIO_Init(GPIOF, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14; // 指定第13、14根引脚
	GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	GPIO_SetBits(GPIOF, GPIO_Pin_9 |GPIO_Pin_10);
	GPIO_SetBits(GPIOE, GPIO_Pin_13 | GPIO_Pin_14);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN; // 配置为输入模式

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4; // KEY1、2、3
	GPIO_Init(GPIOE, &GPIO_InitStruct);	

	while(1)
	{
		PFout(9) = 0;
		delay(0x50000);
		PFout(9) = 1;
		
		PFout(10) = 0;
		delay(0x50000);
		PFout(10) = 1;
		
		PEout(13) = 0;
		delay(0x50000);
		PEout(13) = 1;
		
		PEout(14) = 0;
		delay(0x50000);
		PEout(14) = 1;
		
		PEout(13) = 0;
		delay(0x50000);
		PEout(13) = 1;
		
		PFout(10) = 0;
		delay(0x50000);
		PFout(10) = 1;
	}
	
}

#include "stm32f4xx.h"

/* 
程序功能：KEY0-3控制LED0-3的亮灭

*/
GPIO_InitTypeDef GPIO_InitStruct;

int main(void)
{
	// 打开端口A的硬件时钟，等同于对端口A供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	// 打开端口F的硬件时钟，等同于对端口F供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	
	// 打开端口E的硬件时钟，等同于对端口E供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; // 配置为输出模式
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; // 配置引脚的响应时间=1/100MHz，从高电平切换到低电平
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // 推挽输出模式 
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // 不使能内部上拉电阻
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9; // 指定第9根引脚
	GPIO_Init(GPIOF, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10; // 指定第10根引脚
	GPIO_Init(GPIOF, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13; // 指定第13根引脚
	GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14; // 指定第14根引脚
	GPIO_Init(GPIOE, &GPIO_InitStruct);

	GPIO_SetBits(GPIOF, GPIO_Pin_9);
	GPIO_SetBits(GPIOF, GPIO_Pin_10);
	GPIO_SetBits(GPIOE, GPIO_Pin_13);
	GPIO_SetBits(GPIOE, GPIO_Pin_14);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN; // 配置为输入模式
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0; // KEY0
	GPIO_Init(GPIOA, &GPIO_InitStruct);	

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2; // KEY1
	GPIO_Init(GPIOE, &GPIO_InitStruct);	

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3; // KEY2
	GPIO_Init(GPIOE, &GPIO_InitStruct);	

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4; // KEY3
	GPIO_Init(GPIOE, &GPIO_InitStruct);	

	while(1)
	{
		// KEY0 -- LED0
		if (!GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
		{
			GPIO_ResetBits(GPIOF, GPIO_Pin_9);
		}
		else
		{
			GPIO_SetBits(GPIOF, GPIO_Pin_9);
		}
		
		// KEY1 -- LED1
		if (!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2))
		{
			GPIO_ResetBits(GPIOF, GPIO_Pin_10);
		}
		else
		{
			GPIO_SetBits(GPIOF, GPIO_Pin_10);
		}
		
		// KEY2 -- LED2
		if (!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3))
		{
			GPIO_ResetBits(GPIOE, GPIO_Pin_13);
		}
		else
		{
			GPIO_SetBits(GPIOE, GPIO_Pin_13);
		}
		
		// KEY3 -- LED3
		if (!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4))
		{
			GPIO_ResetBits(GPIOE, GPIO_Pin_14);
		}
		else
		{
			GPIO_SetBits(GPIOE, GPIO_Pin_14);
		}
	}
	return 0;
}

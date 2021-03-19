#include "stm32f4xx.h"

/*
中断优先级
KEY0控制LED0，抢占优先级0，响应优先级0
KEY1控制LED1，抢占优先级2，响应优先级1
KEY2控制LED2，抢占优先级2，响应优先级2
KEY3控制LED3，抢占优先级3，响应优先级2
*/
NVIC_InitTypeDef NVIC_InitStruct;
EXTI_InitTypeDef EXTI_InitStruct;
GPIO_InitTypeDef GPIO_InitStruct;

#define		PFout(x)	*(volatile uint32_t *)(0x42000000 + (GPIOF_BASE + 0x14 - 0x40000000)*32 + x*4)
#define		PEout(x)	*(volatile uint32_t *)(0x42000000 + (GPIOE_BASE + 0x14 - 0x40000000)*32 + x*4)
#define		PEin(x)		*(volatile uint32_t *)(0x42000000 + (GPIOE_BASE + 0x10 - 0x40000000)*32 + x*4)
#define		PAin(x)		*(volatile uint32_t *)(0x42000000 + (GPIOA_BASE + 0x10 - 0x40000000)*32 + x*4)

void delay()
{
	int delaytime = 0x1000000;
	for(;delaytime>0;delaytime--);
}

void EXTI0_IRQHandler(void)
{
	// 检查是否有中断触发
	if (EXTI_GetITStatus(EXTI_Line0) == SET)
	{
		PFout(9) = 0;
		delay();
		PFout(9) = 1;
		// 清空标志位，告诉CPU，当前已经完成中断处理，可以响应新的一次中断请求
		EXTI_ClearITPendingBit(EXTI_Line0);		
	}

}

void EXTI2_IRQHandler(void)
{
	// 检查是否有中断触发
	if (EXTI_GetITStatus(EXTI_Line2) == SET)
	{
		PFout(10) = 0;
		delay();
		PFout(10) = 1;
		// 清空标志位，告诉CPU，当前已经完成中断处理，可以响应新的一次中断请求
		EXTI_ClearITPendingBit(EXTI_Line2);		
	}

}

void EXTI3_IRQHandler(void)
{
	// 检查是否有中断触发
	if (EXTI_GetITStatus(EXTI_Line3) == SET)
	{
		PEout(13) = 0;
		delay();
		PEout(13) = 1;
		// 清空标志位，告诉CPU，当前已经完成中断处理，可以响应新的一次中断请求
		EXTI_ClearITPendingBit(EXTI_Line3);		
	}

}

void EXTI4_IRQHandler(void)
{
	// 检查是否有中断触发
	if (EXTI_GetITStatus(EXTI_Line4) == SET)
	{
		PEout(14) = 0;
		delay();
		PEout(14) = 1;
		// 清空标志位，告诉CPU，当前已经完成中断处理，可以响应新的一次中断请求
		EXTI_ClearITPendingBit(EXTI_Line4);		
	}

}

int main(void)
{
	// 打开端口F的硬件时钟，等同于对端口F供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	
	// 打开端口E的硬件时钟，等同于对端口E供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	
	
	// 打开端口A的硬件时钟，等同于对端口A供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	// 中断优先级分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; // 配置为输出模式
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; // 配置引脚的响应时间=1/100MHz，从高电平切换到低电平
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // 推挽输出模式 
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // 不使能内部上拉电阻
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10; // 指定第9、10根引脚
	GPIO_Init(GPIOF, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14; // 指定第13、14根引脚
	GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	GPIO_SetBits(GPIOF, GPIO_Pin_9 |GPIO_Pin_10);
	GPIO_SetBits(GPIOE, GPIO_Pin_13 | GPIO_Pin_14);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN; // 配置为输入模式

	GPIO_InitStruct.GPIO_Pin = GPIO_PinSource2 | GPIO_PinSource3 | GPIO_PinSource4; // KEY1、2、3
	GPIO_Init(GPIOE, &GPIO_InitStruct);	

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0; // KEY0
	GPIO_Init(GPIOA, &GPIO_InitStruct);	

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

	NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	EXTI_InitStruct.EXTI_Line = EXTI_Line0; 
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;  
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling; 
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;  
	EXTI_Init(&EXTI_InitStruct);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource2);
	NVIC_InitStruct.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStruct);
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line2; 
	EXTI_Init(&EXTI_InitStruct);	
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource3);
	NVIC_InitStruct.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_InitStruct);
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line3; 
	EXTI_Init(&EXTI_InitStruct);	

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource4);
	NVIC_InitStruct.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_InitStruct);
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line4; 
	EXTI_Init(&EXTI_InitStruct);

	while(1)
	{
		
	}
	
}

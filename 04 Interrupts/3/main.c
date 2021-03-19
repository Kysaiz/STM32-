#include "stm32f4xx.h"

/*
跑马灯程序
使用位带操作、通过中断控制KEY1-3来切换时钟
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
	int delaytime = 0x100000;
	for(;delaytime>0;delaytime--);
}

// KEY0切换PLL时钟
void EXTI0_IRQHandler(void)
{
	// 检查是否有中断触发
	if (EXTI_GetITStatus(EXTI_Line0) == SET)
	{
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
		RCC->CFGR |= RCC_CFGR_SW_PLL;
		// 清空标志位，告诉CPU，当前已经完成中断处理，可以响应新的一次中断请求
		EXTI_ClearITPendingBit(EXTI_Line0);		
	}

}

// KEY1切换HSI时钟
void EXTI2_IRQHandler(void)
{
	// 检查是否有中断触发
	if (EXTI_GetITStatus(EXTI_Line2) == SET)
	{
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
		RCC->CFGR |= RCC_CFGR_SW_HSI;
		// 清空标志位，告诉CPU，当前已经完成中断处理，可以响应新的一次中断请求
		EXTI_ClearITPendingBit(EXTI_Line2);		
	}

}

// KEY2切换HSE时钟
void EXTI3_IRQHandler(void)
{
	// 检查是否有中断触发
	if (EXTI_GetITStatus(EXTI_Line3) == SET)
	{
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
		RCC->CFGR |= RCC_CFGR_SW_HSE;
		// 清空标志位，告诉CPU，当前已经完成中断处理，可以响应新的一次中断请求
		EXTI_ClearITPendingBit(EXTI_Line3);		
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

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn; // 指定通道
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x0F; // 抢占优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x0F;// 响应优先级
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE; // 打开通道
	NVIC_Init(&NVIC_InitStruct);

	EXTI_InitStruct.EXTI_Line = EXTI_Line0; 
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;  
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling; 
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;  
	EXTI_Init(&EXTI_InitStruct);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource2);
	NVIC_InitStruct.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_Init(&NVIC_InitStruct);
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line2; 
	EXTI_Init(&EXTI_InitStruct);	
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource3);
	NVIC_InitStruct.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_Init(&NVIC_InitStruct);
	
	EXTI_InitStruct.EXTI_Line = EXTI_Line3; 
	EXTI_Init(&EXTI_InitStruct);	

	while(1)
	{
		GPIO_SetBits(GPIOF, GPIO_Pin_9);
		GPIO_ResetBits(GPIOF, GPIO_Pin_10);
		delay();
		GPIO_SetBits(GPIOF, GPIO_Pin_10);
		GPIO_ResetBits(GPIOE, GPIO_Pin_13);
		delay();
		GPIO_SetBits(GPIOE, GPIO_Pin_13);
		GPIO_ResetBits(GPIOE, GPIO_Pin_14);
		delay();
		GPIO_SetBits(GPIOE, GPIO_Pin_14);
		GPIO_ResetBits(GPIOE, GPIO_Pin_13);
		delay();
		GPIO_SetBits(GPIOE, GPIO_Pin_13);
		GPIO_ResetBits(GPIOF, GPIO_Pin_10);
		delay();
		GPIO_SetBits(GPIOF, GPIO_Pin_10);
		GPIO_ResetBits(GPIOF, GPIO_Pin_9);
		delay();
	}
	
}

#include "stm32f4xx.h"

/*
跑马灯程序
使用位带操作、通过KEY1-3来切换时钟
*/

#define		PFout(x)	*(volatile uint32_t *)(0x42000000 + (GPIOF_BASE + 0x14 - 0x40000000)*32 + x*4)
#define		PEout(x)	*(volatile uint32_t *)(0x42000000 + (GPIOE_BASE + 0x14 - 0x40000000)*32 + x*4)
#define		PEin(x)		*(volatile uint32_t *)(0x42000000 + (GPIOE_BASE + 0x10 - 0x40000000)*32 + x*4)
#define		PAin(x)		*(volatile uint32_t *)(0x42000000 + (GPIOA_BASE + 0x10 - 0x40000000)*32 + x*4)

NVIC_InitTypeDef NVIC_InitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

GPIO_InitTypeDef GPIO_InitStruct;

void tim3_init(void)
{
	//使能定时器3硬件时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period = (10000/2)-1;				//定时时间的配置，也就是配置计数值
	TIM_TimeBaseStructure.TIM_Prescaler = 8400-1;				//配置分频值，确定定时器的时钟频率
	//TIM_TimeBaseStructure.TIM_ClockDivision = 0;				//时钟分频（第二次分频），但F407是没有该分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//向上计数，0->TIM_Period就会触发中断请求
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	//使能定时器中断
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	
	//配置定时器优先级			
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;				//定时器3的中断号
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			//响应优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//使能
	NVIC_Init(&NVIC_InitStructure);
	
	//使能定时器工作
	TIM_Cmd(TIM3,ENABLE);
}

void TIM3_IRQHandler(void)
{
	//检查定时器3是否有时间更新中断
	if(TIM_GetITStatus(TIM3,TIM_IT_Update) == SET)
	{
		PFout(9) ^= 1;
		//清空标志位
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);	
	
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

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4; // KEY1、2、3
	GPIO_Init(GPIOE, &GPIO_InitStruct);	

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0; // KEY0
	GPIO_Init(GPIOA, &GPIO_InitStruct);	
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	tim3_init();
	
	PFout(9)=PFout(10)=PEout(13)=PEout(14)=1;
	while(1)
	{
	
	}
	
}

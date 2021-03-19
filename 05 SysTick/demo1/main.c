#include "stm32f4xx.h"

GPIO_InitTypeDef GPIO_InitStructure ;
EXTI_InitTypeDef EXTI_InitStructure ;
NVIC_InitTypeDef NVIC_InitStructure ;

void delay(void)
{

	uint32_t  i=0x2000000;
	while(i--);
}

#define PFout(n)	*((uint32_t *)(0x42000000+(GPIOF_BASE+0x14-0x40000000)*32+(n)*4))
#define PEout(n)	*((uint32_t *)(0x42000000+(GPIOE_BASE+0x14-0x40000000)*32+(n)*4))


int main(void )
{
	//打开端口E的硬件时钟，等同于对端口E供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE );
	
	//打开端口F的硬件时钟，等同于对端口F供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE );
	
	//初始化对应端口的引脚 
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_9|GPIO_Pin_10;//指定第9根引脚 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT ;//配置为输出模式
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz ;//配置引脚的响应时间=1/100MHz .
	//从高电平切换到低电平1/100MHz,速度越快，功耗会越高
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP ;//推挽的输出模式，增加输出电流和灌电流的能力
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;//不使能内部上下拉电阻
	GPIO_Init(GPIOF ,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_13|GPIO_Pin_14;//指定第9根引脚 
	GPIO_Init(GPIOE ,&GPIO_InitStructure);
	
	PFout(9)=1;
	PFout(10)=1;	
	PEout(13)=1;
	PEout(14)=1;	

	//1秒内触发1000次中断，中断周期T=1/f=1/1000Hz=1ms
	SysTick_Config(SystemCoreClock/1000);

	
	while(1)
	{
	

	}
	
	return 0 ;
}

void SysTick_Handler(void)
{
	static uint32_t i1=0;
	
	i1++;
	
	//500ms已到
	if(i1%100==0)
	{
		PFout(9)^=1;
	}

	if(i1%330==0)
	{
		PFout(10)^=1;
	}
	
	if(i1%1500==0)
	{
		PEout(13)^=1;
	}
	
	if(i1>=2200)
	{
		PEout(14)^=1;
		i1=0;
	}

}




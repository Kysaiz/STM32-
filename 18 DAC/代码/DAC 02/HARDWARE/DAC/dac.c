#include "dac.h"

static DAC_InitTypeDef   		DAC_InitStructure;
static GPIO_InitTypeDef 		GPIO_InitStructure;

void dac_init(void)
{

	//打开端口A的硬件时钟，就是对端口A供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);


	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

	//配置PA4引脚为AN模式，就是模拟信号模式
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	
	GPIO_Init(GPIOA,&GPIO_InitStructure);	
	
	
	DAC_InitStructure.DAC_Trigger = DAC_Trigger_None;
	DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;//不需要输出波形
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;//打开DAC输出电压的通道
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);
	
	//打开通道1
	DAC_Cmd(DAC_Channel_1, ENABLE);


}

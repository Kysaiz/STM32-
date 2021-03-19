#include "adc.h"

static GPIO_InitTypeDef 		GPIO_InitStructure;
static ADC_InitTypeDef   		ADC_InitStructure;
static ADC_CommonInitTypeDef 	ADC_CommonInitStructure;

void adc_init(void)
{
	//打开端口A的硬件时钟，就是对端口A供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	//打开ADC1的硬件时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	//配置PA5引脚为AN模式，就是模拟信号模式
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	
	GPIO_Init(GPIOA,&GPIO_InitStructure);	
	
	//配置ADC1相关的参数
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//单个ADC工作
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;//ADC123的硬件时钟频率=84MHz/2=42MHz
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;//默认不使用dma
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样点的间隔=5*(1/42MHz)
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	//使能ADC1工作
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//使用最高精度进行测量

	
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//只扫描一个通道；ENABLE就是扫描多个通道。
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//不断的对通道的模拟信号进行转换
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//不需要脉冲来触发ADC工作
	//ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐存储
	ADC_InitStructure.ADC_NbrOfConversion = 1;//只有1个通道需要进行转换
	ADC_Init(ADC1, &ADC_InitStructure);

	//告诉ADC1转换哪个通道，当前是转换通道5，顺序号为1（如果有多个通道进行扫描，这个数值决定扫描的顺序），测量采样的点的时间为3*(1/42MHz)
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_3Cycles);
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_3Cycles);

	//使能ADC1工作
	ADC_Cmd(ADC1,ENABLE);
}

// 光敏电阻初始化
void photoresistor_init(void)
{
	//打开端口F的硬件时钟，就是对端口F供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
	
	//打开ADC3的硬件时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
	
	//配置PF7引脚为AN模式，就是模拟信号模式
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	
	GPIO_Init(GPIOF,&GPIO_InitStructure);	
	
	//配置ADC1相关的参数
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//单个ADC工作
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;//ADC123的硬件时钟频率=84MHz/2=42MHz
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;//默认不使用dma
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样点的间隔=5*(1/42MHz)
	ADC_CommonInit(&ADC_CommonInitStructure);
	
	//使能ADC3工作
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//使用最高精度进行测量
	
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//只扫描一个通道；ENABLE就是扫描多个通道。
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//不断的对通道的模拟信号进行转换
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//不需要脉冲来触发ADC工作
	//ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐存储
	ADC_InitStructure.ADC_NbrOfConversion = 1;//只有1个通道需要进行转换
	ADC_Init(ADC3, &ADC_InitStructure);

	//告诉ADC3转换哪个通道，当前是转换通道7，顺序号为1（如果有多个通道进行扫描，这个数值决定扫描的顺序），测量采样的点的时间为3*(1/42MHz)
	ADC_RegularChannelConfig(ADC3, ADC_Channel_5, 1, ADC_SampleTime_3Cycles);
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_3Cycles);

	//使能ADC3工作
	ADC_Cmd(ADC3,ENABLE);
}


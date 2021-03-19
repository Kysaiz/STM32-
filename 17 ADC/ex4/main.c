#include "stm32f4xx.h"
#include <stdio.h>

static GPIO_InitTypeDef 		GPIO_InitStructure;
static NVIC_InitTypeDef   		NVIC_InitStructure;
static USART_InitTypeDef   		USART_InitStructure;
static ADC_InitTypeDef   		ADC_InitStructure;
static ADC_CommonInitTypeDef 	ADC_CommonInitStructure;
//使用到IDR寄存器
#define PAin(n)		(*(volatile uint32_t *)(0x42000000 + (GPIOA_BASE + 0x10 - 0x40000000)*32 + n*4))
#define PEin(n)		(*(volatile uint32_t *)(0x42000000 + (GPIOE_BASE + 0x10 - 0x40000000)*32 + n*4))
#define PGin(n)		(*(volatile uint32_t *)(0x42000000 + (GPIOG_BASE + 0x10 - 0x40000000)*32 + n*4))

//使用到ODR寄存器
#define PEout(n)	(*(volatile uint32_t *)(0x42000000 + (GPIOE_BASE + 0x14 - 0x40000000)*32 + n*4))
#define PFout(n)	(*(volatile uint32_t *)(0x42000000 + (GPIOF_BASE + 0x14 - 0x40000000)*32 + n*4))
#define PGout(n)	(*(volatile uint32_t *)(0x42000000 + (GPIOG_BASE + 0x14 - 0x40000000)*32 + n*4))	

#pragma import(__use_no_semihosting_swi)

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f) 
{
	USART_SendData(USART1,ch);
		
	//等待数据发送成功
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);


	return ch;
}

void _sys_exit(int return_code) {

}

void delay_us(uint32_t n)
{
	SysTick->CTRL = 0; // Disable SysTick
	SysTick->LOAD = n*168-1; // 计数值 
	SysTick->VAL  = 0; // Clear current value as well as count flag
	SysTick->CTRL = 5; // Enable SysTick timer with processor clock(168MHz)
	while ((SysTick->CTRL & 0x00010000)==0);// Wait until count flag is set
	SysTick->CTRL = 0; // Disable SysTick	
}



void delay_ms(uint32_t n)
{
	while(n--)
	{
		SysTick->CTRL = 0; // Disable SysTick
		SysTick->LOAD = 168000-1; // 计数值 Count from n to 0 (168000-1+1=168000 cycles)
		SysTick->VAL = 0; // Clear current value as well as count flag
		SysTick->CTRL = 5; // Enable SysTick timer with processor clock(168MHz)
		while ((SysTick->CTRL & 0x00010000)==0);// Wait until count flag is set
		SysTick->CTRL = 0; // Disable SysTick	
	
	}
}


void usart1_init(uint32_t baud)
{
	//使能端口A硬件时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	//使能串口1硬件时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	
	//配置PA9、PA10为复用功能引脚
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;	
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	//将PA9、PA10连接到USART1的硬件
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	
	
	//配置USART1的相关参数：波特率、数据位、校验位
	USART_InitStructure.USART_BaudRate = baud;//波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//1位停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//允许串口发送和接收数据
	USART_Init(USART1, &USART_InitStructure);
	
	
	//使能串口接收到数据触发中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//使能串口1工作
	USART_Cmd(USART1,ENABLE);
}

void usart1_send_str(char *str)
{

	char *p = str;
	
	while(*p!='\0')
	{
		USART_SendData(USART1,*p);
		
		p++;
		
		
		//等待数据发送成功
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
	
	}
}



void adc_init(void)
{

	//打开端口A的硬件时钟，就是对端口A供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	
	//打开ADC1的硬件时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
	
	//配置PA5引脚为AN模式，就是模拟信号模式
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
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
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_3Cycles);
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_3Cycles);



	//使能ADC1工作
	ADC_Cmd(ADC1,ENABLE);
}



int main(void)
{
	uint32_t i=0;
	uint16_t adc_val=0;
	uint16_t adc_vol=0;
	uint32_t sum=0;
		
	
	//打开端口F的硬件时钟，就是对端口F供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);

	
	//配置GPIOF的第9根
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	
	//初始化
	GPIO_Init(GPIOF,&GPIO_InitStructure);
	
	
	PFout(9)=1;

	
	
	//PG9输出端口为高阻态，这是引脚的初始电平状态
	PGout(9)=1;	
	
	
	//串口1初始化波特率为115200bps
	usart1_init(115200);
	
	delay_ms(500);
	
	//发送数据
	usart1_send_str("This is flash test\r\n");
	
	

	adc_init();
	
	//启动ADC1开始转换
	ADC_SoftwareStartConv(ADC1);

	while(1)
	{
		for(sum=0,i=0; i<100; i++)
		{
			while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET);
			adc_val=ADC_GetConversionValue(ADC1);
			ADC_ClearFlag(ADC1,ADC_FLAG_EOC);
			sum+=adc_val;	
		}
		adc_val = sum/100;
		
		printf("adc_val =%d \r\n",adc_val);
		adc_vol = adc_val *3300/4095;
		printf("adc_vol =%dmv \r\n",adc_vol);		
		
		delay_ms(1000);	
	}

}

void USART1_IRQHandler(void)
{
	static uint8_t d=0;
	
	//检测是否接收到数据
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		d=USART_ReceiveData(USART1);
		
		//将接收到的数据返发给PC
		USART_SendData(USART1,d);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
	
		
		
		//0x01，点亮PF9
		if(d == 0x01)PFout(9)=0;
		
		//0xF1，熄灭PF9
		if(d == 0xF1)PFout(9)=1;
		
		
		//清空标志位，可以响应新的中断请求
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}

}



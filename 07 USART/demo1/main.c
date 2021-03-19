#include "stm32f4xx.h"
#include <stdio.h>

GPIO_InitTypeDef GPIO_InitStructure ;
EXTI_InitTypeDef EXTI_InitStructure ;
NVIC_InitTypeDef NVIC_InitStructure ;
USART_InitTypeDef USART_InitStructure ;

#define PFout(n)	*((uint32_t *)(0x42000000+(GPIOF_BASE+0x14-0x40000000)*32+(n)*4))
#define PEout(n)	*((uint32_t *)(0x42000000+(GPIOE_BASE+0x14-0x40000000)*32+(n)*4))

#pragma import(__use_no_semihosting_swi)

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, ch);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);

	return ch;
}

void _sys_exit(int return_code) {
label: goto label; /* endless loop */
}

void delay_us(uint32_t n)
{
	SysTick->CTRL = 0; // Disable SysTick，关闭系统定时器后才能取配置
	SysTick->LOAD = 21*n-1; 	// 填写计数值，就是我们的延时时间
	SysTick->VAL = 0; 			// 清空标志位
	SysTick->CTRL = 1; 			// 选中21MHz的时钟源，并开始让系统定时器工作
	while ((SysTick->CTRL & 0x10000)==0);//等待计数完毕
	SysTick->CTRL = 0; // 不再使用就关闭系统定时器
}


void delay_ms(uint32_t n)
{

	while(n--)
	{
		SysTick->CTRL = 0; // Disable SysTick，关闭系统定时器后才能取配置
		SysTick->LOAD = 21000-1; 	// 填写计数值，就是我们的延时时间
		SysTick->VAL = 0; 			// 清空标志位
		SysTick->CTRL = 1; 			// 选中21MHz的时钟源，并开始让系统定时器工作
		while ((SysTick->CTRL & 0x10000)==0);//等待计数完毕
		
	}
	SysTick->CTRL = 0; // 不再使用就关闭系统定时器
}


void usart1_init(uint32_t baud)
{
	//打开端口A硬件时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE );
	
	//打开串口1的硬件时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE );	
	
	//配置PA9和PA10引脚，为AF模式（复用功能模式）
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_9|GPIO_Pin_10;//指定第9根引脚 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF ;//配置为复用功能模式
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz ;//配置引脚的响应时间=1/100MHz .
	//从高电平切换到低电平1/100MHz,速度越快，功耗会越高
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP ;//推挽的输出模式，增加输出电流和灌电流的能力
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;//不使能内部上下拉电阻
	GPIO_Init(GPIOA ,&GPIO_InitStructure);


	//将PA9和PA10的功能进行指定为串口1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);	
	
	//配置串口1的参数：波特率、数据位、校验位、停止位、流控制
	USART_InitStructure.USART_BaudRate = baud;//波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//允许串口发送和接收数据
	USART_Init(USART1, &USART_InitStructure);
	
	//使能串口1的接收中断
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	//使能串口1工作
	USART_Cmd(USART1, ENABLE);

}

int main(void )
{
	uint32_t d = 0x100;
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
	
	//初始化串口1的波特率为115200bps
	//注意：如果接收的数据是乱码，要检查PLL。
	usart1_init(115200);

	delay_ms(100);

	printf("Hello GEC\r\n");
	
	printf("d = %d\n", d);
	
	while(1)
	{
	
		
	}
	
	return 0 ;
}


void USART1_IRQHandler(void)
{

	uint8_t d;
	
	//检测是否接收到数据
	if(USART_GetITStatus(USART1,USART_IT_RXNE)==SET)
	{
		d = USART_ReceiveData(USART1);
		
		if(d == 0x01)PFout(9)=0;
		if(d == 0xF1)PFout(9)=1;	
		if(d == 0x02)PFout(10)=0;
		if(d == 0xF2)PFout(10)=1;
		if(d == 0x03)PEout(13)=0;
		if(d == 0xF3)PEout(13)=1;
		if(d == 0x04)PEout(14)=0;
		if(d == 0xF4)PEout(14)=1;
		
		USART_SendData(USART1,d);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);		
		
		//清空标志位，告诉CPU当前数据接收完毕，可以接收新的数据
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	
	}




}




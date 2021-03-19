#include "stm32f4xx.h"
#include <stdio.h>

GPIO_InitTypeDef GPIO_InitStructure ;
EXTI_InitTypeDef EXTI_InitStructure ;
NVIC_InitTypeDef NVIC_InitStructure ;
USART_InitTypeDef USART_InitStructure ;

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
uint32_t cmp = 100;

#define PFout(n)	*((volatile uint32_t *)(0x42000000+(GPIOF_BASE+0x14-0x40000000)*32+(n)*4))
#define PBout(n)	*((volatile uint32_t *)(0x42000000+(GPIOB_BASE+0x14-0x40000000)*32+(n)*4))
#define PEout(n)	*((volatile uint32_t *)(0x42000000+(GPIOE_BASE+0x14-0x40000000)*32+(n)*4))
#define PEin(n)		*((volatile uint32_t *)(0x42000000+(GPIOE_BASE+0x10-0x40000000)*32+(n)*4))

#pragma import(__use_no_semihosting_swi)


struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;


int fputc(int ch, FILE *f) 
{
		
	
	USART_SendData(USART1,ch);
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

void sr04_init(void)
{
	//使能端口B、端口E的硬件时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE );
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE );
	
	
	//PB6为输出模式，因为该引脚连接到Trig
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6;//指定第6根引脚 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT ;//配置为输出模式
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz ;//配置引脚的响应时间=1/100MHz .
	//从高电平切换到低电平1/100MHz,速度越快，功耗会越高
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP ;//推挽的输出模式，增加输出电流和灌电流的能力
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;//不使能内部上下拉电阻
	GPIO_Init(GPIOB ,&GPIO_InitStructure);	
	
	
	//PE6为输入模式，因为要检测ECHO输出高电平的持续时间
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6;//指定第6根引脚 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN ;//配置为输出模式
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz ;//配置引脚的响应时间=1/100MHz .
	//从高电平切换到低电平1/100MHz,速度越快，功耗会越高
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP ;//推挽的输出模式，增加输出电流和灌电流的能力
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;//不使能内部上下拉电阻
	GPIO_Init(GPIOE ,&GPIO_InitStructure);	
	
	
	//PB6引脚初始状态为低电平，根据时序图可以了解到
	PBout(6)=0;

}


uint32_t sr04_get_distance(void)
{
	uint32_t t=0;

	//PB6输出高电平
	PBout(6)=1;
	
	//延时10us
	delay_us(10);

	//PB6输出低电平
	PBout(6)=0;	
	
	//PE6要等待高电平出现
	while(PEin(6)==0);
	
	
	//测量高电平的持续时间
	while(PEin(6))
	{
	
		t++;
		delay_us(9);//超声波每传输9us时间，距离为3mm
	
	}
	
	//因为该时间是包含发射和返回的时间，需要除以2
	t/=2;
	
	return 3*t;
}

void tim13_init(void)
{
	//打开TIM13硬件时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);

	//配置TIM13的相关参数：计数值（决定定时时间）、分频值
	//TIM13的硬件时钟频率=84000000/8400=10000Hz
	//只要进行10000次计数，就是1秒时间的到达，也就是1Hz的频率输出
	//只要进行10000/100次计数，就是1秒/100时间的到达，也就是100Hz的频率输出
	TIM_TimeBaseStructure.TIM_Period = 10000/2-1;//计数值100-1，决定了输出频率为100Hz
	TIM_TimeBaseStructure.TIM_Prescaler = 8400-1;//预分频，也就是第一次分频，当前8400分频，就是84MHz/8400
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;//时钟分频，也称之二次分频这个F407是不支持的，因此该参数是不会生效
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//就是从0开始计数，然后计数到TIM_Period
	TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStructure);
	
	//占空比的配置
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;				//工作在PWM1模式
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//脉冲的输出开关，当前是输出脉冲
	TIM_OCInitStructure.TIM_Pulse = 0;								//这个值是决定了占空比，配置比较值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;		//有效的状态为高电平
	
	//TIM13的通道1配置
	TIM_OC1Init(TIM13, &TIM_OCInitStructure);
	
	//使能定时器13工作
	TIM_Cmd(TIM13, ENABLE);
}


int main(void )
{
	uint32_t d=1000;
	//打开端口E的硬件时钟，等同于对端口E供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE );
	
	//打开端口F的硬件时钟，等同于对端口F供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE );
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	// 中断优先级分组
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	//初始化对应端口的引脚 
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_9|GPIO_Pin_10;// F9、F10，LED0，LED1
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT ;//配置为输出模式
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz ;//配置引脚的响应时间=1/100MHz .
	//从高电平切换到低电平1/100MHz,速度越快，功耗会越高
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP ;//推挽的输出模式，增加输出电流和灌电流的能力
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;//不使能内部上下拉电阻
	GPIO_Init(GPIOF ,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_13|GPIO_Pin_14;// E13、E14，LED2，LED3
	GPIO_Init(GPIOE ,&GPIO_InitStructure);
	
	PFout(9)=PFout(10)=PEout(13)=PEout(14)=1;
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8; // 蜂鸣器
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF ;//配置为复用模式
	GPIO_Init(GPIOF ,&GPIO_InitStructure);
	
	// 中断结构体配置
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource2);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	EXTI_InitStructure.EXTI_Line = EXTI_Line2; 
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;  
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;  
	EXTI_Init(&EXTI_InitStructure);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource3);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line3; 
	EXTI_Init(&EXTI_InitStructure);	
	
	//初始化串口1的波特率为115200bps
	//注意：如果接收的数据是乱码，要检查PLL。
	usart1_init(115200);
	
	delay_ms(100);

	printf("倒车雷达程序\r\n");

	tim13_init();
	
	//将PF8连接到TIM13
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource8, GPIO_AF_TIM13);

	sr04_init();
	
	while(1)
	{
		d = sr04_get_distance();
		
		if(d>=20 && d<=4000)
		{
			printf("distance = %d mm\r\n",d);
		
			if (d >= 200) //  20cm理想安全区，所有LED灯熄灭
			{
				PFout(9)=PFout(10)=PEout(13)=PEout(14)=1;
				TIM_SetCompare1(TIM13,0); // 蜂鸣器静音
				delay_ms(940);
			}
			else if (d >= 150) // 15~20cm非常安全区，亮1盏LED灯
			{
				PFout(9)=0;
				PFout(10)=PEout(13)=PEout(14)=1;
				TIM_SetCompare1(TIM13,0); // 蜂鸣器静音
				delay_ms(940);
			}
			else if (d >= 100) // 10~15cm安全区，亮2盏LED灯
			{
				PFout(9)=PFout(10)=0;
				PEout(13)=PEout(14)=1;
				TIM_TimeBaseStructure.TIM_Period = 10000/2-1; // 输出频率2Hz
				TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStructure);
				TIM_SetCompare1(TIM13,cmp);
				delay_ms(440);
			}
			else if (d >= 50) // 5~10cm警告区，亮3盏LED灯
			{
				PFout(9)=PFout(10)=PEout(13)=0;
				PEout(14)=1;
				TIM_TimeBaseStructure.TIM_Period = 10000/5-1; // 输出频率5Hz
				TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStructure);
				TIM_SetCompare1(TIM13,cmp);
				delay_ms(440);
			}
			else // <5cm 危险区，亮4盏LED灯
			{
				PFout(9)=PFout(10)=PEout(13)=PEout(14)=0;
				TIM_TimeBaseStructure.TIM_Period = 10000/10-1; // 输出频率10Hz
				TIM_TimeBaseInit(TIM13, &TIM_TimeBaseStructure);
				TIM_SetCompare1(TIM13,cmp);
				delay_ms(440);
			}
		
		}
		
		delay_ms(60);
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
		
		if(d == 'a')PFout(9)=0;
		if(d == 'A')PFout(9)=1;		
	
		
		//清空标志位，告诉CPU当前数据接收完毕，可以接收新的数据
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	
	}

}

void EXTI2_IRQHandler(void)
{
	// 检查是否有中断触发
	if (EXTI_GetITStatus(EXTI_Line2) == SET)
	{
		cmp = 100;
		// 清空标志位，告诉CPU，当前已经完成中断处理，可以响应新的一次中断请求
		EXTI_ClearITPendingBit(EXTI_Line2);		
	}

}

void EXTI3_IRQHandler(void)
{
	// 检查是否有中断触发
	if (EXTI_GetITStatus(EXTI_Line3) == SET)
	{
		cmp = 400;
		// 清空标志位，告诉CPU，当前已经完成中断处理，可以响应新的一次中断请求
		EXTI_ClearITPendingBit(EXTI_Line3);		
	}

}

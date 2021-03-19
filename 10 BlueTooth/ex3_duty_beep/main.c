#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
//#define PFout(n)	*((uint32_t *)(0x42000000+((uint32_t)(&GPIOF->ODR)-0x40000000)*32+(n)*4))
//#define PFout(n)	*((uint32_t *)(0x42000000+(GPIOF_BASE+0x14-0x40000000)*32+(n)*4))
#define PFout(n)	*((volatile uint32_t *)(0x42000000+(GPIOF_BASE+0x14-0x40000000)*32+(n)*4))
#define PBout(n)	*((volatile uint32_t *)(0x42000000+(GPIOB_BASE+0x14-0x40000000)*32+(n)*4))
#define PEin(n)		*((volatile uint32_t *)(0x42000000+(GPIOE_BASE+0x10-0x40000000)*32+(n)*4))

#pragma import(__use_no_semihosting_swi)

GPIO_InitTypeDef 	GPIO_InitStructure ;
EXTI_InitTypeDef 	EXTI_InitStructure ;
NVIC_InitTypeDef 	NVIC_InitStructure ;
USART_InitTypeDef 	USART_InitStructure ;
//TIM_InitTypeDef 	TIM_InitStructure ;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;

static volatile uint32_t g_usart3_event=0;		//只要数据包接收完毕，该变量就得置1
static volatile uint8_t  g_usart3_buf[64]={0};	//数据缓冲区
static volatile uint32_t g_usart3_cnt=0;		//数据数目的计数

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



void usart3_init(uint32_t baud)
{
	//打开端口B硬件时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE );
	
	//打开串口3的硬件时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE );	
	
	//配置PB10和PB11引脚，为AF模式（复用功能模式）
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10|GPIO_Pin_11;//指定第10 11根引脚 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF ;//配置为复用功能模式
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz ;//配置引脚的响应时间=1/100MHz .
	//从高电平切换到低电平1/100MHz,速度越快，功耗会越高
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP ;//推挽的输出模式，增加输出电流和灌电流的能力
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;//不使能内部上下拉电阻
	GPIO_Init(GPIOB ,&GPIO_InitStructure);


	//将PB10和PB11的功能进行指定为串口1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);	
	
	//配置串口1的参数：波特率、数据位、校验位、停止位、流控制
	USART_InitStructure.USART_BaudRate = baud;//波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//8位数据位
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//1个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无校验
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//允许串口发送和接收数据
	USART_Init(USART3, &USART_InitStructure);
	
	
	//使能串口3的接收中断
	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	
	
	//使能串口3工作
	USART_Cmd(USART3, ENABLE);

}


void usart3_send_str(char *str)
{
	char *p = str;
	
	while(p && (*p!='\0'))
	{
	
		USART_SendData(USART3,*p);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET);

		p++;
	
	}


}

void tim14_init(void)
{
	//打开TIM14的硬件时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

	
	//配置TIM14的相关参数：计数值（决定定时时间）、分频值
	//TIM14的硬件时钟频率=84000000/8400=10000Hz
	//只要进行10000次计数，就是1秒时间的到达，也就是1Hz的频率输出
	//只要进行10000/100次计数，就是1秒/100时间的到达，也就是100Hz的频率输出
	TIM_TimeBaseStructure.TIM_Period = 10000/100-1;//计数值100-1，决定了输出频率为100Hz
	TIM_TimeBaseStructure.TIM_Prescaler = 8400-1;//预分频，也就是第一次分频，当前8400分频，就是84MHz/8400
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;//时钟分频，也称之二次分频这个F407是不支持的，因此该参数是不会生效
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//就是从0开始计数，然后计数到TIM_Period
	TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);
	
	//占空比的配置
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;				//工作在PWM1模式
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//脉冲的输出开关，当前是输出脉冲
	TIM_OCInitStructure.TIM_Pulse = 100;							//这个值是决定了占空比，配置比较值，默认LED灯为熄灭状态
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//有效的状态为高电平
	
	//TIM14的通道1配置
	TIM_OC1Init(TIM14, &TIM_OCInitStructure);
	

	//使能定时器14工作
	TIM_Cmd(TIM14, ENABLE);
}

int main(void )
{
	
	char *p=NULL;
	
	uint32_t duty=0;
	
	char buf[64]={0};
	
	//打开端口E的硬件时钟，等同于对端口E供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE );
	
	//打开端口F的硬件时钟，等同于对端口F供电
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE );
	
	//初始化对应端口的引脚 
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_9;//指定第9根引脚 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF ;//配置为复用模式
	GPIO_InitStructure.GPIO_Speed =GPIO_Speed_50MHz ;//配置引脚的响应时间=1/100MHz .
	//从高电平切换到低电平1/100MHz,速度越快，功耗会越高
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP ;//推挽的输出模式，增加输出电流和灌电流的能力
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL ;//不使能内部上下拉电阻
	GPIO_Init(GPIOF ,&GPIO_InitStructure);
	
   
   	/* 配置PF8 beep引脚为模式 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;					//第8根引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;				//设置复用模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;				//推挽模式，增加驱动电流
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;			//设置IO的速度为100MHz，频率越高性能越好，频率越低，功耗越低
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;			//不需要上拉电阻
	GPIO_Init(GPIOF, &GPIO_InitStructure);	 
    
    PFout(8) =0 ;
    
	//将PF9连接到TIM14
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource9, GPIO_AF_TIM14);
		
	tim14_init();
	//初始化串口1的波特率为115200bps
	//注意：如果接收的数据是乱码，要检查PLL。
	usart1_init(115200);
	delay_ms(100);
	//串口3的波特率为9600bps，因为蓝牙模块默认使用该波特率
	usart3_init(9600);
	
	delay_ms(500);

	printf("This is Bluetooth test\r\n");
	
	


	while(1)
	{
		//检测是否接收到数据包
		if(g_usart3_event)
		{
			//"duty=50#"
			if(strstr((const char *)g_usart3_buf,"duty"))
			{
				p = strtok((char *)g_usart3_buf,"=");// p="duty"
				printf("strtok[1]=%s\r\n",p);
				
				
				p=strtok(NULL,"=");
				printf("strtok[2]=%s\r\n",p);//p="50#"	

				duty = atoi(p);			//atoi("50#") -> 50
				
				//通过占空比设置灯光的亮度
				TIM_SetCompare1(TIM14,duty);
				
				sprintf(buf,"duty set %d ok\r\n",duty);
				
				usart3_send_str(buf);
			
			}
            //"beep=on#"
			if(strstr((const char *)g_usart3_buf,"beep=on"))
			{

                PFout(8) =1 ;
                printf("beep on ok\r\n");
				
				usart3_send_str("beep on ok\r\n");
			
			}
            
            //"beep=off#"
			if(strstr((const char *)g_usart3_buf,"beep=off"))
			{

                PFout(8) =0 ;
                printf("beep off ok\r\n");
				
				usart3_send_str("beep off ok\r\n");
			
			}
            
            
            

			memset((void *)g_usart3_buf,0,sizeof g_usart3_buf);
			g_usart3_cnt=0;
			g_usart3_event=0;
		}
		
	
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
		
			
		//清空标志位，告诉CPU当前数据接收完毕，可以接收新的数据
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	
	}
}

void USART3_IRQHandler(void)
{

	uint8_t d;
	
	
	//检测是否接收到数据
	if(USART_GetITStatus(USART3,USART_IT_RXNE)==SET)
	{
		d = USART_ReceiveData(USART3);
		
		g_usart3_buf[g_usart3_cnt]=d;

		g_usart3_cnt++;
		
		//检测是否数据接收完毕
		//检测数据的计数是否大于缓冲区
		if((d == '#') || (g_usart3_cnt>sizeof g_usart3_buf))
		{
			g_usart3_event=1;
		
		}
		
		//清空标志位，告诉CPU当前数据接收完毕，可以接收新的数据
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);
	
	}
}


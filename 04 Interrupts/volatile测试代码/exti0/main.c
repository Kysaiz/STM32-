#include "stm32f4xx.h"
#include "sys.h"


//GPIO初始化的结构体
static GPIO_InitTypeDef  GPIO_InitStructure;

//EXTI初始化的结构体
static EXTI_InitTypeDef   EXTI_InitStructure;

//NVIC初始化的结构体
static NVIC_InitTypeDef   NVIC_InitStructure;

void delay(void)
{

	uint32_t i=0x5000000;
	
	while(i--);
}

//不添加volatile 
static volatile uint32_t g_exti0_event=0;


int main(void)
{
	uint32_t a=0;
	uint32_t b=0;

	/* GPIOA GPIOE GPIOF硬件时钟使能，就是让GPIOA GPIOE GPIOF工作 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);	
	
	/* 使能系统配置时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	

	
	
	/* 配置PF9 PF10为输出模式，让这根引脚具有输出高低电平的功能 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;	//第9 10号引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;			//输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//推挽输出，增强驱动能力，引脚的输出电流更大
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		//引脚的速度最大为100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;		//没有使用内部上拉电阻
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOF,GPIO_Pin_9);

	
	/* 配置PA0为输入模式，让这根引脚具有识别高低电平的功能 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;				//第0号引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;			//输入模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		//引脚的速度最大为100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;		//没有使用内部上拉电阻
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
		

	/* 将PA0引脚连接到外部中断控制线0*/
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

	/* 配置外部中断控制线0 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;				//指定外部中断控制线是为EXTI0
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		//中断触发
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  //上升沿触发：按键释放才会触发中断
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;				//使能中断/屏蔽
	EXTI_Init(&EXTI_InitStructure);

	/* 允许外部中断控制线0触发中断，并配置优先级 */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//外部中断控制线0的中断号
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;//抢占优先级 0x0F
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;		//响应（子）优先级0x0F
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//使能中断/屏蔽
	NVIC_Init(&NVIC_InitStructure);
	
	PFout(9)=PFout(10)=1;
	
	a=g_exti0_event;	//0
	
	delay();
	PFout(9)=0;			//点亮时候，我就按下按键1
	delay();
	PFout(9)=1;
	
	b=g_exti0_event;	//1
	
	if(a == b)			//a=0 b=1
	{
		PFout(10)=0;
		
		delay();
		
		PFout(10)=1;
		
		delay();

	
	}
	
	
	
	while(1)
	{
		

	}

}

//中断服务函数
void EXTI0_IRQHandler(void)
{
	//检查外部中断控制线0的中断状态是否有中断请求
	if(EXTI_GetITStatus(EXTI_Line0) == SET)
	{
		
		g_exti0_event++;

		/* 清空外部中断控制线0的标志（挂起）位，告诉CPU已经处理完中断请求，可以处理新的一次中断请求 */
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}



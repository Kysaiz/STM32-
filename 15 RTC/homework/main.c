#include "stm32f4xx.h"
#include <stdio.h>
#include <string.h>

static GPIO_InitTypeDef 		GPIO_InitStructure;
static NVIC_InitTypeDef   		NVIC_InitStructure;
static USART_InitTypeDef   		USART_InitStructure;
static RTC_InitTypeDef   		RTC_InitStructure;
static RTC_TimeTypeDef  		RTC_TimeStructure;
static RTC_DateTypeDef			RTC_DateStructure;
static EXTI_InitTypeDef			EXTI_InitStructure;
static RTC_AlarmTypeDef 		RTC_AlarmStructure;

static volatile uint32_t 	g_rtc_wakeup_event=0;


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

int32_t delay_us(uint32_t nus)
{
	uint32_t temp;

	SysTick->CTRL = 0; 					
	SysTick->LOAD = (nus*21)-1; 		
	SysTick->VAL = 0; 					
	SysTick->CTRL = 1; 					
	
	while(1)
	{
	
		temp=SysTick->CTRL;
		
		//检测count flag
		if(temp & 0x00010000)
			break;
		
		//检测系统定时器是否意外关闭	
		if((temp & 0x1)==0)
			return -1;		
	}
	
	SysTick->CTRL = 0; 					

	return 0;
}



int32_t  delay_ms(uint32_t nms)
{
	uint32_t t = nms;

	uint32_t temp;
	
	
	while(t--)
	{
		SysTick->CTRL = 0; 			
		SysTick->LOAD = 21000-1; 	
		SysTick->VAL = 0; 			
		SysTick->CTRL = 1; 			
		while(1)
		{

			temp=SysTick->CTRL;
			
			//检测count flag
			if(temp & 0x00010000)
				break;
			
			//检测系统定时器是否意外关闭	
			if((temp & 0x1)==0)
				return -1;		
		}
	}	
	
	SysTick->CTRL = 0; 	
	
	return 0;
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

void rtc_init(void)
{

	/* Enable the PWR clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	
	/* Allow access to RTC，RTC的电源开关使能 */
	PWR_BackupAccessCmd(ENABLE);

	//使能LSE（32.768KHz）
	RCC_LSEConfig(RCC_LSE_ON);
	
	/* Wait till LSE is ready ，等待LSE就绪*/  
	while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
	{
	}
	
	/* Select the RTC Clock Source，为RTC选中的时钟源为LSE */
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);


	/* Enable the RTC Clock，使能RTC的硬件时钟 */
	RCC_RTCCLKCmd(ENABLE);
	
	/* Wait for RTC APB registers synchronisation ，等待RTC相关寄存器就绪*/
	RTC_WaitForSynchro();
	
	/* Configure the RTC data register and RTC prescaler */
	/* ck_spre(1Hz) = RTCCLK(LSE) /((uwAsynchPrediv + 1)*(uwSynchPrediv + 1))*/

	//RTC的硬件时钟=32768Hz/(0x7F+1)/(0xFF+1)=1Hz
	RTC_InitStructure.RTC_AsynchPrediv = 0x7F;
	RTC_InitStructure.RTC_SynchPrediv = 0xFF;
	RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;//24小时格式
	RTC_Init(&RTC_InitStructure);


	/* Set the date: Monday February 22th 2021 */
	RTC_DateStructure.RTC_Year = 0x21;
	RTC_DateStructure.RTC_Month = RTC_Month_February;
	RTC_DateStructure.RTC_Date = 0x22;
	RTC_DateStructure.RTC_WeekDay = RTC_Weekday_Monday;
	RTC_SetDate(RTC_Format_BCD, &RTC_DateStructure);
	
	/* Set the time to 16h 28mn 30s PM， */
	RTC_TimeStructure.RTC_H12     = RTC_H12_PM;
	RTC_TimeStructure.RTC_Hours   = 0x18;
	RTC_TimeStructure.RTC_Minutes = 0x59;
	RTC_TimeStructure.RTC_Seconds = 0x50; 
	RTC_SetTime(RTC_Format_BCD, &RTC_TimeStructure);


	//关闭唤醒功能
	RTC_WakeUpCmd(DISABLE);
	
	//为唤醒功能选择RTC配置好的时钟源
	RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);
	
	//设置唤醒计数值为自动重载，写入值默认是0
	RTC_SetWakeUpCounter(1-1);
	
	//清除RTC唤醒中断标志
	RTC_ClearITPendingBit(RTC_IT_WUT);
	
	//使能RTC唤醒中断
	RTC_ITConfig(RTC_IT_WUT, ENABLE);

	//使能唤醒功能
	RTC_WakeUpCmd(ENABLE);

	/* Configure EXTI Line22，配置外部中断控制线22 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line22;			//当前使用外部中断控制线22
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		//中断模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;		//上升沿触发中断 
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;			//使能外部中断控制线22
	EXTI_Init(&EXTI_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;		//允许RTC唤醒中断触发
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;	//抢占优先级为0x3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;		//响应优先级为0x3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//使能
	NVIC_Init(&NVIC_InitStructure);

}

void rtc_alarm_init(void)
{
	/* 允许RTC的A闹钟触发中断 */
	RTC_ITConfig(RTC_IT_ALRA, ENABLE);
	
	/* 清空标志位 */
	RTC_ClearFlag(RTC_FLAG_ALRAF);

	/*使能外部中断控制线17的中断*/
	EXTI_ClearITPendingBit(EXTI_Line17);
	EXTI_InitStructure.EXTI_Line = EXTI_Line17;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	/*使能闹钟的中断 */
	NVIC_InitStructure.NVIC_IRQChannel = RTC_Alarm_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void rtc_alarm_set(RTC_AlarmTypeDef RTC_AlarmStructure)
{
	/* 关闭闹钟，若不关闭，配置闹钟触发的中断有BUG，无论怎么配置，只要到00秒，则触发中断*/
	RTC_AlarmCmd(RTC_Alarm_A, DISABLE);
	
	/* 配置RTC的A闹钟，注：RTC的闹钟有两个，分别为闹钟A与闹钟B */
	RTC_SetAlarm(RTC_Format_BCD, RTC_Alarm_A, &RTC_AlarmStructure);
	
	/* 让RTC的闹钟A工作*/
	RTC_AlarmCmd(RTC_Alarm_A, ENABLE);
}

int main(void)
{
	//串口1初始化波特率为115200bps
	usart1_init(115200);
	
	//每次复位的时候，都取读取备份寄存器0
	if(0x1688!=RTC_ReadBackupRegister(RTC_BKP_DR0))
	{
		//rtc的初始化
		rtc_init();
	
		//往备份寄存器0写入数据为0x1688
		RTC_WriteBackupRegister(RTC_BKP_DR0, 0x1688);	
	}
	else
	{
		/* Enable the PWR clock */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
		
		/* Allow access to RTC，RTC的电源开关使能 */
		PWR_BackupAccessCmd(ENABLE);

		//使能LSE（32.768KHz）
		RCC_LSEConfig(RCC_LSE_ON);
		
		/* Wait till LSE is ready ，等待LSE就绪*/  
		while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
		{
		}
		
		/* Select the RTC Clock Source，为RTC选中的时钟源为LSE */
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);


		/* Enable the RTC Clock，使能RTC的硬件时钟 */
		RCC_RTCCLKCmd(ENABLE);
		
		/* Wait for RTC APB registers synchronisation ，等待RTC相关寄存器就绪*/
		RTC_WaitForSynchro();
		
		
		//关闭唤醒功能
		RTC_WakeUpCmd(DISABLE);
		
		//为唤醒功能选择RTC配置好的时钟源
		RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);
		
		//设置唤醒计数值为自动重载，写入值默认是0
		RTC_SetWakeUpCounter(1-1);
		
		//清除RTC唤醒中断标志
		RTC_ClearITPendingBit(RTC_IT_WUT);
		
		//使能RTC唤醒中断
		RTC_ITConfig(RTC_IT_WUT, ENABLE);

		//使能唤醒功能
		RTC_WakeUpCmd(ENABLE);

		/* Configure EXTI Line22，配置外部中断控制线22 */
		EXTI_InitStructure.EXTI_Line = EXTI_Line22;			//当前使用外部中断控制线22
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		//中断模式
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;		//上升沿触发中断 
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;			//使能外部中断控制线22
		EXTI_Init(&EXTI_InitStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;		//允许RTC唤醒中断触发
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;	//抢占优先级为0x3
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;		//响应优先级为0x3
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//使能
		NVIC_Init(&NVIC_InitStructure);	
	}
	
	rtc_alarm_init();
	
	printf("This is rtc test\r\n");	
	
	// 配置闹钟
    RTC_AlarmStructure.RTC_AlarmTime.RTC_H12 = RTC_H12_PM;
    RTC_AlarmStructure.RTC_AlarmTime.RTC_Hours = 0x19 ;
    RTC_AlarmStructure.RTC_AlarmTime.RTC_Minutes = 0x00;
    RTC_AlarmStructure.RTC_AlarmTime.RTC_Seconds = 0x00;
	RTC_AlarmStructure.RTC_AlarmDateWeekDay = RTC_Weekday_Monday;		//星期一
	RTC_AlarmStructure.RTC_AlarmDateWeekDaySel = RTC_AlarmDateWeekDaySel_WeekDay;//指定星期几生效
	RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_None;				//不屏蔽哪一天和星期的配置   
    rtc_alarm_set(RTC_AlarmStructure);	
	
	while (1)
	{
		if(g_rtc_wakeup_event)
		{
			//获取日期
			RTC_GetDate(RTC_Format_BCD,&RTC_DateStructure);
			printf("20%02x/%02x/%02x Week%x\r\n",RTC_DateStructure.RTC_Year,RTC_DateStructure.RTC_Month,RTC_DateStructure.RTC_Date,RTC_DateStructure.RTC_WeekDay);

			//获取时间
			RTC_GetTime(RTC_Format_BCD,&RTC_TimeStructure);
			printf("%02x:%02x:%02x\r\n\n",RTC_TimeStructure.RTC_Hours,RTC_TimeStructure.RTC_Minutes,RTC_TimeStructure.RTC_Seconds);

			g_rtc_wakeup_event=0;
		}
	}
}

void RTC_WKUP_IRQHandler(void)
{
	if(RTC_GetITStatus(RTC_IT_WUT) != RESET)
	{
		g_rtc_wakeup_event=1;
		RTC_ClearITPendingBit(RTC_IT_WUT);
		EXTI_ClearITPendingBit(EXTI_Line22);
	} 
}

void RTC_Alarm_IRQHandler(void)
{    
	if(RTC_GetFlagStatus(RTC_FLAG_ALRAF)==SET)//ALARM A中断
	{
		RTC_ClearFlag(RTC_FLAG_ALRAF);//清除中断标志
		printf("ALARM A!\r\n");
		printf("ALARM A!\r\n");
		printf("ALARM A!\r\n");
		printf("ALARM A!\r\n");
		printf("ALARM A!\r\n");
		printf("ALARM A!\r\n");
		printf("ALARM A!\r\n");
	}   
	EXTI_ClearITPendingBit(EXTI_Line17);	//清除中断线17的中断标志 											 
}

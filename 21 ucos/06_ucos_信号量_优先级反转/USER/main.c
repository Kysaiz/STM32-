#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "includes.h"


static EXTI_InitTypeDef   EXTI_InitStructure;
static GPIO_InitTypeDef   GPIO_InitStructure;
static NVIC_InitTypeDef   NVIC_InitStructure;

//任务L控制块
OS_TCB TaskL_TCB;

void taskL(void *parg);

CPU_STK taskL_stk[128];			//任务L的任务堆栈，大小为128字，也就是512字节



//任务M控制块
OS_TCB TaskM_TCB;

void taskM(void *parg);

CPU_STK taskM_stk[128];			//任务M的任务堆栈，大小为128字，也就是512字节


//任务H控制块
OS_TCB TaskH_TCB;

void taskH(void *parg);

CPU_STK taskH_stk[128];			//任务H的任务堆栈，大小为128字，也就是512字节

OS_SEM	g_sem;					//信号量

OS_Q	g_queue;				//消息队列

OS_FLAG_GRP	g_flag_grp;			//事件标志组

void res(void)
{

	volatile uint32_t i=0xF000000;
	
	while(i--);
}

void exti0_init(void)
{
	//打开端口A硬件时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	

	//打开SYSCFG硬件时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;			//引脚配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;		//配置模式为输入
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;	//配置速率为高速
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		//配置为推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	//上下拉电阻不使能
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	//将EXTI0连接到PA0
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
	
	
	//配置EXTI0的触发方式
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;				//外部中断线0
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		//中断模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 	//下降沿触发，用于检测按键的按下
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	//配置EXTI0的优先级
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//EXTI0的中断号
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级0x00
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;		//响应优先级0x02
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//使能EXTI0的通道
	NVIC_Init(&NVIC_InitStructure);
}

//主函数
int main(void)
{
	OS_ERR err;

	systick_init();  													//时钟初始化
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);						//中断分组配置
	
	usart_init(9600);  				 									//串口初始化
	
	LED_Init();         												//LED初始化	
	
	exti0_init();

	//OS初始化，它是第一个运行的函数,初始化各种的全局变量，例如中断嵌套计数器、优先级、存储器
	OSInit(&err);


	//创建任务L
	OSTaskCreate(	(OS_TCB *)&TaskL_TCB,									//任务控制块，等同于线程id
					(CPU_CHAR *)"TaskL",									//任务的名字，名字可以自定义的
					(OS_TASK_PTR)taskL,										//任务函数，等同于线程函数
					(void *)0,												//传递参数，等同于线程的传递参数
					(OS_PRIO)9,											 	//任务的优先级9		
					(CPU_STK *)taskL_stk,									//任务堆栈基地址
					(CPU_STK_SIZE)128/10,									//任务堆栈深度限位，用到这个位置，任务不能再继续使用
					(CPU_STK_SIZE)128,										//任务堆栈大小			
					(OS_MSG_QTY)0,											//禁止任务消息队列
					(OS_TICK)0,												//默认时间片长度																
					(void  *)0,												//不需要补充用户存储区
					(OS_OPT)OS_OPT_TASK_NONE,								//没有任何选项
					&err													//返回的错误码
				);
					
	if(err!=OS_ERR_NONE)
	{
		printf("task L create fail\r\n");
		
		while(1);
	
	}


	//创建任务M
	OSTaskCreate(	(OS_TCB *)&TaskM_TCB,									//任务控制块
					(CPU_CHAR *)"TaskM",									//任务的名字
					(OS_TASK_PTR)taskM,										//任务函数
					(void *)0,												//传递参数
					(OS_PRIO)8,											 	//任务的优先级8		
					(CPU_STK *)taskM_stk,									//任务堆栈基地址
					(CPU_STK_SIZE)128/10,									//任务堆栈深度限位，用到这个位置，任务不能再继续使用
					(CPU_STK_SIZE)128,										//任务堆栈大小			
					(OS_MSG_QTY)0,											//禁止任务消息队列
					(OS_TICK)0,												//默认时间片长度																
					(void  *)0,												//不需要补充用户存储区
					(OS_OPT)OS_OPT_TASK_NONE,								//没有任何选项
					&err													//返回的错误码
				);
					
	if(err!=OS_ERR_NONE)
	{
		printf("task M create fail\r\n");
		
		while(1);
	
	}
	
	
	
	//创建任务H
	OSTaskCreate(	(OS_TCB *)&TaskH_TCB,									//任务控制块
					(CPU_CHAR *)"TaskH",									//任务的名字
					(OS_TASK_PTR)taskH,										//任务函数
					(void *)0,												//传递参数
					(OS_PRIO)7,											 	//任务的优先级7		
					(CPU_STK *)taskH_stk,									//任务堆栈基地址
					(CPU_STK_SIZE)128/10,									//任务堆栈深度限位，用到这个位置，任务不能再继续使用
					(CPU_STK_SIZE)128,										//任务堆栈大小			
					(OS_MSG_QTY)0,											//禁止任务消息队列
					(OS_TICK)0,												//默认时间片长度																
					(void  *)0,												//不需要补充用户存储区
					(OS_OPT)OS_OPT_TASK_NONE,								//没有任何选项
					&err													//返回的错误码
				);
					
	if(err!=OS_ERR_NONE)
	{
		printf("task H create fail\r\n");
		
		while(1);
	
	}		
	
	//创建信号量，初值为1.思考为什么不写初值为0
	OSSemCreate(&g_sem,"g_sem",1,&err);
	
	
	//创建事件标志组，所有标志位初值为0
	OSFlagCreate(&g_flag_grp,"g_flag_grp",0,&err);


	//创建消息队列，支持6条消息，就支持6个消息指针
	OSQCreate(&g_queue,"g_queue",6,&err);	

	//启动OS，进行任务调度
	OSStart(&err);
						
	printf(".......\r\n");
					
	while(1);
	
}


void taskL(void *parg)
{

	OS_ERR err;

	printf("taskL is create ok\r\n");

	while(1)
	{
	
		
		OSSemPend(&g_sem,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		
		printf("[taskL]:access res begin\r\n");
		res();
		printf("[taskL]:access res end\r\n");
		
		OSSemPost(&g_sem,OS_OPT_POST_1,&err);
		

		delay_ms(50);
		
		
	}
}


void taskM(void *parg)
{

	OS_ERR err;
	
	OS_FLAGS  flags=0;

	printf("taskM is create ok\r\n");

	while(1)
	{
		
		flags=OSFlagPend(&g_flag_grp,0x01,0,OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME+OS_OPT_PEND_BLOCKING,NULL,&err);
				
		
		if(flags & 0x01)
		{
			printf("[taskM]:key set\r\n");
			
		}
		
	}
}



void taskH(void *parg)
{

	OS_ERR err;
	
	OS_MSG_SIZE msg_size;
	
	char *p=NULL;
	
	
	printf("taskH is create ok\r\n");

	while(1)
	{
		
		p=OSQPend(&g_queue,0,OS_OPT_PEND_BLOCKING,&msg_size,NULL,&err);
		
		
		if(p && msg_size)
		{
			//将得到的数据内容和数据大小进行打印
			printf("[taskH]:queue msg[%s],len[%d]\r\n",p,msg_size);
			
			//清空指向消息的内容
			memset(p,0,msg_size);
		
		}

		
		OSSemPend(&g_sem,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		printf("[taskH]:access res begin\r\n");
		res();
		printf("[taskH]:access res end\r\n");
		OSSemPost(&g_sem,OS_OPT_POST_1,&err);
	}
}



//EXTI0的中断服务函数
void EXTI0_IRQHandler(void)
{
	uint32_t b=0;
	OS_ERR err;
	
	OSIntEnter();
	
	//检测EXTI0是否有中断请求
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		b=1;

		EXTI_ClearITPendingBit(EXTI_Line0);
	}
	
	OSIntExit();
	
	if(b)
	{
		//对事件标志组的bit0置位(1)
		OSFlagPost(&g_flag_grp,0x01,OS_OPT_POST_FLAG_SET,&err);
	
	}
}








#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "includes.h"

//任务1控制块
OS_TCB Task1_TCB;

void task1(void *parg);

CPU_STK task1_stk[128];			//任务1的任务堆栈，大小为128字，也就是512字节



//任务2控制块
OS_TCB Task2_TCB;

void task2(void *parg);

CPU_STK task2_stk[128];			//任务2的任务堆栈，大小为128字，也就是512字节




OS_FLAG_GRP	g_flag_grp;		//事件标志组的对象


//主函数
int main(void)
{
	OS_ERR err;

	systick_init();  													//时钟初始化
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);						//中断分组配置
	
	usart_init(9600);  				 									//串口初始化
	
	LED_Init();         												//LED初始化	


	//OS初始化，它是第一个运行的函数,初始化各种的全局变量，例如中断嵌套计数器、优先级、存储器
	OSInit(&err);


	//创建任务1
	OSTaskCreate(	(OS_TCB *)&Task1_TCB,									//任务控制块，等同于线程id
					(CPU_CHAR *)"Task1",									//任务的名字，名字可以自定义的
					(OS_TASK_PTR)task1,										//任务函数，等同于线程函数
					(void *)0,												//传递参数，等同于线程的传递参数
					(OS_PRIO)6,											 	//任务的优先级6		
					(CPU_STK *)task1_stk,									//任务堆栈基地址
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
		printf("task 1 create fail\r\n");
		
		while(1);
	
	}


	//创建任务2
	OSTaskCreate(	(OS_TCB *)&Task2_TCB,									//任务控制块
					(CPU_CHAR *)"Task2",									//任务的名字
					(OS_TASK_PTR)task2,										//任务函数
					(void *)0,												//传递参数
					(OS_PRIO)6,											 	//任务的优先级7		
					(CPU_STK *)task2_stk,									//任务堆栈基地址
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
		printf("task 2 create fail\r\n");
		
		while(1);
	
	}
	
	//创建事件标志组，标志组里面32个标志位为0
	OSFlagCreate(&g_flag_grp,"g_flag_grp",0,&err);

	//启动OS，进行任务调度
	OSStart(&err);
					
					
	printf(".......\r\n");
					
	while(1);
	
}


void task1(void *parg)
{
	OS_FLAGS   flags=0;
	
	OS_ERR err;
	
	char *p=NULL;

	printf("task1 is create ok\r\n");

	while(1)
	{
		//等待事件标志组
		//0x01,等待bit0
		//OS_OPT_PEND_FLAG_SET_ANY,等待事件标志组中对应的标志位其中一个置1
		//OS_OPT_PEND_FLAG_CONSUME，置1完毕后，帮忙做清零动作
		
		flags=OSFlagPend(&g_flag_grp,0x01|0x02,0,OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME+OS_OPT_PEND_BLOCKING,NULL,&err);
		
		
		if(flags & 0x01)
		{
			printf("bit0 set\r\n");
			
			

		}

		if(flags & 0x02)
		{
			printf("bit1 set\r\n");
			
			

		}
		
	}
}

void task2(void *parg)
{
	OS_ERR err;
	

	
	printf("task2 is create ok\r\n");

	while(1)
	{
		//对事件标志组的bit0置1
	
		OSFlagPost (&g_flag_grp,0x01,OS_OPT_POST_FLAG_SET,&err);

		delay_ms(1000);
		
		
		//对事件标志组的bit1置1
	
		OSFlagPost (&g_flag_grp,0x02,OS_OPT_POST_FLAG_SET,&err);

		delay_ms(1000);
	}
}









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

OS_Q	g_queue;				//消息队列的对象


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
	
	//支持16条消息，也就是最多能够存储16个指针而已
	OSQCreate(&g_queue,"g_queue",16,&err);

	//启动OS，进行任务调度
	OSStart(&err);
					
					
	printf(".......\r\n");
					
	while(1);
	
}


void task1(void *parg)
{
	OS_MSG_SIZE msg_size;
	
	OS_ERR err;
	
	char *p=NULL;

	printf("task1 is create ok\r\n");

	while(1)
	{
		//等待消息队列
		//msg_size：获取到消息的大小
		p=OSQPend(&g_queue,0,OS_OPT_PEND_BLOCKING,&msg_size,NULL,&err);
		
		
		if(p && msg_size)
		{
			printf("p[%s],msg_size[%d]\r\n",p,msg_size);
			
			memset(p,0,msg_size);

		}


		
	}
}

void task2(void *parg)
{
	OS_ERR err;
	
	char buf[64]={0};
	
	uint32_t i=0;
	
	printf("task2 is create ok\r\n");

	while(1)
	{
		i++;
		
		sprintf(buf,"task2 run times %d",i);
		
		//发送消息，先进先出
		OSQPost(&g_queue,&buf,strlen(buf),OS_OPT_POST_FIFO,&err);
		
		printf("task2 is running ...\r\n");


		delay_ms(1000);
	}
}









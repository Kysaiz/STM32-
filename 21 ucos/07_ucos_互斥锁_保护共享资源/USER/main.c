#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "includes.h"

//����1���ƿ�
OS_TCB Task1_TCB;

void task1(void *parg);

CPU_STK task1_stk[128];			//����1�������ջ����СΪ128�֣�Ҳ����512�ֽ�



//����2���ƿ�
OS_TCB Task2_TCB;

void task2(void *parg);

CPU_STK task2_stk[128];			//����2�������ջ����СΪ128�֣�Ҳ����512�ֽ�


//����3���ƿ�
OS_TCB  Task3_TCB;

void task3(void *parg);

CPU_STK task3_stk[128];			//����3�������ջ����СΪ128�֣�Ҳ����512�ֽ�


//�������Ķ���
OS_MUTEX 		g_mutex;


//������
int main(void)
{
	OS_ERR err;

	systick_init();  													//ʱ�ӳ�ʼ��
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);						//�жϷ�������
	
	usart_init(9600);  				 									//���ڳ�ʼ��
	
	LED_Init();         												//LED��ʼ��	


	//OS��ʼ�������ǵ�һ�����еĺ���,��ʼ�����ֵ�ȫ�ֱ����������ж�Ƕ�׼����������ȼ����洢��
	OSInit(&err);


	//��������1
	OSTaskCreate(	(OS_TCB *)&Task1_TCB,									//������ƿ飬��ͬ���߳�id
					(CPU_CHAR *)"Task1",									//��������֣����ֿ����Զ����
					(OS_TASK_PTR)task1,										//����������ͬ���̺߳���
					(void *)0,												//���ݲ�������ͬ���̵߳Ĵ��ݲ���
					(OS_PRIO)6,											 	//��������ȼ�6		
					(CPU_STK *)task1_stk,									//�����ջ����ַ
					(CPU_STK_SIZE)128/10,									//�����ջ�����λ���õ����λ�ã��������ټ���ʹ��
					(CPU_STK_SIZE)128,										//�����ջ��С			
					(OS_MSG_QTY)0,											//��ֹ������Ϣ����
					(OS_TICK)0,												//Ĭ��ʱ��Ƭ����																
					(void  *)0,												//����Ҫ�����û��洢��
					(OS_OPT)OS_OPT_TASK_NONE,								//û���κ�ѡ��
					&err													//���صĴ�����
				);
					
	if(err!=OS_ERR_NONE)
	{
		printf("task 1 create fail\r\n");
		
		while(1);
	
	}


	//��������2
	OSTaskCreate(	(OS_TCB *)&Task2_TCB,									//������ƿ�
					(CPU_CHAR *)"Task2",									//���������
					(OS_TASK_PTR)task2,										//������
					(void *)0,												//���ݲ���
					(OS_PRIO)7,											 	//��������ȼ�7		
					(CPU_STK *)task2_stk,									//�����ջ����ַ
					(CPU_STK_SIZE)128/10,									//�����ջ�����λ���õ����λ�ã��������ټ���ʹ��
					(CPU_STK_SIZE)128,										//�����ջ��С			
					(OS_MSG_QTY)0,											//��ֹ������Ϣ����
					(OS_TICK)0,												//Ĭ��ʱ��Ƭ����																
					(void  *)0,												//����Ҫ�����û��洢��
					(OS_OPT)OS_OPT_TASK_NONE,								//û���κ�ѡ��
					&err													//���صĴ�����
				);
					
	if(err!=OS_ERR_NONE)
	{
		printf("task 2 create fail\r\n");
		
		while(1);
	
	}
	
	
	
	//����������
	
	OSMutexCreate(&g_mutex,"g_mutex",&err);

	//����OS�������������
	OSStart(&err);
					
					
	printf(".......\r\n");
					
	while(1);
	
}


void task1(void *parg)
{
	
	OS_ERR	err;
	printf("task1 is create ok\r\n");

	while(1)
	{
		//0��һֱ�ȴ���OS_OPT_PEND_BLOCKING��������ʽȥ�ȴ�
		OSMutexPend(&g_mutex,0,OS_OPT_PEND_BLOCKING,NULL,&err); 
		printf("task1 is running ...\r\n");
		
		//�ͷŻ������������ȴ�������
		OSMutexPost(&g_mutex,OS_OPT_POST_NONE,&err); 

		delay_ms(20);
	}
}

void task2(void *parg)
{
	OS_ERR	err;
	
	printf("task2 is create ok\r\n");

	while(1)
	{
		//0��һֱ�ȴ���OS_OPT_PEND_BLOCKING��������ʽȥ�ȴ�
		OSMutexPend(&g_mutex,0,OS_OPT_PEND_BLOCKING,NULL,&err); 
		printf("task2 is running ...\r\n");
		
		//�ͷŻ������������ȴ�������
		OSMutexPost(&g_mutex,OS_OPT_POST_NONE,&err);
		
		delay_ms(50);

	}
}









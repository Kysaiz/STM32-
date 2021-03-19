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

OS_Q	g_queue;				//��Ϣ���еĶ���
OS_SEM	g_sem;					//�ź����ĵĶ���

OS_PEND_DATA  	pend_tbl[2];		//����ں˶��������

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
					(OS_PRIO)6,											 	//��������ȼ�7		
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
	
	//�����ź���,����ֵ��ֵΪ0
	
	OSSemCreate(&g_sem,"g_sem",0,&err);

	
	//֧��16����Ϣ��Ҳ��������ܹ��洢16��ָ�����
	OSQCreate(&g_queue,"g_queue",16,&err);
	
	
	//Ϊpend_tbl���Ҫ�ȴ����ں˶���
	pend_tbl[0].PendObjPtr=(OS_PEND_OBJ *)&g_sem;
	pend_tbl[1].PendObjPtr=(OS_PEND_OBJ *)&g_queue;
	//����OS�������������
	OSStart(&err);
					
					
	printf(".......\r\n");
					
	while(1);
	
}


void task1(void *parg)
{
	OS_MSG_SIZE msg_size;
	
	OS_ERR err;
	
	OS_OBJ_QTY obj_num=0;//�ж��ٸ��ں˶������
	

	printf("task1 is create ok\r\n");

	while(1)
	{

		obj_num=OSPendMulti(pend_tbl,2,0,OS_OPT_PEND_BLOCKING,&err);
		
		
		if(obj_num == 0)
			continue;
		
		//�ж��Ƿ�õ��ź���
		if(pend_tbl[0].RdyObjPtr == (OS_PEND_OBJ *)&g_sem)
		{
			printf("sem get\r\n");
		
		}
		if(pend_tbl[1].RdyObjPtr == (OS_PEND_OBJ *)&g_queue)
		{
			if(pend_tbl[1].RdyMsgPtr && pend_tbl[1].RdyMsgSize)
			{
				printf("p[%s],msg_size[%d]\r\n",pend_tbl[1].RdyMsgPtr,pend_tbl[1].RdyMsgSize);
				
				memset(pend_tbl[1].RdyMsgPtr,0,pend_tbl[1].RdyMsgSize);

			}			
		
		}		



		
	}
}

void task2(void *parg)
{
	OS_ERR err;
	

	
	printf("task2 is create ok\r\n");

	while(1)
	{
		OSSemPost(&g_sem,OS_OPT_POST_1,&err); 
		
		printf("task2 is running ...\r\n");
		
		delay_ms(1000);
	}
}









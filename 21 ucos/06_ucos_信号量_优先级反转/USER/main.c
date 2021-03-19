#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "includes.h"


static EXTI_InitTypeDef   EXTI_InitStructure;
static GPIO_InitTypeDef   GPIO_InitStructure;
static NVIC_InitTypeDef   NVIC_InitStructure;

//����L���ƿ�
OS_TCB TaskL_TCB;

void taskL(void *parg);

CPU_STK taskL_stk[128];			//����L�������ջ����СΪ128�֣�Ҳ����512�ֽ�



//����M���ƿ�
OS_TCB TaskM_TCB;

void taskM(void *parg);

CPU_STK taskM_stk[128];			//����M�������ջ����СΪ128�֣�Ҳ����512�ֽ�


//����H���ƿ�
OS_TCB TaskH_TCB;

void taskH(void *parg);

CPU_STK taskH_stk[128];			//����H�������ջ����СΪ128�֣�Ҳ����512�ֽ�

OS_SEM	g_sem;					//�ź���

OS_Q	g_queue;				//��Ϣ����

OS_FLAG_GRP	g_flag_grp;			//�¼���־��

void res(void)
{

	volatile uint32_t i=0xF000000;
	
	while(i--);
}

void exti0_init(void)
{
	//�򿪶˿�AӲ��ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	

	//��SYSCFGӲ��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;			//��������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;		//����ģʽΪ����
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;	//��������Ϊ����
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;		//����Ϊ�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;	//���������費ʹ��
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	//��EXTI0���ӵ�PA0
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
	
	
	//����EXTI0�Ĵ�����ʽ
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;				//�ⲿ�ж���0
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;		//�ж�ģʽ
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 	//�½��ش��������ڼ�ⰴ���İ���
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
	//����EXTI0�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;			//EXTI0���жϺ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//��ռ���ȼ�0x00
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;		//��Ӧ���ȼ�0x02
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ʹ��EXTI0��ͨ��
	NVIC_Init(&NVIC_InitStructure);
}

//������
int main(void)
{
	OS_ERR err;

	systick_init();  													//ʱ�ӳ�ʼ��
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);						//�жϷ�������
	
	usart_init(9600);  				 									//���ڳ�ʼ��
	
	LED_Init();         												//LED��ʼ��	
	
	exti0_init();

	//OS��ʼ�������ǵ�һ�����еĺ���,��ʼ�����ֵ�ȫ�ֱ����������ж�Ƕ�׼����������ȼ����洢��
	OSInit(&err);


	//��������L
	OSTaskCreate(	(OS_TCB *)&TaskL_TCB,									//������ƿ飬��ͬ���߳�id
					(CPU_CHAR *)"TaskL",									//��������֣����ֿ����Զ����
					(OS_TASK_PTR)taskL,										//����������ͬ���̺߳���
					(void *)0,												//���ݲ�������ͬ���̵߳Ĵ��ݲ���
					(OS_PRIO)9,											 	//��������ȼ�9		
					(CPU_STK *)taskL_stk,									//�����ջ����ַ
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
		printf("task L create fail\r\n");
		
		while(1);
	
	}


	//��������M
	OSTaskCreate(	(OS_TCB *)&TaskM_TCB,									//������ƿ�
					(CPU_CHAR *)"TaskM",									//���������
					(OS_TASK_PTR)taskM,										//������
					(void *)0,												//���ݲ���
					(OS_PRIO)8,											 	//��������ȼ�8		
					(CPU_STK *)taskM_stk,									//�����ջ����ַ
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
		printf("task M create fail\r\n");
		
		while(1);
	
	}
	
	
	
	//��������H
	OSTaskCreate(	(OS_TCB *)&TaskH_TCB,									//������ƿ�
					(CPU_CHAR *)"TaskH",									//���������
					(OS_TASK_PTR)taskH,										//������
					(void *)0,												//���ݲ���
					(OS_PRIO)7,											 	//��������ȼ�7		
					(CPU_STK *)taskH_stk,									//�����ջ����ַ
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
		printf("task H create fail\r\n");
		
		while(1);
	
	}		
	
	//�����ź�������ֵΪ1.˼��Ϊʲô��д��ֵΪ0
	OSSemCreate(&g_sem,"g_sem",1,&err);
	
	
	//�����¼���־�飬���б�־λ��ֵΪ0
	OSFlagCreate(&g_flag_grp,"g_flag_grp",0,&err);


	//������Ϣ���У�֧��6����Ϣ����֧��6����Ϣָ��
	OSQCreate(&g_queue,"g_queue",6,&err);	

	//����OS�������������
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
			//���õ����������ݺ����ݴ�С���д�ӡ
			printf("[taskH]:queue msg[%s],len[%d]\r\n",p,msg_size);
			
			//���ָ����Ϣ������
			memset(p,0,msg_size);
		
		}

		
		OSSemPend(&g_sem,0,OS_OPT_PEND_BLOCKING,NULL,&err);
		printf("[taskH]:access res begin\r\n");
		res();
		printf("[taskH]:access res end\r\n");
		OSSemPost(&g_sem,OS_OPT_POST_1,&err);
	}
}



//EXTI0���жϷ�����
void EXTI0_IRQHandler(void)
{
	uint32_t b=0;
	OS_ERR err;
	
	OSIntEnter();
	
	//���EXTI0�Ƿ����ж�����
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		b=1;

		EXTI_ClearITPendingBit(EXTI_Line0);
	}
	
	OSIntExit();
	
	if(b)
	{
		//���¼���־���bit0��λ(1)
		OSFlagPost(&g_flag_grp,0x01,OS_OPT_POST_FLAG_SET,&err);
	
	}
}








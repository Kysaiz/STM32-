#ifndef __USART3_H
#define __USART3_H
#include "stm32f4xx.h"
#include "sys.h" 


// ����3��ʼ��
extern void usart3_init(uint32_t baud);

// �������ݵ��ֻ�
extern void usart3_send_str(char *str);

// ���������ֻ�������ָ��
extern void USART3_IRQHandler(void);

// ��������״̬�жϳ�ʼ��
extern void usart3_EXTI4_init(void);


#endif

#ifndef __SYSTICK_H
#define __SYSTICK_H


#include "stm32f10x.h"

void SystickInit(void);

/**
***********************************************************
* @brief ע��������Ȼص�����
* @param pFunc, ����ص�������ַ
* @return 
***********************************************************
*/
void TaskScheduleCbReg(void (*pFunc)(void));
/**
***********************************************************
* @brief ��ȡϵͳ����ʱ��
* @param
* @return ��1msΪ��λ
***********************************************************
*/
uint64_t GetSysRunTime(void);


#endif /* __SYSTICK_H */

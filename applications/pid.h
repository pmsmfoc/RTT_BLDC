/**
 ****************************************************************************************************
 * @file        pid.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       PID����
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� STM32F407���������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 * �޸�˵��
 * V1.0 20211014
 * ��һ�η���
 *
 ****************************************************************************************************
 */
#ifndef __PID_H
#define __PID_H

#include "stm32f4xx_hal.h"

/******************************************************************************************/
/* PID��ز��� */

#define  INCR_LOCT_SELECT  0        /* 0��ѡ��λ��ʽ  1������ʽ���� */

#if INCR_LOCT_SELECT
#define  S_KP      0.00800f         /* P���� */
#define  S_KI      0.00025f         /* I���� */
#define  S_KD      0.00020f         /* D���� */
#else
#define  S_KP      0.00800f         /* P���� */
#define  S_KI      0.00025f         /* I���� */
#define  S_KD      0.00020f         /* D���� */
#endif

#define SMAPLSE_PID_SPEED  40       /* ������ ��λms */

/*����λ��PID������غ�*/
/*PID�ṹ��*/
typedef struct
{
    __IO float  SetPoint;           /* �趨Ŀ�� */
    __IO float  ActualValue;        /* ����ֵ */
    __IO float  SumError;           /* ����ۼ� */
    __IO float  Proportion;         /* �������� P */
    __IO float  Integral;           /* ���ֳ��� I */
    __IO float  Derivative;         /* ΢�ֳ��� D */
    __IO float  Error;              /* Error[-1] */
    __IO float  LastError;          /* Error[-1] */
    __IO float  PrevError;          /* Error[-2] */
    __IO float  IngMin;
    __IO float  IngMax;
    __IO float  OutMin;
    __IO float  OutMax;
} PID_TypeDef;

extern PID_TypeDef  g_location_pid; /* λ��PID�����ṹ�� */
extern PID_TypeDef  g_speed_pid;    /* �ٶ�PID�����ṹ�� */
/******************************************************************************************/
/* �ⲿ�ӿں���*/
void pid_init(void);                                                    /* PID��ʼ�� */
int32_t increment_pid_ctrl(PID_TypeDef *PID,float Feedback_value);      /* PID�����㷨 */
#endif

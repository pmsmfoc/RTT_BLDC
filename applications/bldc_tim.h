/**
 ****************************************************************************************************
 * @file        bldc_tim.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-19
 * @brief       ��ʱ�� ��������
 * @license     Copyright (c) 2020-2032, ������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� F407���������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 * �޸�˵��
 * V1.0 20211019
 * ��һ�η���
 *
 ****************************************************************************************************
 */

#ifndef __BLDC_TIM_H
#define __BLDC_TIM_H

#include "stm32f4xx_hal.h"

/************************************** �߼���ʱ�� ���� **************************************/

extern TIM_HandleTypeDef g_atimx_handle;                                                       /* ��ʱ��x��� */

 /* ��ͨ��IO�궨�� */
#define ATIM_TIMX_PWM_CH1_GPIO_PORT            GPIOA
#define ATIM_TIMX_PWM_CH1_GPIO_PIN             GPIO_PIN_8
#define ATIM_TIMX_PWM_CH1_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)    /* PA��ʱ��ʹ�� */

#define ATIM_TIMX_PWM_CH2_GPIO_PORT            GPIOA
#define ATIM_TIMX_PWM_CH2_GPIO_PIN             GPIO_PIN_9
#define ATIM_TIMX_PWM_CH2_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)    /* PA��ʱ��ʹ�� */

#define ATIM_TIMX_PWM_CH3_GPIO_PORT            GPIOA
#define ATIM_TIMX_PWM_CH3_GPIO_PIN             GPIO_PIN_10
#define ATIM_TIMX_PWM_CH3_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)    /* PA��ʱ��ʹ�� */

/* ����ͨ��IO */
#define M1_LOW_SIDE_U_PORT                      GPIOB
#define M1_LOW_SIDE_U_PIN                       GPIO_PIN_13
#define M1_LOW_SIDE_U_GPIO_CLK_ENABLE()         do{  __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)    /* PB��ʱ��ʹ�� */

#define M1_LOW_SIDE_V_PORT                      GPIOB
#define M1_LOW_SIDE_V_PIN                       GPIO_PIN_14
#define M1_LOW_SIDE_V_GPIO_CLK_ENABLE()         do{  __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)    /* PB��ʱ��ʹ�� */

#define M1_LOW_SIDE_W_PORT                      GPIOB
#define M1_LOW_SIDE_W_PIN                       GPIO_PIN_15
#define M1_LOW_SIDE_W_GPIO_CLK_ENABLE()         do{  __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)    /* PB��ʱ��ʹ�� */

#define ATIM_TIMX_PWM_CHY_GPIO_AF               GPIO_AF1_TIM1

#define ATIM_TIMX_PWM                           TIM1
#define ATIM_TIMX_PWM_IRQn                      TIM1_UP_TIM10_IRQn
#define ATIM_TIMX_PWM_IRQHandler                TIM1_UP_TIM10_IRQHandler
#define ATIM_TIMX_PWM_CH1                       TIM_CHANNEL_1                               /* ͨ��1 */
#define ATIM_TIMX_PWM_CH2                       TIM_CHANNEL_2                               /* ͨ��2 */
#define ATIM_TIMX_PWM_CH3                       TIM_CHANNEL_3                               /* ͨ��3 */

#define ATIM_TIMX_PWM_CHY_CLK_ENABLE()          do{ __HAL_RCC_TIM1_CLK_ENABLE(); }while(0)  /* TIM1 ʱ��ʹ�� */

/******************************* ������ʱ�� ���� *************************************/

#define BTIM_TIMX_INT                           TIM6
#define BTIM_TIMX_INT_IRQn                      TIM6_DAC_IRQn
#define BTIM_TIMX_INT_IRQHandler                TIM6_DAC_IRQHandler
#define BTIM_TIMX_INT_CLK_ENABLE()              do{ __HAL_RCC_TIM6_CLK_ENABLE(); }while(0)  /* TIM6 ʱ��ʹ�� */

extern TIM_HandleTypeDef g_atimx_handle;                    /* ��ʱ��x��� */
/******************************************************************************************/

void atim_timx_oc_chy_init(uint16_t arr, uint16_t psc);     /* �߼���ʱ�� PWM��ʼ������ */
void btim_timx_int_init(uint16_t arr, uint16_t psc);        /* ������ʱ���жϳ�ʼ�� */

#endif


















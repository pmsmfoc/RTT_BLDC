/**
 ****************************************************************************************************
 * @file        bldc.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       BLDC ��������
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
#ifndef __BLDC_H
#define __BLDC_H

#include "stm32f4xx_hal.h"

/***************************************** ���״̬�ṹ�� **********************************************/
typedef struct 
{
    __IO uint8_t    run_flag;       /* ���б�־ */
    __IO uint8_t    locked_rotor;   /* ��ת��� */
    __IO uint8_t    step_sta;       /* ���λ���״̬ */
    __IO uint8_t    hall_single_sta;/* ��������״̬ */
    __IO uint8_t    hall_sta_edge;  /* ��������״̬���� */
    __IO uint8_t    step_last;      /* �ϴλ���״̬ */
    __IO uint8_t    dir;            /* �����ת���� */
    __IO int32_t    pos;            /* ���λ�� */
    __IO int32_t    speed;          /* ����ٶ� */
    __IO int16_t    current;        /* ����ٶ� */
    __IO uint16_t   pwm_duty;       /* ���ռ�ձ� */
    __IO uint32_t   hall_keep_t;    /* ��������ʱ�� */
    __IO uint32_t   hall_pul_num;   /* ���������������� */
    __IO uint32_t   lock_time;      /* �����תʱ�� */
    __IO uint32_t   no_single;
    __IO uint32_t   count_j;
    __IO uint64_t   sum_pos;
} _bldc_obj;

/******************************************************************************************/
#define MOTOR_1                     1
#define MOTOR_2                     2
extern _bldc_obj g_bldc_motor1;
/***************************************** ɲ������ *********************************************/

#define SHUTDOWN_PIN                      GPIO_PIN_10     
#define SHUTDOWN_PIN_GPIO                 GPIOF
#define SHUTDOWN_PIN_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)    /* PF��ʱ��ʹ�� */

#define SHUTDOWN2_PIN                     GPIO_PIN_2     
#define SHUTDOWN2_PIN_GPIO                GPIOF
#define SHUTDOWN2_PIN_GPIO_CLK_ENABLE()   do{  __HAL_RCC_GPIOF_CLK_ENABLE(); }while(0)    /* PF��ʱ��ʹ�� */

#define SHUTDOWN_EN                       HAL_GPIO_WritePin(SHUTDOWN_PIN_GPIO,SHUTDOWN_PIN,GPIO_PIN_SET);
#define SHUTDOWN_OFF                      HAL_GPIO_WritePin(SHUTDOWN_PIN_GPIO,SHUTDOWN_PIN,GPIO_PIN_RESET);


#define SHUTDOWN2_EN                       HAL_GPIO_WritePin(SHUTDOWN2_PIN_GPIO,SHUTDOWN2_PIN,GPIO_PIN_SET);
#define SHUTDOWN2_OFF                      HAL_GPIO_WritePin(SHUTDOWN2_PIN_GPIO,SHUTDOWN2_PIN,GPIO_PIN_RESET);

/****************************************** �����������ӿ�һ ************************************************/

#define HALL1_TIM_CH1_PIN           GPIO_PIN_10     /* U */
#define HALL1_TIM_CH1_GPIO          GPIOH
#define HALL1_U_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)    /* PH��ʱ��ʹ�� */

#define HALL1_TIM_CH2_PIN           GPIO_PIN_11     /* V */
#define HALL1_TIM_CH2_GPIO          GPIOH
#define HALL1_V_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)    /* PH��ʱ��ʹ�� */

#define HALL1_TIM_CH3_PIN           GPIO_PIN_12     /* W */
#define HALL1_TIM_CH3_GPIO          GPIOH
#define HALL1_W_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOH_CLK_ENABLE(); }while(0)    /* PH��ʱ��ʹ�� */

/****************************************** �����������ӿڶ� ************************************************/

#define HALL2_TIM_CH1_PIN           GPIO_PIN_12     /* U */
#define HALL2_TIM_CH1_GPIO          GPIOD
#define HALL2_U_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)    /* PD��ʱ��ʹ�� */

#define HALL2_TIM_CH2_PIN           GPIO_PIN_13     /* V */
#define HALL2_TIM_CH2_GPIO          GPIOD
#define HALL2_V_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOD_CLK_ENABLE(); }while(0)    /* PD��ʱ��ʹ�� */

#define HALL2_TIM_CH3_PIN           GPIO_PIN_8      /* W */
#define HALL2_TIM_CH3_GPIO          GPIOB
#define HALL2_W_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)    /* PB��ʱ��ʹ�� */

/*************************************** ������ϵ�� *****************************************************/

#define MAX_PWM_DUTY    (((168000/18) - 1)*0.96)        /* ���ռ�ձ� */

#define H_PWM_L_ON
#ifndef H_PWM_L_ON
#define H_PWM_L_PWM
#endif

#define CCW                         (1)                 /* 逆时针 */
#define CW                          (2)                 /* 顺时针 */
#define HALL_ERROR                  (0xF0)              /* ���������־ */
#define RUN                         (1)                 /* ����˶���־ */
#define STOP                        (0)                 /* ���ͣ����־ */

#define SPEED_COEFF      (uint32_t)((18000/4)*60)       /* ��תһȦ�仯4���źţ�2�Լ����������ԣ�NSNS��4����*/

#define ADC2CURT    (float)(3.3f/4.096f/0.12f)
#define ADC2VBUS    (float)(3.3f*25/4096)

#define NUM_CLEAR(para,val)     {if(para >= val){para=0;}}
#define NUM_MAX_LIMIT(para,val) {if(para > val){para=val;}}
#define NUM_MIN_LIMIT(para,val) {if(para < val){para=val;}}

typedef void(*pctr) (void);
void stop_motor1(void);
void start_motor1(void);

#define FirstOrderRC_LPF(Yn_1,Xn,a) Yn_1 = (1-a)*Yn_1 + a*Xn;   /* Yn:out;Xn:in;a:ϵ�� */
/***************************************** �������� *************************************************/

void bldc_init(uint16_t arr, uint16_t psc);                     /* BLDC��ʼ�� */
uint8_t check_hall_dir(_bldc_obj * obj);                        /* �������ת���� */
extern pctr pfunclist_m1[6];                                    /* �������ຯ��ָ������ */
void bldc_ctrl(uint8_t motor_id,int32_t dir,float duty);        /* bldc���ƺ��� */
uint8_t uemf_edge(uint8_t val);                                 /* ����״̬��� */
float get_temp(uint16_t para);                                  /* ��ȡ�¶�ֵ */
void calc_adc_val(uint16_t * p);                                /* adc��������˲����� */
void hall_gpio_init(void);                                      /* �����ӿڳ�ʼ�� */
uint32_t hallsensor_get_state(uint8_t motor_id);                /* ��ȡ����״̬ */
/*  �������� */
void m1_uhvl(void);
void m1_uhwl(void);
void m1_vhwl(void);
void m1_vhul(void);
void m1_whul(void);
void m1_whvl(void);

#endif

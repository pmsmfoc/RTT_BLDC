/**
 ****************************************************************************************************
 * @file        bldc_tim.c
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


#include "bldc_tim.h"
#include "bldc.h"
#include "pid.h"
#include "adc.h"

/******************************************************************************************/
/* ��ʱ�����þ�� ���� */

/* �߼���ʱ��PWM */
TIM_HandleTypeDef g_atimx_handle;           /* ��ʱ��x��� */
TIM_OC_InitTypeDef g_atimx_oc_chy_handle;   /* ��ʱ�������� */
extern _bldc_obj g_bldc_motor1;
extern PID_TypeDef  g_speed_pid;            /* �ٶ�PID�����ṹ��*/
/******************************************************************************************/

/**
 * @brief       �߼���ʱ��TIMX PWM ��ʼ������
 * @note
 *              �߼���ʱ����ʱ������APB2, ��PCLK2 = 168Mhz, ��������PPRE2����Ƶ, ���
 *              �߼���ʱ��ʱ�� = 168Mhz
 *              ��ʱ�����ʱ����㷽��: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=��ʱ������Ƶ��,��λ:Mhz
 *
 * @param       arr: �Զ���װֵ
 * @param       psc: ʱ��Ԥ��Ƶ��
 * @retval      ��
 */
void atim_timx_oc_chy_init(uint16_t arr, uint16_t psc)
{
    ATIM_TIMX_PWM_CHY_CLK_ENABLE();                             /* TIMX ʱ��ʹ�� */

    
    g_atimx_handle.Instance = ATIM_TIMX_PWM;                    /* ��ʱ��x */
    g_atimx_handle.Init.Prescaler = psc;                        /* ��ʱ����Ƶ */
    g_atimx_handle.Init.CounterMode = TIM_COUNTERMODE_UP;       /* ���ϼ���ģʽ */
    g_atimx_handle.Init.Period = arr;                           /* �Զ���װ��ֵ */
    g_atimx_handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;   /* ��Ƶ���� */
    g_atimx_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; /*ʹ��TIMx_ARR���л���*/
    g_atimx_handle.Init.RepetitionCounter = 0;                  /* ��ʼʱ������*/
    HAL_TIM_PWM_Init(&g_atimx_handle);                          /* ��ʼ��PWM */
    
    g_atimx_oc_chy_handle.OCMode = TIM_OCMODE_PWM1;             /* ģʽѡ��PWM1 */
    g_atimx_oc_chy_handle.Pulse = 0;
    g_atimx_oc_chy_handle.OCPolarity = TIM_OCPOLARITY_HIGH;     /* ����Ƚϼ���Ϊ�� */
    g_atimx_oc_chy_handle.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    g_atimx_oc_chy_handle.OCFastMode = TIM_OCFAST_DISABLE;
    g_atimx_oc_chy_handle.OCIdleState = TIM_OCIDLESTATE_RESET;
    g_atimx_oc_chy_handle.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&g_atimx_handle, &g_atimx_oc_chy_handle, ATIM_TIMX_PWM_CH1); /* ����TIMxͨ��y */
    HAL_TIM_PWM_ConfigChannel(&g_atimx_handle, &g_atimx_oc_chy_handle, ATIM_TIMX_PWM_CH2); /* ����TIMxͨ��y */
    HAL_TIM_PWM_ConfigChannel(&g_atimx_handle, &g_atimx_oc_chy_handle, ATIM_TIMX_PWM_CH3); /* ����TIMxͨ��y */
   
    /* ������ʱ��ͨ��1���PWM */
    HAL_TIM_PWM_Start(&g_atimx_handle,TIM_CHANNEL_1);

    /* ������ʱ��ͨ��2���PWM */
    HAL_TIM_PWM_Start(&g_atimx_handle,TIM_CHANNEL_2);

    /* ������ʱ��ͨ��3���PWM */
    HAL_TIM_PWM_Start(&g_atimx_handle,TIM_CHANNEL_3);
    
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 2, 2);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
               
    HAL_TIM_Base_Start_IT(&g_atimx_handle);                     /*�����߼���ʱ��1*/
}


/**
 * @brief       ��ʱ���ײ�������ʱ��ʹ�ܣ���������
                �˺����ᱻHAL_TIM_PWM_Init()����
 * @param       htim:��ʱ�����
 * @retval      ��
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == ATIM_TIMX_PWM)
    {
        GPIO_InitTypeDef gpio_init_struct;
        ATIM_TIMX_PWM_CHY_CLK_ENABLE();                             /* ʹ�ܶ�ʱ��ʱ�� */
        /* �������ű۶�ӦIOʱ��ʹ�� */
        ATIM_TIMX_PWM_CH1_GPIO_CLK_ENABLE();                        /* IOʱ��ʹ�� */
        ATIM_TIMX_PWM_CH2_GPIO_CLK_ENABLE();                        /* IOʱ��ʹ�� */
        ATIM_TIMX_PWM_CH3_GPIO_CLK_ENABLE();                        /* IOʱ��ʹ�� */
        /* �������ű۶�ӦIOʱ��ʹ�� */
        M1_LOW_SIDE_U_GPIO_CLK_ENABLE();                            /* IOʱ��ʹ�� */
        M1_LOW_SIDE_V_GPIO_CLK_ENABLE();                            /* IOʱ��ʹ�� */
        M1_LOW_SIDE_W_GPIO_CLK_ENABLE();                            /* IOʱ��ʹ�� */

        /* UVW_LOW��IO��ʼ�� */
        gpio_init_struct.Pin = M1_LOW_SIDE_U_PIN;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_HIGH;
        gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;                /* �������ģʽ */
        HAL_GPIO_Init(M1_LOW_SIDE_U_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = M1_LOW_SIDE_V_PIN;
        HAL_GPIO_Init(M1_LOW_SIDE_V_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = M1_LOW_SIDE_W_PIN;
        HAL_GPIO_Init(M1_LOW_SIDE_W_PORT, &gpio_init_struct);
        
        
        /*��ʱ��IO��ʼ��*/
        gpio_init_struct.Pin = ATIM_TIMX_PWM_CH1_GPIO_PIN;          /* ͨ��y��IO�� */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                    /* ����������� */
        gpio_init_struct.Pull = GPIO_NOPULL;                        /* ���� */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* ���� */
        gpio_init_struct.Alternate = ATIM_TIMX_PWM_CHY_GPIO_AF;     /* �˿ڸ��� */
        HAL_GPIO_Init(ATIM_TIMX_PWM_CH1_GPIO_PORT, &gpio_init_struct);
        
        gpio_init_struct.Pin = ATIM_TIMX_PWM_CH2_GPIO_PIN;          
        HAL_GPIO_Init(ATIM_TIMX_PWM_CH2_GPIO_PORT, &gpio_init_struct);
       
        gpio_init_struct.Pin = ATIM_TIMX_PWM_CH3_GPIO_PIN;         
        HAL_GPIO_Init(ATIM_TIMX_PWM_CH3_GPIO_PORT, &gpio_init_struct);
    }
}

void ATIM_TIMX_PWM_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_atimx_handle);
}

/******************************************������ʱ����ʼ��**********************************************************/
TIM_HandleTypeDef timx_handler;         /* ��ʱ��������� */


/**
 * @brief       ������ʱ��TIMX��ʱ�жϳ�ʼ������
 * @note
 *              ������ʱ����ʱ������APB1,��PPRE1 �� 2��Ƶ��ʱ��
 *              ������ʱ����ʱ��ΪAPB1ʱ�ӵ�2��, ��APB1Ϊ42M, ���Զ�ʱ��ʱ�� = 842Mhz
 *              ��ʱ�����ʱ����㷽��: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=��ʱ������Ƶ��,��λ:Mhz
 *
 * @param       arr: �Զ���װֵ��
 * @param       psc: ʱ��Ԥ��Ƶ��
 * @retval      ��
 */
void btim_timx_int_init(uint16_t arr, uint16_t psc)
{
    timx_handler.Instance = BTIM_TIMX_INT;                      /* ͨ�ö�ʱ��X */
    timx_handler.Init.Prescaler = psc;                          /* ����Ԥ��Ƶ��  */
    timx_handler.Init.CounterMode = TIM_COUNTERMODE_UP;         /* ���ϼ����� */
    timx_handler.Init.Period = arr;                             /* �Զ�װ��ֵ */
    timx_handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;   /* ʱ�ӷ�Ƶ���� */
    HAL_TIM_Base_Init(&timx_handler);
    
    HAL_TIM_Base_Start_IT(&timx_handler);                       /* ʹ��ͨ�ö�ʱ��x�ͼ�������жϣ�TIM_IT_UPDATE */
    __HAL_TIM_CLEAR_IT(&timx_handler,TIM_IT_UPDATE);            /* ��������жϱ�־λ */
}

/**
 * @brief       ��ʱ���ײ�����������ʱ�ӣ������ж����ȼ�
                �˺����ᱻHAL_TIM_Base_Init()��������
 * @param       ��
 * @retval      ��
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == BTIM_TIMX_INT)
    {
        BTIM_TIMX_INT_CLK_ENABLE();                     /* ʹ��TIMʱ�� */
        HAL_NVIC_SetPriority(BTIM_TIMX_INT_IRQn, 1, 3); /* ��ռ1�������ȼ�3����2 */
        HAL_NVIC_EnableIRQ(BTIM_TIMX_INT_IRQn);         /* ����ITM3�ж� */
    }
}

/**
 * @brief       ������ʱ��TIMX�жϷ�����
 * @param       ��
 * @retval      ��
 */
void BTIM_TIMX_INT_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&timx_handler);                  /* ��ʱ���ص����� */
}


/***********************************************��ʱ���жϻص�����***********************************************/
int32_t temp_pwm1 = 0.0;                    /* ���PID����������ֵ */
int32_t motor_pwm_s = 0;                    /* ���һ���˲�������� */

#define ADC_AMP_OFFSET_TIMES 50             /* ͣ��״̬���������ADC�ɼ����� */
uint16_t adc_amp_offset[3][ADC_AMP_OFFSET_TIMES+1];
uint8_t adc_amp_offset_p = 0;
int16_t adc_amp[3];

volatile uint16_t adc_val_m1[ADC_CH_NUM];   /* ADC���ݻ����� */
int16_t adc_amp_un[3];
float  adc_amp_bus = 0.0f;
/**
 * @brief       ��ʱ���жϻص�
 * @param       ��
 * @retval      ��
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    uint8_t i;
    uint8_t bldc_dir=0;
    static uint8_t times_count=0;           /* ��ʱ��ʱ���¼ */
    int16_t temp_speed=0;                   /* ��ʱ�ٶȴ洢 */
    if(htim->Instance == ATIM_TIMX_PWM)     /* 55us */
    {
        /******************************* �������� *******************************/
        if(g_bldc_motor1.run_flag == RUN)
        {
            if(g_bldc_motor1.dir == CW)
            {
                g_bldc_motor1.step_sta = hallsensor_get_state(MOTOR_1);
            }
            else
            {
                g_bldc_motor1.step_sta = 7 - hallsensor_get_state(MOTOR_1);
            }
            if((g_bldc_motor1.step_sta <= 6)&&(g_bldc_motor1.step_sta >= 1))
            {
                pfunclist_m1[g_bldc_motor1.step_sta-1]();
            }
            else    /* ���������󡢽Ӵ��������Ͽ������ */
            {
                stop_motor1();
                g_bldc_motor1.run_flag = STOP;
            }
            
            /******************************* �ٶȼ��� *******************************/
            g_bldc_motor1.count_j++;                /* �����ٶ�ר�ü���ֵ */
            g_bldc_motor1.hall_sta_edge = uemf_edge(g_bldc_motor1.hall_single_sta);/* ��ⵥ�������źŵı仯 */
            if(g_bldc_motor1.hall_sta_edge == 0)    /* ͳ�Ƶ��������źŵĸߵ�ƽʱ�䣬��ֻ��һ�Լ���ʱ����תһȦΪһ���������塣һ��һ����Ӽ���תһȦ������ʱ��*/
            {
                /*�����ٶ�*/
                if(g_bldc_motor1.dir == CW)
                    temp_speed = (SPEED_COEFF/g_bldc_motor1.count_j);
                else
                    temp_speed = -(SPEED_COEFF/g_bldc_motor1.count_j);
                FirstOrderRC_LPF(g_bldc_motor1.speed,temp_speed,0.2379f);   /* һ���˲� */
                g_bldc_motor1.no_single = 0;
                g_bldc_motor1.count_j = 0;
            }
            if(g_bldc_motor1.hall_sta_edge == 1)    /* ���ɼ����½���ʱ������0 */
            {
                g_bldc_motor1.no_single = 0;
                g_bldc_motor1.count_j = 0;
            }
            if(g_bldc_motor1.hall_sta_edge == 2)    /* ����ֵһֱ�������δ���� */
            {
                g_bldc_motor1.no_single++;          /* ������ʱ���ۼ� ��ʱ���ж��ٶ�Ϊ0 */
                
                if(g_bldc_motor1.no_single > 15000)
                {
                    
                    g_bldc_motor1.no_single = 0;
                    g_bldc_motor1.speed = 0;        /* ��ʱ���� �ж�Ϊֹͣ �ٶ�Ϊ0 */
                }
            }
            /******************************* λ�ü�¼ *******************************/
            if(g_bldc_motor1.step_last != g_bldc_motor1.step_sta)
            {
                bldc_dir = check_hall_dir(&g_bldc_motor1);
                if(bldc_dir == CCW)
                {
                    g_bldc_motor1.pos -= 1;
                }
                else if(bldc_dir == CW)
                {
                    g_bldc_motor1.pos += 1;
                }
                g_bldc_motor1.step_last = g_bldc_motor1.step_sta;
            }
            /******************************* PID���� *******************************/
                temp_pwm1 = increment_pid_ctrl(&g_speed_pid,g_bldc_motor1.speed);   /* PID�����㷨���������ֵ */
                FirstOrderRC_LPF(motor_pwm_s,temp_pwm1,0.085);                      /* һ���˲� */
                if(motor_pwm_s < 0)                                                 /* �ж�����ֵ */
                {
                    g_bldc_motor1.pwm_duty = -motor_pwm_s;
                }
                else
                {
                   g_bldc_motor1.pwm_duty = motor_pwm_s;
                }
            /******************************* ����������� *******************************/
            for(i=0; i<3; i++)
            {
                adc_val_m1[i] = g_adc_val[i+2];
                adc_amp[i] = adc_val_m1[i] - adc_amp_offset[i][ADC_AMP_OFFSET_TIMES];  /* �˶�״̬ADCֵ - ͣ��״̬ADCֵ = ʵ������ADCֵ */
                
                if(adc_amp[i]>=0)                                                      /* ȥ�����綯������ĸ��������� */
                 adc_amp_un[i] = adc_amp[i];
            }
            /* ����ĸ�ߵ�����ĸ�ߵ���Ϊ���������п��ض����������֮�ͣ�*/
            if(g_bldc_motor1.step_sta == 0x05)
            {
                adc_amp_bus= (adc_amp_un[0]+ adc_amp_un[1])*ADC2CURT;/* UV */
            }
            else if(g_bldc_motor1.step_sta== 0x01)
            {
                adc_amp_bus= (adc_amp_un[0]+ adc_amp_un[2])*ADC2CURT;/* UW */
            }
            else if(g_bldc_motor1.step_sta== 0x03)
            {
                adc_amp_bus= (adc_amp_un[1]+ adc_amp_un[2])*ADC2CURT;/* VW */
            }
            else if(g_bldc_motor1.step_sta== 0x02)
            {
                adc_amp_bus= (adc_amp_un[0]+ adc_amp_un[1])*ADC2CURT;/* UV */
            }
            else if(g_bldc_motor1.step_sta == 0x06)
            {
                adc_amp_bus= (adc_amp_un[0]+ adc_amp_un[2])*ADC2CURT;/* WU */
            }
            else if(g_bldc_motor1.step_sta == 0x04)
            {
                adc_amp_bus= (adc_amp_un[2]+ adc_amp_un[1])*ADC2CURT;/* WV */
            }
        }
    }
    else if(htim->Instance == TIM6)
    {
        /******************************* �ɼ����ͣ��״̬�µ�ƫ�õ�ѹ *******************************/
        times_count++;
        if(g_bldc_motor1.run_flag == STOP)
        {
            uint8_t i;
            uint32_t avg[3] = {0,0,0};
            adc_amp_offset[0][adc_amp_offset_p] = g_adc_val[2];     /* ��ȡ���ͣ��״̬�µ�������� */
            adc_amp_offset[1][adc_amp_offset_p] = g_adc_val[3];
            adc_amp_offset[2][adc_amp_offset_p] = g_adc_val[4];

            adc_amp_offset_p ++;
            NUM_CLEAR(adc_amp_offset_p,ADC_AMP_OFFSET_TIMES);
            for(i=0; i<ADC_AMP_OFFSET_TIMES; i++)
            {
                avg[0] += adc_amp_offset[0][i];                     /* ������ֵ�ۼ� */
                avg[1] += adc_amp_offset[1][i];
                avg[2] += adc_amp_offset[2][i];
            }
            for(i=0; i<3; i++)
            {
                avg[i] /= ADC_AMP_OFFSET_TIMES;                     /* ȡƽ�� */
                adc_amp_offset[i][ADC_AMP_OFFSET_TIMES] = avg[i];   /* ��ֵ */
            }
        }
    }
}



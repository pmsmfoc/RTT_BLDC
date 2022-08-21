/**
 ****************************************************************************************************
 * @file        adc.h
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-18
 * @brief       ADC ��������
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
 * V1.0 20211018
 * ��һ�η���
 *
 ****************************************************************************************************
 */

#ifndef __ADC_H
#define __ADC_H

#include "stm32f4xx_hal.h"


/******************************************************************************************/
/* ADC������ ���� */

#define ADC_ADCX_CH0_GPIO_PORT              GPIOB                                               /* ��Դ��ѹ�ɼ����� */
#define ADC_ADCX_CH0_GPIO_PIN               GPIO_PIN_1
#define ADC_ADCX_CH0_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

#define ADC_ADCX_CH1_GPIO_PORT              GPIOA                                               /* �¶Ȳɼ����� */
#define ADC_ADCX_CH1_GPIO_PIN               GPIO_PIN_0
#define ADC_ADCX_CH1_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

#define ADC_ADCX_CH2_GPIO_PORT              GPIOB                                               /* U��ɼ����� */
#define ADC_ADCX_CH2_GPIO_PIN               GPIO_PIN_0
#define ADC_ADCX_CH2_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

#define ADC_ADCX_CH3_GPIO_PORT              GPIOA                                               /* V��ɼ����� */
#define ADC_ADCX_CH3_GPIO_PIN               GPIO_PIN_6
#define ADC_ADCX_CH3_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

#define ADC_ADCX_CH4_GPIO_PORT              GPIOA                                               /* W��ɼ����� */
#define ADC_ADCX_CH4_GPIO_PIN               GPIO_PIN_3
#define ADC_ADCX_CH4_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

#define ADC_ADCX                            ADC1 
#define ADC_ADCX_CH0                        ADC_CHANNEL_9                                       /* ͨ��Y,  0 <= Y <= 17 */
#define ADC_ADCX_CH1                        ADC_CHANNEL_0
#define ADC_ADCX_CH2                        ADC_CHANNEL_8
#define ADC_ADCX_CH3                        ADC_CHANNEL_6
#define ADC_ADCX_CH4                        ADC_CHANNEL_3

#define ADC_ADCX_CHY_CLK_ENABLE()           do{ __HAL_RCC_ADC1_CLK_ENABLE(); }while(0)          /* ADC1 ʱ��ʹ�� */

#define ADC_CH_NUM                          5                                                   /* ��Ҫת����ͨ����Ŀ */
#define ADC_COLL                            50                                                  /* ���ɼ����� */
#define ADC_SUM                             ADC_CH_NUM * ADC_COLL                               /* �ܲɼ����� */

/* ADC��ͨ��/��ͨ�� DMA�ɼ� DMA��������� ����
 * ע��: �������ǵ�ͨ������ʹ������Ķ���.
 */
#define ADC_ADCX_DMASx                      DMA2_Stream4
#define ADC_ADCX_DMASx_Chanel               DMA_CHANNEL_0                                       /* ADC1_DMA����Դ */
#define ADC_ADCX_DMASx_IRQn                 DMA2_Stream4_IRQn
#define ADC_ADCX_DMASx_IRQHandler           DMA2_Stream4_IRQHandler

extern uint16_t g_adc_val[ADC_CH_NUM];


/******************************************************************************************/

void adc_init(void);                                                                            /* ADC��ʼ�� */
uint32_t adc_get_result_average(uint8_t ch);                                                    /* ���ĳ��ͨ��ֵ  */
void adc_nch_dma_init(void);
void adc_dma_init(uint32_t mar);                                                                /* ADC DMA�ɼ���ʼ�� */

#endif 
















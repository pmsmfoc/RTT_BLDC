/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-08-20     RT-Thread    first version
 */

#include <rtthread.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include "bldc_tim.h"
#include "bldc.h"
#include "pid.h"
#include "uart.h"


int main(void)
{

    bldc_init(168000/18-1,0);
    bldc_ctrl(MOTOR_1,CCW,0);                /* 初始无刷电机接口1速度 */

    pid_init();
    uart4_init();


//    g_bldc_motor1.dir = CCW;
//    g_bldc_motor1.run_flag = RUN;
//    start_motor1();//开启pwm输出
//    g_speed_pid.SetPoint = -1000;



    uart4_rx_td = rt_thread_create("uart4",
                            uart4_td_entry,
                            RT_NULL,
                            8192,
                            3,
                            20);
    if(uart4_rx_td != RT_NULL)
        rt_thread_startup(uart4_rx_td);



    return RT_EOK;
}
void motor_info(void)
{
    rt_kprintf("motor speed is %d\r\n",g_bldc_motor1.speed);
    rt_kprintf("motor position is %d\r\n",g_bldc_motor1.pos);
    rt_kprintf("\r\n");
}
MSH_CMD_EXPORT(motor_info,show motor info);


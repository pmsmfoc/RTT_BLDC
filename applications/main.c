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
float*user_setpoint = (float*)(&g_speed_pid.SetPoint);    /* 设置目标值指针 指向存放目标值地址 */
int main(void)
{

    bldc_init(168000/18-1,0);
    bldc_ctrl(MOTOR_1,CCW,0);                /* 初始无刷电机接口1速度 */

    pid_init();
    g_bldc_motor1.dir = CW;
    g_bldc_motor1.run_flag = RUN;
    start_motor1();
    *user_setpoint = 1200;
    while (1)
    {
        rt_kprintf("motor speed is %d\r\n",g_bldc_motor1.speed);
        rt_kprintf("motor position is %d\r\n",g_bldc_motor1.pos%6);
        rt_kprintf("\r\n");
        rt_thread_mdelay(1000);
    }

    return RT_EOK;
}

/*
 * Copyright (c) 2020-2021, Bluetrum Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes
 * 2020-11-18     greedyhao         first version
 */

#ifndef DRV_COMMON_H__
#define DRV_COMMON_H__

#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>

#define GET_PIN(PORTx,PIN) (rt_uint8_t)__AB32_GET_PIN_##PORTx(PIN)

void uart0_irq_post(void);
void uart1_irq_post(void);
void uart2_irq_post(void);

#endif // DRV_COMMON_H__

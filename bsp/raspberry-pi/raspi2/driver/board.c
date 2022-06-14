/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */

#include <rthw.h>
#include <rtthread.h>

#include "board.h"
#include "drv_uart.h"
#include "cp15.h"

void rt_hw_timer_isr(int vector, void *parameter)
{
    ARM_TIMER_IRQCLR = 0;

    rt_tick_increase();
}

int rt_hw_timer_init(void)
{
    /* timer_clock = apb_clock/(pre_divider + 1) */
    ARM_TIMER_PREDIV = (250 - 1);

    ARM_TIMER_RELOAD = 0;
    ARM_TIMER_LOAD   = 0;
    ARM_TIMER_IRQCLR = 0;
    ARM_TIMER_CTRL   = 0;

    ARM_TIMER_RELOAD = 10000;
    ARM_TIMER_LOAD   = 10000;

    /* 23-bit counter, enable interrupt, enable timer */
    ARM_TIMER_CTRL   = (1 << 1) | (1 << 5) | (1 << 7);

    rt_hw_interrupt_install(IRQ_ARM_TIMER, rt_hw_timer_isr, RT_NULL, "tick");
    rt_hw_interrupt_umask(IRQ_ARM_TIMER);

    return 0;
}

void vector_copy(void)
{
    rt_memcpy((void*)0x0, (void*)0x8000, 64);
}

void rt_hw_board_init(void)
{
    /* initialize hardware interrupt */
    rt_hw_interrupt_init();
    rt_hw_vector_init();

    /* initialize uart */
    rt_hw_uart_init();

#if defined(RT_USING_CONSOLE) && defined(RT_USING_DEVICE)
    /* set console device */
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif

#ifdef RT_USING_HEAP
    /* initialize memory system */
    rt_kprintf("heap: 0x%08x - 0x%08x\n", RT_HW_HEAP_BEGIN, RT_HW_HEAP_END);
    rt_system_heap_init(RT_HW_HEAP_BEGIN, RT_HW_HEAP_END);
#endif

    /* initialize timer for os tick */
    rt_hw_timer_init();

#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif
}

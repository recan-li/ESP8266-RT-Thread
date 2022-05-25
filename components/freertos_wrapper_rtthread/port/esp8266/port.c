/*
 * FreeRTOS Kernel V10.0.1
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 * Additions Copyright 2018 Espressif Systems (Shanghai) PTE LTD
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* Scheduler includes. */

#include <stdint.h>
#include <string.h>

#include <xtensa/config/core.h>
#include <xtensa/tie/xt_interrupt.h>
#include <xtensa/tie/xt_timer.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/xtensa_rtos.h"

#include "esp_attr.h"
#include "esp_libc.h"
#include "esp_task_wdt.h"
#include "esp_sleep.h"
#include "esp_log.h"

#include "esp8266/eagle_soc.h"
#include "rom/ets_sys.h"
#include "esp8266/rom_functions.h"
#include "driver/soc.h"

#ifdef CONFIG_ENABLE_ESP_OSAL_RTTHREAD
#include "rtthread.h"

extern volatile rt_uint8_t rt_interrupt_nest;
extern rt_thread_t rt_current_thread;

extern void rt_schedule(void);

#endif

#define SET_STKREG(r,v)     sp[(r) >> 2] = (uint32_t)(v)
#define PORT_ASSERT(x)      do { if (!(x)) {ets_printf("%s %u\n", "rtos_port", __LINE__); while(1){}; }} while (0)

extern uint8_t NMIIrqIsOn;

uint32_t cpu_sr;

uint32_t _xt_tick_divisor;

/* Each task maintains its own interrupt status in the critical nesting
variable. */
static uint32_t uxCriticalNesting = 0;

uint32_t g_esp_boot_ccount;
uint64_t g_esp_os_ticks;
uint64_t g_esp_os_us;
uint64_t g_esp_os_cpu_clk;

static uint32_t s_switch_ctx_flag;

void vPortEnterCritical(void);
void vPortExitCritical(void);

void IRAM_ATTR portYIELD_FROM_ISR(void)
{
    s_switch_ctx_flag = 1;
}

#ifndef CONFIG_ENABLE_ESP_OSAL_RTTHREAD

char *g_task_name = NULL;

uint8_t *__cpu_init_stk(uint8_t *stack_top, void (*_entry)(void *), void *param, void (*_exit)(void))
{

    uint32_t *sp, *tp, *stk = (uint32_t *)stack_top;

    /* Create interrupt stack frame aligned to 16 byte boundary */
    sp = (uint32_t *)(((uint32_t)(stk + 1) - XT_CP_SIZE - XT_STK_FRMSZ) & ~0xf);

    /* Clear the entire frame (do not use memset() because we don't depend on C library) */
    for (tp = sp; tp <= stk; ++tp) {
        *tp = 0;
    }

    /* Explicitly initialize certain saved registers */
    SET_STKREG(XT_STK_PC,   _entry);                        /* task entrypoint                  */
    SET_STKREG(XT_STK_A0,   _exit);                         /* to terminate GDB backtrace       */
    SET_STKREG(XT_STK_A1,   (uint32_t)sp + XT_STK_FRMSZ);   /* physical top of stack frame      */
    SET_STKREG(XT_STK_A2,   param);                         /* parameters      */
    SET_STKREG(XT_STK_EXIT, _xt_user_exit);                 /* user exception exit dispatcher   */

    /* Set initial PS to int level 0, EXCM disabled ('rfe' will enable), user mode. */
    SET_STKREG(XT_STK_PS,      PS_UM | PS_EXCM);

    ets_printf("stack_top: %p\r\n", stack_top);
    extern void dump_memory(char *name, uint8_t *data, uint32_t len);
    dump_memory(g_task_name, sp, 80);

    return (uint8_t *)sp;
}

/*
 * See header file for description.
 */
StackType_t *pxPortInitialiseStack(StackType_t *pxTopOfStack, pdTASK_CODE pxCode, void *pvParameters)
{
    return (StackType_t *)__cpu_init_stk((uint8_t *)pxTopOfStack, pxCode, pvParameters, NULL);
}
#endif

void IRAM_ATTR PendSV(int req)
{
    if (req == 1) {
        vPortEnterCritical();
        s_switch_ctx_flag = 1;
        xthal_set_intset(1 << ETS_SOFT_INUM);
        vPortExitCritical();
    } else if (req == 2) {
        xthal_set_intset(1 << ETS_SOFT_INUM);
    }
}

void IRAM_ATTR SoftIsrHdl(void* arg)
{
    extern int MacIsrSigPostDefHdl(void);

    if (MacIsrSigPostDefHdl()) {
        portYIELD_FROM_ISR();
    }
}

void IRAM_ATTR esp_increase_tick_cnt(const TickType_t ticks)
{
    g_esp_os_ticks += ticks;
}

void IRAM_ATTR xPortSysTickHandle(void *p)
{
    uint32_t us;
    uint32_t ticks;
    uint32_t ccount;

    extern int ets_puc(int c);
    ets_putc('+');
    RT_DEBUG_MORE("+\r\n");

    /**
     * System or application may close interrupt too long, such as the operation of read/write/erase flash.
     * And then the "ccount" value may be overflow.
     *
     * So add code here to calibrate system time.
     */
    ccount = soc_get_ccount();
    us = ccount / g_esp_ticks_per_us;
  
    g_esp_os_us += us;
    g_esp_os_cpu_clk += ccount;

    soc_set_ccount(0);
    soc_set_ccompare(_xt_tick_divisor);

    ticks = us / 1000 / portTICK_PERIOD_MS;

    if (ticks > 1) {
        vTaskStepTick(ticks - 1);
    }

    g_esp_os_ticks++;

    if (xTaskIncrementTick() != pdFALSE) {
        ets_printf("t\r\n");
        portYIELD_FROM_ISR();
    }
    ets_putc('+');
}

/**
 * @brief Return current CPU clock frequency
 */
int esp_clk_cpu_freq(void)
{
    return _xt_tick_divisor * XT_TICK_PER_SEC;
}

/*
 * See header file for description.
 */
portBASE_TYPE xPortStartScheduler(void)
{
    /*
     * TAG 1.2.3 FreeRTOS call "portDISABLE_INTERRUPTS" at file tasks.c line 1973, this is not at old one.
     * This makes it to be a wrong value.
     * 
     * So we should initialize global value "cpu_sr" with a right value.
     * 
     * Todo: Remove this one when refactor startup function.
     */
    cpu_sr = 0x20;

#if 0
    /*******software isr*********/
    _xt_isr_attach(ETS_SOFT_INUM, SoftIsrHdl, NULL);
    _xt_isr_unmask(1 << ETS_SOFT_INUM);

    _xt_isr_attach(ETS_MAX_INUM, xPortSysTickHandle, NULL);

    /* Initialize system tick timer interrupt and schedule the first tick. */
    _xt_tick_divisor = xtbsp_clock_freq_hz() / XT_TICK_PER_SEC;

    g_esp_boot_ccount = soc_get_ccount();
    soc_set_ccount(0);
    _xt_tick_timer_init();
#else
    extern void sys_tick_int_init(void);
    sys_tick_int_init();
#endif

    vTaskSwitchContext();

    while (0) {
        RT_DEBUG_MORE("6+1\r\n");
        for(int i = 0; i < 10000000; i++);
        //RT_DEBUG_MORE("6+1\r\n");
        //for(int i = 0; i < 10000000; i++);
        //RT_DEBUG_MORE("6+1\r\n");
        //for(int i = 0; i < 10000000; i++);
            break;
    }

    /* Restore the context of the first task that is going to run. */
    _xt_enter_first_task();

    /* Should not get here as the tasks are now running! */
    return pdTRUE;
}

void vPortInitContextFromOldStack(StackType_t *newStackTop, StackType_t *oldStackTop, UBaseType_t stackSize)
{
    uintptr_t *sp;

    memcpy(newStackTop, oldStackTop, stackSize);
    sp = (uintptr_t *)newStackTop;
    sp[XT_STK_A1 / sizeof(uintptr_t)] = (uintptr_t)sp + XT_STK_FRMSZ;
}

void vPortEndScheduler(void)
{
    /* It is unlikely that the CM3 port will require this function as there
    is nothing to return to.  */
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

static char ClosedLv1Isr = 0;

void IRAM_ATTR vPortEnterCritical(void)
{
    if (NMIIrqIsOn == 0) {
        if (ClosedLv1Isr != 1) {
            portDISABLE_INTERRUPTS();
            ClosedLv1Isr = 1;
        }
        uxCriticalNesting++;
    }
}
/*-----------------------------------------------------------*/

void IRAM_ATTR vPortExitCritical(void)
{
    if (NMIIrqIsOn == 0) {
        if (uxCriticalNesting > 0) {
            uxCriticalNesting--;

            if (uxCriticalNesting == 0) {
                if (ClosedLv1Isr == 1) {
                    ClosedLv1Isr = 0;
                    portENABLE_INTERRUPTS();
                }
            }
        } else {
            ets_printf(DRAM_STR("E:C:%u\n"), uxCriticalNesting);
            PORT_ASSERT((uxCriticalNesting > 0));
        }
    }
}

void show_critical_info(void)
{
    ets_printf("ShowCritical:%u\n", uxCriticalNesting);
    ets_printf("s_switch_ctx_flag:%u\n", s_switch_ctx_flag);
}

#ifdef ESP_DPORT_CLOSE_NMI
static int s_nmi_is_closed;

void esp_dport_close_nmi(void)
{
    vPortEnterCritical();
    REG_WRITE(PERIPHS_DPORT_BASEADDR, REG_READ(PERIPHS_DPORT_BASEADDR) & ~0x1);
    s_nmi_is_closed = 1;
    vPortExitCritical();
}

#define ESP_NMI_IS_CLOSED()     s_nmi_is_closed
#else
#define ESP_NMI_IS_CLOSED()     0
#endif

void IRAM_ATTR vPortETSIntrLock(void)
{
    if (NMIIrqIsOn == 0) {
        uint32_t regval = REG_READ(NMI_INT_ENABLE_REG);

        vPortEnterCritical();

        REG_WRITE(NMI_INT_ENABLE_REG, 0);

        if (!ESP_NMI_IS_CLOSED()) {
            do {
                REG_WRITE(INT_ENA_WDEV, WDEV_TSF0_REACH_INT);
            } while(REG_READ(INT_ENA_WDEV) != WDEV_TSF0_REACH_INT);
        }

        REG_WRITE(NMI_INT_ENABLE_REG, regval);
    }
}

void IRAM_ATTR vPortETSIntrUnlock(void)
{
    if (NMIIrqIsOn == 0) {
        uint32_t regval = REG_READ(NMI_INT_ENABLE_REG);

        REG_WRITE(NMI_INT_ENABLE_REG, 0);

        if (!ESP_NMI_IS_CLOSED()) {
            extern uint32_t WDEV_INTEREST_EVENT;

            REG_WRITE(INT_ENA_WDEV, WDEV_INTEREST_EVENT);
        }

        REG_WRITE(NMI_INT_ENABLE_REG, regval);

        vPortExitCritical();
    }
}

/*
 * @brief check if CPU core interrupt is disable
 */
bool interrupt_is_disable(void)
{
    uint32_t tmp;

    __asm__ __volatile__ (
        "rsr %0, PS\n"
        : "=a"(tmp) : : "memory");

    return tmp & 0xFUL ? true : false;
}

static _xt_isr_entry s_isr[16];
static uint8_t s_xt_isr_status = 0;

void _xt_isr_attach(uint8_t i, _xt_isr func, void* arg)
{
    s_isr[i].handler = func;
    s_isr[i].arg = arg;
}

void IRAM_ATTR _xt_isr_handler(void)
{
    do {
        uint32_t mask = soc_get_int_mask();

        //ets_printf("isr mask: %x %p\r\n", mask, rt_current_thread);
        //ets_printf("%s\n", rt_current_thread->name);
        for (int i = 0; i < ETS_INT_MAX && mask; i++) {
            int bit = 1 << i;

            if (!(bit & mask) || !s_isr[i].handler)
                continue;

            soc_clear_int_mask(bit);

            s_xt_isr_status = 1;
            s_isr[i].handler(s_isr[i].arg);
            s_xt_isr_status = 0;

            mask &= ~bit;
        }
    } while (soc_get_int_mask());

    if (s_switch_ctx_flag) {
        ets_printf("s_switch_ctx_flag 11111000... switch begin\r\n");
#ifndef CONFIG_ENABLE_ESP_OSAL_RTTHREAD
        vTaskSwitchContext();
#else
        ets_printf("cur_task1: %s\n", rt_current_thread->name);        
        rt_schedule();
        ets_printf("cur_task2: %s\n", rt_current_thread->name);
#endif
		ets_printf("s_switch_ctx_flag 11111222333... switch end\r\n");
        s_switch_ctx_flag = 0;
    }
}

__attribute__((section("text"))) void debug_int_enter(void)
{
    ets_printf("1--->\n");
}

__attribute__((section("text"))) void debug_int_exit(void)
{
    ets_printf("1--->\n");
}

int xPortInIsrContext(void)
{
    return s_xt_isr_status != 0;
}

void __attribute__((weak, noreturn)) vApplicationStackOverflowHook(xTaskHandle xTask, const char *pcTaskName)
{
    ets_printf("***ERROR*** A stack overflow in task %s has been detected.\r\n", pcTaskName);
    abort();
}

signed portBASE_TYPE xTaskGenericCreate(TaskFunction_t pxTaskCode,
                                        const signed char * const pcName,
                                        unsigned short usStackDepth,
                                        void *pvParameters,
                                        unsigned portBASE_TYPE uxPriority,
                                        TaskHandle_t *pxCreatedTask,
                                        StackType_t *puxStackBuffer,
                                        const MemoryRegion_t * const xRegions)
{
    (void)puxStackBuffer;
    (void)xRegions;
    return xTaskCreate(pxTaskCode, (const char * const)pcName, usStackDepth,
                       pvParameters, uxPriority, pxCreatedTask);
}

#ifndef CONFIG_ENABLE_ESP_OSAL_RTTHREAD
BaseType_t xQueueGenericReceive(QueueHandle_t xQueue, void * const pvBuffer,
                                TickType_t xTicksToWait, const BaseType_t xJustPeeking)
{
    configASSERT(xJustPeeking == 0);
    return xQueueReceive(xQueue, pvBuffer, xTicksToWait);
}
#endif

void esp_internal_idle_hook(void)
{
    esp_task_wdt_reset();

    esp_sleep_start();
}

#ifndef CONFIG_ENABLE_ESP_OSAL_RTTHREAD
#if configUSE_IDLE_HOOK == 1
void __attribute__((weak)) vApplicationIdleHook(void)
{

}
#endif

#if configUSE_TICK_HOOK == 1
void __attribute__((weak)) vApplicationTickHook(void)
{

}
#endif
#endif

uint32_t xPortGetTickRateHz(void)
{
    return (uint32_t)configTICK_RATE_HZ;
}

/**************** wrapper with RT-Thead *********************/

#include "rtthread.h"
#include "driver/uart.h"

volatile rt_ubase_t  rt_interrupt_from_thread = 0;
volatile rt_ubase_t  rt_interrupt_to_thread   = 0;

#define rt_thread_switch_interrupt_flag s_switch_ctx_flag

void xt_ints_on(unsigned int mask)
{
    _xt_isr_mask(mask);
}

// rt-thread 中的控制台输出
void rt_hw_console_output(const char *str)
{
    /* empty console output */
    ets_printf(str);
}

// rt-thread 中的finsh组件的输入
char rt_hw_console_getchar(void)
{
    rt_uint16_t len = 0;
    char ch = 0;

    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_0, (size_t*)&len));
    if(len > 0)
    {
        uart_read_bytes(UART_NUM_0, (uint8_t *)&ch, 1, 100);
        return ch;
    }
    return -1;
}

//使能中断
void rt_hw_interrupt_enable(rt_base_t level)
{
    //rt_kprintf("int enable !1111\r\n");
    cpu_sr = level;
    portENABLE_INTERRUPTS();
}

//关闭中断
rt_base_t rt_hw_interrupt_disable(void)
{
    //rt_kprintf("int disable !1111\r\n");
    portDISABLE_INTERRUPTS();
    return cpu_sr;
}

/**
 * This function will initialize thread stack
 *
 * @param tentry the entry of thread
 * @param parameter the parameter of entry
 * @param stack_addr the beginning stack address
 * @param texit the function will be called when thread exit
 *
 * @return stack address
 */
#define SET_STKREG(r,v)     sp[(r) >> 2] = (uint32_t)(v)

char *g_task_name = NULL;

rt_uint8_t *rt_hw_stack_init(void       *tentry,
                             void       *parameter,
                             rt_uint8_t *stack_addr,
                             void       *texit) //线程退出地址，即rt_thread_exit，暂时未考虑
{

    uint32_t *sp, *tp, *stk = (uint32_t *)stack_addr;

    /* Create interrupt stack frame aligned to 16 byte boundary */
    sp = (uint32_t *)(((uint32_t)(stk + 1) - XT_CP_SIZE - XT_STK_FRMSZ) & ~0xf);

    /* Clear the entire frame (do not use memset() because we don't depend on C library) */
    for (tp = sp; tp <= stk; ++tp) {
        *tp = 0;
    }

    RT_DEBUG_MORE("XT_STK_FRMSZ=%d", XT_STK_FRMSZ);

    /* Explicitly initialize certain saved registers */
    SET_STKREG(XT_STK_PC,   tentry);                        /* task entrypoint                  */
    SET_STKREG(XT_STK_A0,   0);                         /* to terminate GDB backtrace       */
    SET_STKREG(XT_STK_A1,   (uint32_t)sp + XT_STK_FRMSZ);   /* physical top of stack frame      */
    SET_STKREG(XT_STK_A2,   parameter);                     /* parameters      */
    SET_STKREG(XT_STK_EXIT, _xt_user_exit);                 /* user exception exit dispatcher   */

    /* Set initial PS to int level 0, EXCM disabled ('rfe' will enable), user mode. */
    SET_STKREG(XT_STK_PS,      PS_UM | PS_EXCM);

    extern void dump_memory(char *name, uint8_t *data, uint32_t len);
    dump_memory(g_task_name, sp, 100);

    return (uint8_t *)sp;
}

#if 1
void rt_hw_context_switch_to(rt_uint32_t to)
{
    ets_printf("%s\r\n", __func__);
    rt_interrupt_from_thread = 0;
    rt_interrupt_to_thread = to;
    extern void _xt_enter_first_task(void);
    _xt_enter_first_task();
}

#define offsetof(TYPE, MEMBER) ((size_t) &((TYPE *)0)->MEMBER)
#define container_of(ptr, TYPE, MEMBER) ({ \
    const typeof( ((TYPE *)0)->MEMBER ) *__mptr = (ptr); \
    (TYPE *)( (char *)__mptr - offsetof(TYPE, MEMBER) );})

void rt_hw_context_switch(rt_uint32_t from, rt_uint32_t to)
{
    if (s_switch_ctx_flag) {
        return;
    }
    ets_printf("---%s (%p) <- (%p)\r\n", __func__, to, from);
    if (rt_thread_switch_interrupt_flag != 1) {
        rt_interrupt_from_thread = from;
    }
    rt_interrupt_to_thread = to;
    extern rt_thread_t rt_current_thread;
    rt_current_thread = container_of(from, struct rt_thread, sp);
    PendSV(1);
}

void rt_hw_context_switch_interrupt(rt_uint32_t from, rt_uint32_t to)
{
    ets_printf("%s %p <- %p\r\n", __func__, to, from);
    if (rt_thread_switch_interrupt_flag != 1) {
        rt_interrupt_from_thread = from;
        rt_thread_switch_interrupt_flag = 1;
    }
    rt_interrupt_to_thread = to;
}
#endif

/***************************** EOF **********************************/
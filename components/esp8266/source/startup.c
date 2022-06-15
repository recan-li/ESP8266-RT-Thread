// Copyright 2018-2019 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <string.h>

#include "sdkconfig.h"

#include "nvs_flash.h"
#include "tcpip_adapter.h"

#include "esp_log.h"
#include "esp_image_format.h"
#include "esp_phy_init.h"
#include "esp_heap_caps_init.h"
#include "esp_task_wdt.h"
#include "esp_private/wifi.h"
#include "esp_private/esp_system_internal.h"
#include "esp8266/eagle_soc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "esp_task.h"

#include "esp_newlib.h"
#include "esp_log.h"
#include "portmacro.h"

#if CONFIG_ENABLE_ESP_OSAL_RTTHREAD
#include "rtthread.h"
#endif

extern esp_err_t esp_pthread_init(void);
extern void chip_boot(void);
extern int base_gpio_init(void);

static inline int should_load(uint32_t load_addr)
{
    if (IS_USR_RTC(load_addr)) {
        if (esp_reset_reason_early() == ESP_RST_DEEPSLEEP)
            return 0;
    }

    if (IS_FLASH(load_addr))
        return 0;

    return 1;
}

putchar_like_t esp_log_set_putchar(putchar_like_t func);

static int my_local_putc(int ch)
{
    return ets_putc(ch);
}

void user_init_entry(void *param)
{
    void (**func)(void);

    esp_log_set_putchar(my_local_putc);

    extern void (*__init_array_start)(void);
    extern void (*__init_array_end)(void);

    extern void app_main(void);
    extern uint32_t esp_get_time(void);

    /* initialize C++ construture function */
    for (func = &__init_array_start; func < &__init_array_end; func++)
        func[0]();

    esp_phy_init_clk();

    assert(base_gpio_init() == 0);

    if (esp_reset_reason_early() != ESP_RST_FAST_SW) {
        assert(esp_mac_init() == ESP_OK);
    }

#if CONFIG_RESET_REASON
    esp_reset_reason_init();
#endif

#ifdef CONFIG_ESP_TASK_WDT
    esp_task_wdt_init();
#endif

    assert(esp_pthread_init() == 0);

#ifdef CONFIG_BOOTLOADER_FAST_BOOT
    REG_CLR_BIT(DPORT_CTL_REG, DPORT_CTL_DOUBLE_CLK);
#endif

#ifdef CONFIG_ESP8266_DEFAULT_CPU_FREQ_160
    esp_set_cpu_freq(ESP_CPU_FREQ_160M);
#endif

    RT_DEBUG_MORE("");

    app_main();

    RT_DEBUG_MORE("");

    while(1) { //wait here
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

#if 0
STRUCT_FIELD (long, 4, XT_STK_EXIT,     exit) /* exit point for dispatch */
STRUCT_FIELD (long, 4, XT_STK_PC,       pc)   /* return PC */
STRUCT_FIELD (long, 4, XT_STK_PS,       ps)   /* return PS */
STRUCT_FIELD (long, 4, XT_STK_A0,       a0)
STRUCT_FIELD (long, 4, XT_STK_A1,       a1)   /* stack pointer before interrupt */
STRUCT_FIELD (long, 4, XT_STK_A2,       a2)
STRUCT_FIELD (long, 4, XT_STK_A3,       a3)
STRUCT_FIELD (long, 4, XT_STK_A4,       a4)
STRUCT_FIELD (long, 4, XT_STK_A5,       a5)
STRUCT_FIELD (long, 4, XT_STK_A6,       a6)
STRUCT_FIELD (long, 4, XT_STK_A7,       a7)
STRUCT_FIELD (long, 4, XT_STK_A8,       a8)
STRUCT_FIELD (long, 4, XT_STK_A9,       a9)
STRUCT_FIELD (long, 4, XT_STK_A10,      a10)
STRUCT_FIELD (long, 4, XT_STK_A11,      a11)
STRUCT_FIELD (long, 4, XT_STK_A12,      a12)
STRUCT_FIELD (long, 4, XT_STK_A13,      a13)
STRUCT_FIELD (long, 4, XT_STK_A14,      a14)
STRUCT_FIELD (long, 4, XT_STK_A15,      a15)
STRUCT_FIELD (long, 4, XT_STK_SAR,      sar)    
#endif

void dump_memory(char *name, uint8_t *data, uint32_t len)
{
#if 0
    int i;
    uint32_t *p;

    len = 80;
    p = (uint32_t *)data + 4 * (len / 4 - 1) / 4;
    ets_printf("[%s] sp memory: %p (%p) : %d bytes\r\n", name, data, p, len);
    ets_printf("   REG          address       value\r\n");
    ets_printf("XT_STK_SAR : 0x%08x -> 0x%08x\n", p, *p); p--;
    ets_printf("XT_STK_A15 : 0x%08x -> 0x%08x\n", p, *p); p--;
    ets_printf("XT_STK_A14 : 0x%08x -> 0x%08x\n", p, *p); p--;
    ets_printf("XT_STK_A13 : 0x%08x -> 0x%08x\n", p, *p); p--;
    ets_printf("XT_STK_A12 : 0x%08x -> 0x%08x\n", p, *p); p--;
    ets_printf("XT_STK_A11 : 0x%08x -> 0x%08x\n", p, *p); p--;
    ets_printf("XT_STK_A10 : 0x%08x -> 0x%08x\n", p, *p); p--;
    ets_printf("XT_STK_A09 : 0x%08x -> 0x%08x\n", p, *p); p--;
    ets_printf("XT_STK_A08 : 0x%08x -> 0x%08x\n", p, *p); p--;
    ets_printf("XT_STK_A07 : 0x%08x -> 0x%08x\n", p, *p); p--;
    ets_printf("XT_STK_A06 : 0x%08x -> 0x%08x\n", p, *p); p--;
    ets_printf("XT_STK_A05 : 0x%08x -> 0x%08x\n", p, *p); p--;
    ets_printf("XT_STK_A04 : 0x%08x -> 0x%08x\n", p, *p); p--;
    ets_printf("XT_STK_A03 : 0x%08x -> 0x%08x\n", p, *p); p--;
    ets_printf("XT_STK_A02 : 0x%08x -> 0x%08x\n", p, *p); p--;
    ets_printf("XT_STK_A01 : 0x%08x -> 0x%08x\n", p, *p); p--;
    ets_printf("XT_STK_A00 : 0x%08x -> 0x%08x\n", p, *p); p--;
    ets_printf("XT_STK_PS  : 0x%08x -> 0x%08x\n", p, *p); p--;
    ets_printf("XT_STK_PC  : 0x%08x -> 0x%08x\n", p, *p); p--;
    ets_printf("XT_STK_EXIT: 0x%08x -> 0x%08x\n", p, *p); p--;
#endif
}

void test_task_entry_h(void *param)
{
    int i;

    ets_printf("%s start ... %p\r\n", __func__, &i);

#ifdef CONFIG_ENABLE_ESP_OSAL_RTTHREAD
    dump_memory(rt_thread_self()->name, rt_thread_self()->sp, 80);
#else
    extern void dump_cur_task_sp_memory(void);
    dump_cur_task_sp_memory();
#endif

    for (i = 0; i < 10000000; i++) {
        ets_printf("-------------> h + hello 222 ... %d, %d\n", (int)xTaskGetTickCount(), i);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        {
            uint8_t *p = NULL;
            //*p = 0x00;
        }
    }

    user_init_entry(NULL);

    RT_DEBUG_MORE("");

    while(1) { //wait here
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

#ifdef CONFIG_ENABLE_ESP_OSAL_RTTHREAD

#include "rtthread.h"

static int cnt = 0;
static rt_timer_t timer1;

/* 定时器 1 超时函数 */
static void timeout1(void *parameter)
{
    rt_kprintf("periodic timer is timeout %d\n", cnt);

    /* 运行第 10 次，停止周期定时器 */
    if (cnt++>= 9)
    {
        rt_timer_stop(timer1);
        rt_kprintf("periodic timer was stopped! \n");
    }
}

#endif

void test_task_entry(void *param)
{
    int i;

    ets_printf("%s start ... %p\r\n", __func__, &i);

#ifdef CONFIG_ENABLE_ESP_OSAL_RTTHREAD
    dump_memory(rt_thread_self()->name, rt_thread_self()->sp, 80);
#else
    extern void dump_cur_task_sp_memory(void);
    dump_cur_task_sp_memory();
#endif

    for (i = 0; i < 10000000; i++) {
#ifdef CONFIG_ENABLE_ESP_OSAL_RTTHREAD
        if (i == 5) {
            /* 创建定时器 1  周期定时器 */
            timer1 = rt_timer_create("timer1", timeout1,
                             RT_NULL, 10,
                             RT_TIMER_FLAG_PERIODIC | RT_TIMER_FLAG_SOFT_TIMER);

            /* 启动定时器 1 */
            if (timer1 != RT_NULL) {
                rt_timer_start(timer1);
            }
        }
#endif
        ets_printf("-------------> hello 111 ... %d, %d\n", (int)xTaskGetTickCount(), i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    user_init_entry(NULL);

    RT_DEBUG_MORE("");

    while(1) { //wait here
#ifdef CONFIG_ENABLE_ESP_OSAL_RTTHREAD
        rt_schedule();
#endif
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

#ifndef xtbsp_clock_freq_hz
#define xtbsp_clock_freq_hz() 80000000
#endif

extern void xPortSysTickHandle(void *p);

extern void IRAM_ATTR SoftIsrHdl(void* arg);

void sys_tick_int_init(void)
{
    /*******software isr*********/
    _xt_isr_attach(ETS_SOFT_INUM, SoftIsrHdl, NULL);

    _xt_isr_unmask(1 << ETS_SOFT_INUM);

    _xt_isr_attach(ETS_MAX_INUM, xPortSysTickHandle, NULL);

    /* Initialize system tick timer interrupt and schedule the first tick. */
    _xt_tick_divisor = xtbsp_clock_freq_hz() / CONFIG_FREERTOS_HZ;

    RT_DEBUG_MORE("");

    _xt_tick_timer_init(); 

    RT_DEBUG_MORE("");
}

void call_start_cpu(size_t start_addr)
{
    int i;
    int *p;

    extern int _bss_start, _bss_end;
    extern int _iram_bss_start, _iram_bss_end;

#ifdef CONFIG_BOOTLOADER_FAST_BOOT
    REG_SET_BIT(DPORT_CTL_REG, DPORT_CTL_DOUBLE_CLK);
#endif

    esp_image_header_t *head = (esp_image_header_t *)(FLASH_BASE + (start_addr & (FLASH_SIZE - 1)));
    esp_image_segment_header_t *segment = (esp_image_segment_header_t *)((uintptr_t)head + sizeof(esp_image_header_t));

    /* The data in flash cannot be accessed by byte in this stage, so just access by word and get the segment count. */
    uint8_t segment_count = ((*(volatile uint32_t *)head) & 0xFF00) >> 8;

    for (i = 0; i < segment_count - 1; i++) {
        segment = (esp_image_segment_header_t *)((uintptr_t)segment + sizeof(esp_image_segment_header_t) + segment->data_len);
        if (!should_load(segment->load_addr))
            continue;

        uint32_t *dest = (uint32_t *)segment->load_addr;
        uint32_t *src = (uint32_t *)((uintptr_t)segment + sizeof(esp_image_segment_header_t));
        uint32_t size = segment->data_len / sizeof(uint32_t);

        while (size--)
            *dest++ = *src++;
    }

    /* 
     * When finish copying IRAM program, the exception vect must be initialized.
     * And then user can load/store data which is not aligned by 4-byte.
     */
    __asm__ __volatile__(
        "movi       a0, 0x40100000\n"
        "wsr        a0, vecbase\n"
        : : :"memory");

#ifndef CONFIG_BOOTLOADER_INIT_SPI_FLASH
    chip_boot();
#endif

    /* clear bss data */
    for (p = &_bss_start; p < &_bss_end; p++)
        *p = 0;

    for (p = &_iram_bss_start; p < &_iram_bss_end; p++)
        *p = 0;

    __asm__ __volatile__(
        "rsil       a2, 2\n"
        "movi       a1, _chip_interrupt_tmp\n"
        : : :"memory");
    
    heap_caps_init();

#ifdef CONFIG_INIT_OS_BEFORE_START
    extern int __esp_os_init(void);
    assert(__esp_os_init() == 0);
#endif

#if CONFIG_ENABLE_ESP_OSAL_RTTHREAD
    /* timer system initialization */
    rt_system_timer_init();

    /* scheduler system initialization */
    rt_system_scheduler_init();
#endif

    assert(esp_newlib_init() == 0);

#ifdef CONFIG_ENABLE_ESP_OSAL_RTTHREAD
    #define offsetof(type, member) (size_t)&(((type*)0)->member)
    ets_printf("offsetof sp: %d 0x%02x %d\r\n", \
        offsetof(struct rt_thread, sp), offsetof(struct rt_thread, sp), sizeof(rt_list_t));
#endif

    assert(xTaskCreate(test_task_entry_h, "test-h", ESP_TASK_MAIN_STACK, NULL, 13, NULL) == pdPASS);

    assert(xTaskCreate(test_task_entry, "test", ESP_TASK_MAIN_STACK, NULL, 12, NULL) == pdPASS);

    //assert(xTaskCreate(user_init_entry, "uiT", ESP_TASK_MAIN_STACK, NULL, ESP_TASK_MAIN_PRIO, NULL) == pdPASS);

    vTaskStartScheduler();

    /* never run to here ! */
}

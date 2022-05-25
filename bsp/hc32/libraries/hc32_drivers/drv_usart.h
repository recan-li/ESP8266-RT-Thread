/*
 * Copyright (C) 2022, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-28     CDT          first version
 */

#ifndef __DRV_USART_H__
#define __DRV_USART_H__

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <rtthread.h>
#include "rtdevice.h"
#include "drv_irq.h"
#include "drv_dma.h"

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/
struct hc32_uart_irq_config
{
    struct hc32_irq_config irq_config;
    func_ptr_t             irq_callback;
};

/* HC32 config Rx timeout */
struct hc32_uart_rxto
{
    CM_TMR0_TypeDef             *TMR0_Instance;
    rt_uint32_t                 channel;
    rt_uint32_t                 clock;
    rt_size_t                   timeout_bits;
    struct hc32_irq_config      irq_config;
    func_ptr_t                  irq_callback;
};

/* HC32 config uart class */
struct hc32_uart_config
{
    const char                  *name;
    CM_USART_TypeDef            *Instance;
    rt_uint32_t                 clock;
    struct hc32_uart_irq_config rxerr_irq;
    struct hc32_uart_irq_config rx_irq;
    struct hc32_uart_irq_config tx_irq;
#ifdef RT_SERIAL_USING_DMA
    struct hc32_uart_rxto       *rx_timeout;
    struct dma_config           *dma_rx;
    struct hc32_uart_irq_config *tc_irq;
    struct dma_config           *dma_tx;
#endif
};

/* HC32 uart dirver class */
struct hc32_uart
{
    struct hc32_uart_config *config;
#ifdef RT_SERIAL_USING_DMA
    rt_size_t               dma_rx_last_index;
#endif
    rt_uint16_t             uart_dma_flag;
    struct rt_serial_device serial;
};

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/

/*******************************************************************************
 * Global function prototypes (definition in C source)
 ******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* __DRV_USART_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/

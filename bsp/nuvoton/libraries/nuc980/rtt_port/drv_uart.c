/**************************************************************************//**
*
* @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*
* SPDX-License-Identifier: Apache-2.0
*
* Change Logs:
* Date            Author       Notes
* 2020-12-12      Wayne        First version
*
******************************************************************************/

#include <rtconfig.h>

#if defined(BSP_USING_UART)

#include <rtdevice.h>
#include <rthw.h>
#include "NuMicro.h"
#include <drv_uart.h>
#include <drv_sys.h>

#if defined(RT_SERIAL_USING_DMA)
    #include <drv_pdma.h>
#endif

/* Private define ---------------------------------------------------------------*/

enum
{
    UART_START = -1,
#if defined(BSP_USING_UART0)
    UART0_IDX,
#endif
#if defined(BSP_USING_UART1)
    UART1_IDX,
#endif
#if defined(BSP_USING_UART2)
    UART2_IDX,
#endif
#if defined(BSP_USING_UART3)
    UART3_IDX,
#endif
#if defined(BSP_USING_UART4)
    UART4_IDX,
#endif
#if defined(BSP_USING_UART5)
    UART5_IDX,
#endif
#if defined(BSP_USING_UART6)
    UART6_IDX,
#endif
#if defined(BSP_USING_UART7)
    UART7_IDX,
#endif
#if defined(BSP_USING_UART8)
    UART8_IDX,
#endif
#if defined(BSP_USING_UART9)
    UART9_IDX,
#endif
    UART_CNT
};

/* Private typedef --------------------------------------------------------------*/
struct nu_uart
{
    rt_serial_t dev;
    char *name;
    UART_T *uart_base;
    IRQn_Type irqn;
    E_SYS_IPRST rstidx;
    E_SYS_IPCLK clkidx;

#if defined(RT_SERIAL_USING_DMA)
    uint32_t dma_flag;
    int16_t pdma_perp_tx;
    int8_t  pdma_chanid_tx;

    int16_t pdma_perp_rx;
    int8_t  pdma_chanid_rx;
    int32_t rx_write_offset;
    int32_t rxdma_trigger_len;

    nu_pdma_desc_t pdma_rx_desc;
#endif

};
typedef struct nu_uart *nu_uart_t;

/* Private functions ------------------------------------------------------------*/
static rt_err_t nu_uart_configure(struct rt_serial_device *serial, struct serial_configure *cfg);
static rt_err_t nu_uart_control(struct rt_serial_device *serial, int cmd, void *arg);
static int nu_uart_send(struct rt_serial_device *serial, char c);
static int nu_uart_receive(struct rt_serial_device *serial);

#if defined(RT_SERIAL_USING_DMA)
    static rt_size_t nu_uart_dma_transmit(struct rt_serial_device *serial, rt_uint8_t *buf, rt_size_t size, int direction);
    static void nu_pdma_uart_rx_cb(void *pvOwner, uint32_t u32Events);
    static void nu_pdma_uart_tx_cb(void *pvOwner, uint32_t u32Events);
#endif

/* Public functions ------------------------------------------------------------*/

/* Private variables ------------------------------------------------------------*/

static const struct rt_uart_ops nu_uart_ops =
{
    .configure = nu_uart_configure,
    .control = nu_uart_control,
    .putc = nu_uart_send,
    .getc = nu_uart_receive,
#if defined(RT_SERIAL_USING_DMA)
    .dma_transmit = nu_uart_dma_transmit
#else
    .dma_transmit = RT_NULL
#endif
};

static const struct serial_configure nu_uart_default_config =
        RT_SERIAL_CONFIG_DEFAULT;

static struct nu_uart nu_uart_arr [] =
{
#if defined(BSP_USING_UART0)
    {
        .name = "uart0",
        .uart_base = UART0,
        .irqn = IRQ_UART0,
        .rstidx = UART0RST,
        .clkidx = UART0CKEN,

#if defined(RT_SERIAL_USING_DMA)
#if defined(BSP_USING_UART0_TX_DMA)
        .pdma_perp_tx = PDMA_UART0_TX,
#else
        .pdma_perp_tx = NU_PDMA_UNUSED,
#endif
#if defined(BSP_USING_UART0_RX_DMA)
        .pdma_perp_rx = PDMA_UART0_RX,
        .rx_write_offset = 0,
#else
        .pdma_perp_rx = NU_PDMA_UNUSED,
#endif
#endif
    },
#endif

#if defined(BSP_USING_UART1)
    {
        .name = "uart1",
        .uart_base = UART1,
        .irqn = IRQ_UART1,
        .rstidx = UART1RST,
        .clkidx = UART1CKEN,

#if defined(RT_SERIAL_USING_DMA)
#if defined(BSP_USING_UART1_TX_DMA)
        .pdma_perp_tx = PDMA_UART1_TX,
#else
        .pdma_perp_tx = NU_PDMA_UNUSED,
#endif
#if defined(BSP_USING_UART1_RX_DMA)
        .pdma_perp_rx = PDMA_UART1_RX,
        .rx_write_offset = 0,
#else
        .pdma_perp_rx = NU_PDMA_UNUSED,
#endif
#endif
    },
#endif

#if defined(BSP_USING_UART2)
    {
        .name = "uart2",
        .uart_base = UART2,
        .irqn = IRQ_UART2,
        .rstidx = UART2RST,
        .clkidx = UART2CKEN,

#if defined(RT_SERIAL_USING_DMA)
#if defined(BSP_USING_UART2_TX_DMA)
        .pdma_perp_tx = PDMA_UART2_TX,
#else
        .pdma_perp_tx = NU_PDMA_UNUSED,
#endif
#if defined(BSP_USING_UART2_RX_DMA)
        .pdma_perp_rx = PDMA_UART2_RX,
        .rx_write_offset = 0,
#else
        .pdma_perp_rx = NU_PDMA_UNUSED,
#endif
#endif
    },
#endif

#if defined(BSP_USING_UART3)
    {
        .name = "uart3",
        .uart_base = UART3,
        .irqn = IRQ_UART3,
        .rstidx = UART3RST,
        .clkidx = UART3CKEN,

#if defined(RT_SERIAL_USING_DMA)
#if defined(BSP_USING_UART3_TX_DMA)
        .pdma_perp_tx = PDMA_UART3_TX,
#else
        .pdma_perp_tx = NU_PDMA_UNUSED,
#endif
#if defined(BSP_USING_UART3_RX_DMA)
        .pdma_perp_rx = PDMA_UART3_RX,
        .rx_write_offset = 0,
#else
        .pdma_perp_rx = NU_PDMA_UNUSED,
#endif
#endif
    },
#endif

#if defined(BSP_USING_UART4)
    {
        .name = "uart4",
        .uart_base = UART4,
        .irqn = IRQ_UART4,
        .rstidx = UART4RST,
        .clkidx = UART4CKEN,

#if defined(RT_SERIAL_USING_DMA)
#if defined(BSP_USING_UART4_TX_DMA)
        .pdma_perp_tx = PDMA_UART4_TX,
#else
        .pdma_perp_tx = NU_PDMA_UNUSED,
#endif
#if defined(BSP_USING_UART4_RX_DMA)
        .pdma_perp_rx = PDMA_UART4_RX,
        .rx_write_offset = 0,
#else
        .pdma_perp_rx = NU_PDMA_UNUSED,
#endif
#endif
    },
#endif

#if defined(BSP_USING_UART5)
    {
        .name = "uart5",
        .uart_base = UART5,
        .irqn = IRQ_UART5,
        .rstidx = UART5RST,
        .clkidx = UART5CKEN,

#if defined(RT_SERIAL_USING_DMA)
#if defined(BSP_USING_UART5_TX_DMA)
        .pdma_perp_tx = PDMA_UART5_TX,
#else
        .pdma_perp_tx = NU_PDMA_UNUSED,
#endif
#if defined(BSP_USING_UART5_RX_DMA)
        .pdma_perp_rx = PDMA_UART5_RX,
        .rx_write_offset = 0,
#else
        .pdma_perp_rx = NU_PDMA_UNUSED,
#endif
#endif
    },
#endif

#if defined(BSP_USING_UART6)
    {
        .name = "uart6",
        .uart_base = UART6,
        .irqn = IRQ_UART6,
        .rstidx = UART6RST,
        .clkidx = UART6CKEN,

#if defined(RT_SERIAL_USING_DMA)
#if defined(BSP_USING_UART6_TX_DMA)
        .pdma_perp_tx = PDMA_UART6_TX,
#else
        .pdma_perp_tx = NU_PDMA_UNUSED,
#endif
#if defined(BSP_USING_UART6_RX_DMA)
        .pdma_perp_rx = PDMA_UART6_RX,
        .rx_write_offset = 0,
#else
        .pdma_perp_rx = NU_PDMA_UNUSED,
#endif
#endif
    },
#endif

#if defined(BSP_USING_UART7)
    {
        .name = "uart7",
        .uart_base = UART7,
        .irqn = IRQ_UART7,
        .rstidx = UART7RST,
        .clkidx = UART7CKEN,

#if defined(RT_SERIAL_USING_DMA)
#if defined(BSP_USING_UART7_TX_DMA)
        .pdma_perp_tx = PDMA_UART7_TX,
#else
        .pdma_perp_tx = NU_PDMA_UNUSED,
#endif
#if defined(BSP_USING_UART7_RX_DMA)
        .pdma_perp_rx = PDMA_UART7_RX,
        .rx_write_offset = 0,
#else
        .pdma_perp_rx = NU_PDMA_UNUSED,
#endif
#endif
    },
#endif

#if defined(BSP_USING_UART8)
    {
        .name = "uart8",
        .uart_base = UART8,
        .irqn = IRQ_UART8,
        .rstidx = UART8RST,
        .clkidx = UART8CKEN,

#if defined(RT_SERIAL_USING_DMA)
#if defined(BSP_USING_UART8_TX_DMA)
        .pdma_perp_tx = PDMA_UART8_TX,
#else
        .pdma_perp_tx = NU_PDMA_UNUSED,
#endif
#if defined(BSP_USING_UART8_RX_DMA)
        .pdma_perp_rx = PDMA_UART8_RX,
        .rx_write_offset = 0,
#else
        .pdma_perp_rx = NU_PDMA_UNUSED,
#endif
#endif
    },
#endif

#if defined(BSP_USING_UART9)
    {
        .name = "uart9",
        .uart_base = UART9,
        .irqn = IRQ_UART9,
        .rstidx = UART9RST,
        .clkidx = UART9CKEN,

#if defined(RT_SERIAL_USING_DMA)
#if defined(BSP_USING_UART9_TX_DMA)
        .pdma_perp_tx = PDMA_UART9_TX,
#else
        .pdma_perp_tx = NU_PDMA_UNUSED,
#endif
#if defined(BSP_USING_UART9_RX_DMA)
        .pdma_perp_rx = PDMA_UART9_RX,
        .rx_write_offset = 0,
#else
        .pdma_perp_rx = NU_PDMA_UNUSED,
#endif
#endif
    },
#endif

}; /* uart nu_uart */

/**
 * All UART interrupt service routine
 */
static void nu_uart_isr(int vector, void *param)
{
    /* Get base address of uart register */
    nu_uart_t serial = (nu_uart_t)param;
    UART_T *uart_base = ((nu_uart_t)serial)->uart_base;

    /* Get interrupt event */
    uint32_t u32IntSts = uart_base->INTSTS;
    uint32_t u32FIFOSts = uart_base->FIFOSTS;

#if defined(RT_SERIAL_USING_DMA)
    if (u32IntSts & UART_INTSTS_HWRLSIF_Msk)
    {
        /* Drain RX FIFO to remove remain FEF frames in FIFO. */
        uart_base->FIFO |= UART_FIFO_RXRST_Msk;
        uart_base->FIFOSTS |= (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk);
        return;
    }
#endif

    /* Handle RX event */
    if (u32IntSts & (UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))
    {
        rt_hw_serial_isr(&serial->dev, RT_SERIAL_EVENT_RX_IND);
    }
    uart_base->INTSTS = u32IntSts;
    uart_base->FIFOSTS = u32FIFOSts;
}

/**
 * Set RS-485 AUD mode
 */
void nu_uart_set_rs485aud(struct rt_serial_device *serial, rt_bool_t bRTSActiveLowLevel)
{
    UART_T *uart_base;
    RT_ASSERT(serial != RT_NULL);

    /* Get base address of uart register */
    uart_base = ((nu_uart_t)serial)->uart_base;

    /* Set RTS as RS-485 phy direction controlling ping. */
    UART_SelectRS485Mode(uart_base, UART_ALTCTL_RS485AUD_Msk, 0);

    if (bRTSActiveLowLevel)
    {
        /* Set direction pin as active-low. */
        uart_base->MODEM |= UART_MODEM_RTSACTLV_Msk;
    }
    else
    {
        /* Set direction pin as active-high. */
        uart_base->MODEM &= ~UART_MODEM_RTSACTLV_Msk;
    }

    rt_kprintf("Set %s to RS-485 AUD function mode. ActiveLowLevel-%s\n", ((nu_uart_t)serial)->name, bRTSActiveLowLevel ? "YES" : "NO");
}

/**
 * Configure uart port
 */
static rt_err_t nu_uart_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    rt_err_t ret = RT_EOK;
    uint32_t uart_word_len = 0;
    uint32_t uart_stop_bit = 0;
    uint32_t uart_parity = 0;

    /* Get base address of uart register */
    UART_T *uart_base = ((nu_uart_t)serial)->uart_base;

    /* Check baudrate */
    RT_ASSERT(cfg->baud_rate != 0);

    /* Check word len */
    switch (cfg->data_bits)
    {
    case DATA_BITS_5:
        uart_word_len = UART_WORD_LEN_5;
        break;

    case DATA_BITS_6:
        uart_word_len = UART_WORD_LEN_6;
        break;

    case DATA_BITS_7:
        uart_word_len = UART_WORD_LEN_7;
        break;

    case DATA_BITS_8:
        uart_word_len = UART_WORD_LEN_8;
        break;

    default:
        rt_kprintf("Unsupported data length");
        ret = RT_EINVAL;
        goto exit_nu_uart_configure;
    }

    /* Check stop bit */
    switch (cfg->stop_bits)
    {
    case STOP_BITS_1:
        uart_stop_bit = UART_STOP_BIT_1;
        break;

    case STOP_BITS_2:
        uart_stop_bit = UART_STOP_BIT_2;
        break;

    default:
        rt_kprintf("Unsupported stop bit");
        ret = RT_EINVAL;
        goto exit_nu_uart_configure;
    }

    /* Check parity */
    switch (cfg->parity)
    {
    case PARITY_NONE:
        uart_parity = UART_PARITY_NONE;
        break;

    case PARITY_ODD:
        uart_parity = UART_PARITY_ODD;
        break;

    case PARITY_EVEN:
        uart_parity = UART_PARITY_EVEN;
        break;

    default:
        rt_kprintf("Unsupported parity");
        ret = RT_EINVAL;
        goto exit_nu_uart_configure;
    }

    nu_sys_ip_reset(((nu_uart_t)serial)->rstidx);

    /* Open Uart and set UART Baudrate */
    UART_Open(uart_base, cfg->baud_rate);

    /* Set line configuration. */
    UART_SetLineConfig(uart_base, 0, uart_word_len, uart_parity, uart_stop_bit);

    /* Enable interrupt. */
    rt_hw_interrupt_umask(((nu_uart_t)serial)->irqn);

exit_nu_uart_configure:

    if (ret != RT_EOK)
        UART_Close(uart_base);

    return -(ret);
}

#if defined(RT_SERIAL_USING_DMA)

static rt_err_t nu_pdma_uart_rx_config(struct rt_serial_device *serial, uint8_t *pu8Buf, int32_t i32TriggerLen)
{
    rt_err_t result = RT_EOK;
    struct nu_pdma_chn_cb sChnCB;
    nu_uart_t psNuUart = (nu_uart_t)serial;

    /* Get base address of uart register */
    UART_T *uart_base = psNuUart->uart_base;

    /* Register ISR callback function */
    sChnCB.m_eCBType = eCBType_Event;
    sChnCB.m_pfnCBHandler = nu_pdma_uart_rx_cb;
    sChnCB.m_pvUserData = (void *)serial;

    nu_pdma_filtering_set(psNuUart->pdma_chanid_rx, NU_PDMA_EVENT_TRANSFER_DONE | NU_PDMA_EVENT_TIMEOUT);
    result = nu_pdma_callback_register(psNuUart->pdma_chanid_rx, &sChnCB);

    if (result != RT_EOK)
    {
        goto exit_nu_pdma_uart_rx_config;
    }

    if (serial->config.bufsz == 0)
    {
        result = nu_pdma_transfer(((nu_uart_t)serial)->pdma_chanid_rx,
                                  8,
                                  (uint32_t)uart_base,
                                  (uint32_t)pu8Buf,
                                  i32TriggerLen,
                                  1000);  //Idle-timeout, 1ms
        if (result != RT_EOK)
        {
            goto exit_nu_pdma_uart_rx_config;
        }
    }
    else
    {
        /* For Serial RX FIFO - Single buffer recycle SG trigger */
        /* Link to next */
        nu_pdma_desc_t next = psNuUart->pdma_rx_desc;

        result = nu_pdma_desc_setup(psNuUart->pdma_chanid_rx,
                                    psNuUart->pdma_rx_desc,
                                    8,
                                    (uint32_t)uart_base,
                                    (uint32_t)pu8Buf,
                                    i32TriggerLen,
                                    next,
                                    0);
        if (result != RT_EOK)
        {
            goto exit_nu_pdma_uart_rx_config;
        }

        /* Assign head descriptor & go */
        result = nu_pdma_sg_transfer(psNuUart->pdma_chanid_rx, psNuUart->pdma_rx_desc, 1000);
        if (result != RT_EOK)
        {
            goto exit_nu_pdma_uart_rx_config;
        }
    }

    /* Enable Receive Line interrupt & Start DMA RX transfer. */
    UART_ENABLE_INT(uart_base, UART_INTEN_RLSIEN_Msk | UART_INTEN_RXPDMAEN_Msk);

exit_nu_pdma_uart_rx_config:

    return result;
}

static void nu_pdma_uart_rx_cb(void *pvOwner, uint32_t u32Events)
{
    rt_size_t recv_len = 0;
    rt_size_t transferred_rxbyte = 0;
    struct rt_serial_device *serial = (struct rt_serial_device *)pvOwner;
    nu_uart_t puart = (nu_uart_t)serial;
    RT_ASSERT(serial != RT_NULL);

    /* Get base address of uart register */
    UART_T *uart_base = puart->uart_base;

    transferred_rxbyte = nu_pdma_transferred_byte_get(puart->pdma_chanid_rx, puart->rxdma_trigger_len);

    if (u32Events & (NU_PDMA_EVENT_TRANSFER_DONE | NU_PDMA_EVENT_TIMEOUT))
    {
#if defined(BSP_USING_MMU)
        struct rt_serial_rx_fifo *rx_fifo = (struct rt_serial_rx_fifo *)serial->serial_rx;
#endif
        if (u32Events & NU_PDMA_EVENT_TRANSFER_DONE)
        {
            transferred_rxbyte = puart->rxdma_trigger_len;
        }
        else if ((u32Events & NU_PDMA_EVENT_TIMEOUT) && !UART_GET_RX_EMPTY(uart_base))
        {
            return;
        }

        recv_len = transferred_rxbyte - puart->rx_write_offset;

#if defined(BSP_USING_MMU)
        mmu_invalidate_dcache((uint32_t)&rx_fifo->buffer[puart->rx_write_offset], recv_len);
#endif

        puart->rx_write_offset = transferred_rxbyte % puart->rxdma_trigger_len;
    }

    if ((serial->config.bufsz == 0) && (u32Events & NU_PDMA_EVENT_TRANSFER_DONE))
    {
        recv_len = puart->rxdma_trigger_len;
    }

    if (recv_len)
    {
        rt_hw_serial_isr(&puart->dev, RT_SERIAL_EVENT_RX_DMADONE | (recv_len << 8));
    }
}

static rt_err_t nu_pdma_uart_tx_config(struct rt_serial_device *serial)
{
    struct nu_pdma_chn_cb sChnCB;
    RT_ASSERT(serial != RT_NULL);

    /* Register ISR callback function */
    sChnCB.m_eCBType = eCBType_Event;
    sChnCB.m_pfnCBHandler = nu_pdma_uart_tx_cb;
    sChnCB.m_pvUserData = (void *)serial;

    nu_pdma_filtering_set(((nu_uart_t)serial)->pdma_chanid_tx, NU_PDMA_EVENT_TRANSFER_DONE);
    return nu_pdma_callback_register(((nu_uart_t)serial)->pdma_chanid_tx, &sChnCB);
}

static void nu_pdma_uart_tx_cb(void *pvOwner, uint32_t u32Events)
{
    nu_uart_t puart = (nu_uart_t)pvOwner;

    RT_ASSERT(puart != RT_NULL);

    UART_DISABLE_INT(puart->uart_base, UART_INTEN_TXPDMAEN_Msk);// Stop DMA TX transfer

    if (u32Events & NU_PDMA_EVENT_TRANSFER_DONE)
    {
        rt_hw_serial_isr(&puart->dev, RT_SERIAL_EVENT_TX_DMADONE);
    }
}

/**
 * Uart DMA transfer
 */
static rt_size_t nu_uart_dma_transmit(struct rt_serial_device *serial, rt_uint8_t *buf, rt_size_t size, int direction)
{
    rt_err_t result = RT_EOK;
    nu_uart_t psNuUart = (nu_uart_t)serial;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(buf != RT_NULL);

    /* Get base address of uart register */
    UART_T *uart_base = psNuUart->uart_base;
    if (direction == RT_SERIAL_DMA_TX)
    {
        result = nu_pdma_transfer(psNuUart->pdma_chanid_tx,
                                  8,
                                  (uint32_t)buf,
                                  (uint32_t)uart_base,
                                  size,
                                  0);  // wait-forever
        UART_ENABLE_INT(uart_base, UART_INTEN_TXPDMAEN_Msk);// Start DMA TX transfer
    }
    else if (direction == RT_SERIAL_DMA_RX)
    {
        UART_DISABLE_INT(uart_base, UART_INTEN_RLSIEN_Msk | UART_INTEN_RXPDMAEN_Msk); // Start DMA TX transfer

        // If config.bufsz = 0, serial will trigger once.
        psNuUart->rxdma_trigger_len = size;
        psNuUart->rx_write_offset = 0;
        result = nu_pdma_uart_rx_config(serial, buf, size);
    }
    else
    {
        result = RT_ERROR;
    }

    return result;
}

static int nu_hw_uart_dma_allocate(nu_uart_t pusrt)
{
    RT_ASSERT(pusrt != RT_NULL);

    /* Allocate UART_TX nu_dma channel */
    if (pusrt->pdma_perp_tx != NU_PDMA_UNUSED)
    {
        pusrt->pdma_chanid_tx = nu_pdma_channel_allocate(pusrt->pdma_perp_tx);
        if (pusrt->pdma_chanid_tx >= 0)
        {
            pusrt->dma_flag |= RT_DEVICE_FLAG_DMA_TX;
        }
    }

    /* Allocate UART_RX nu_dma channel */
    if (pusrt->pdma_perp_rx != NU_PDMA_UNUSED)
    {
        pusrt->pdma_chanid_rx = nu_pdma_channel_allocate(pusrt->pdma_perp_rx);
        if (pusrt->pdma_chanid_rx >= 0)
        {
            rt_err_t ret = RT_EOK;
            pusrt->dma_flag |= RT_DEVICE_FLAG_DMA_RX;
            ret = nu_pdma_sgtbls_allocate(&pusrt->pdma_rx_desc, 1);
            RT_ASSERT(ret == RT_EOK);
        }
    }

    return RT_EOK;
}
#endif

/**
 * Uart interrupt control
 */
static rt_err_t nu_uart_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    nu_uart_t psNuUart = (nu_uart_t)serial;
    rt_err_t result = RT_EOK;
    rt_uint32_t flag;
    rt_ubase_t ctrl_arg = (rt_ubase_t)arg;

    RT_ASSERT(serial != RT_NULL);

    /* Get base address of uart register */
    UART_T *uart_base = psNuUart->uart_base;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        if (ctrl_arg == RT_DEVICE_FLAG_INT_RX) /* Disable INT-RX */
        {
            flag = UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_TOCNTEN_Msk;
            UART_DISABLE_INT(uart_base, flag);
        }
        else if (ctrl_arg == RT_DEVICE_FLAG_DMA_RX) /* Disable DMA-RX */
        {
            /* Disable Receive Line interrupt & Stop DMA RX transfer. */
#if defined(RT_SERIAL_USING_DMA)
            nu_pdma_channel_terminate(psNuUart->pdma_chanid_rx);
            UART_DISABLE_INT(uart_base, UART_INTEN_RLSIEN_Msk | UART_INTEN_RXPDMAEN_Msk);
#endif
        }
        break;

    case RT_DEVICE_CTRL_SET_INT:
        if (ctrl_arg == RT_DEVICE_FLAG_INT_RX) /* Enable INT-RX */
        {
            flag = UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_TOCNTEN_Msk;
            UART_ENABLE_INT(uart_base, flag);
        }
        break;

#if defined(RT_SERIAL_USING_DMA)
    case RT_DEVICE_CTRL_CONFIG:
        if (ctrl_arg == RT_DEVICE_FLAG_DMA_RX) /* Configure and trigger DMA-RX */
        {
            struct rt_serial_rx_fifo *rx_fifo = (struct rt_serial_rx_fifo *)serial->serial_rx;
            psNuUart->rxdma_trigger_len = serial->config.bufsz;
            psNuUart->rx_write_offset = 0;

            result = nu_pdma_uart_rx_config(serial, &rx_fifo->buffer[0], psNuUart->rxdma_trigger_len);  // Config & trigger
        }
        else if (ctrl_arg == RT_DEVICE_FLAG_DMA_TX) /* Configure DMA-TX */
        {
            result = nu_pdma_uart_tx_config(serial);
        }
        break;
#endif

    case RT_DEVICE_CTRL_CLOSE:
        /* Disable interrupt. */
        rt_hw_interrupt_mask(psNuUart->irqn);

#if defined(RT_SERIAL_USING_DMA)
        nu_pdma_channel_terminate(psNuUart->pdma_chanid_tx);
        nu_pdma_channel_terminate(psNuUart->pdma_chanid_rx);
#endif

        /* Close UART port */
        UART_Close(uart_base);

        break;

    default:
        result = -RT_EINVAL;
        break;

    }
    return result;
}

/**
 * Uart put char
 */
static int nu_uart_send(struct rt_serial_device *serial, char c)
{
    RT_ASSERT(serial != RT_NULL);

    /* Get base address of uart register */
    UART_T *uart_base = ((nu_uart_t)serial)->uart_base;

    /* Waiting if TX-FIFO is full. */
    while (UART_IS_TX_FULL(uart_base));

    /* Put char into TX-FIFO */
    UART_WRITE(uart_base, c);

    return 1;
}

/**
 * Uart get char
 */
static int nu_uart_receive(struct rt_serial_device *serial)
{
    RT_ASSERT(serial != RT_NULL);

    /* Get base address of uart register */
    UART_T *uart_base = ((nu_uart_t)serial)->uart_base;

    /* Return failure if RX-FIFO is empty. */
    if (UART_GET_RX_EMPTY(uart_base))
    {
        return -1;
    }

    /* Get char from RX-FIFO */
    return UART_READ(uart_base);
}

/**
 * Hardware UART Initialization
 */
rt_err_t rt_hw_uart_init(void)
{
    int i;
    rt_uint32_t flag;
    rt_err_t ret = RT_EOK;

    for (i = (UART_START + 1); i < UART_CNT; i++)
    {
        flag = RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX;

        nu_uart_arr[i].dev.ops    = &nu_uart_ops;
        nu_uart_arr[i].dev.config = nu_uart_default_config;

#if defined(RT_SERIAL_USING_DMA)
        nu_uart_arr[i].dma_flag = 0;
        nu_hw_uart_dma_allocate(&nu_uart_arr[i]);
        flag |= nu_uart_arr[i].dma_flag;
#endif

        rt_hw_interrupt_install(nu_uart_arr[i].irqn, nu_uart_isr, &nu_uart_arr[i], nu_uart_arr[i].name);

        nu_sys_ipclk_enable(nu_uart_arr[i].clkidx);

        ret = rt_hw_serial_register(&nu_uart_arr[i].dev, nu_uart_arr[i].name, flag, NULL);
        RT_ASSERT(ret == RT_EOK);
    }

    return ret;
}

#endif //#if defined(BSP_USING_UART)

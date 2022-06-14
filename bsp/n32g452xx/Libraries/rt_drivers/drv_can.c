/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-01-09     chenbin      the first version
 */

#include "drv_can.h"
#ifdef BSP_USING_CAN

#include "stdint.h"
#include "n32g45x.h"
#include "n32g45x_can.h"

struct n32g45x_baud_rate_tab
{
    uint32_t baud_rate;
    uint16_t PRESCALE;
    uint8_t RSJW;
    uint8_t TBS1;
    uint8_t TBS2;
    uint8_t notused;
};

#define N32_CAN_BAUD_DEF(rate, rsjw, tbs1, tbs2, prescale) \
    {                                                      \
        .baud_rate = rate,                                 \
        .RSJW = rsjw,                                      \
        .TBS1 = tbs1,                                      \
        .TBS2 = tbs2,                                      \
        .PRESCALE = prescale                               \
    }

/* N32G45x can device */
struct n32g45x_can
{
    char *name;
    CAN_Module *CANx;
    CAN_InitType can_init;
    CAN_FilterInitType can_filter_init;
    struct rt_can_device device; /* inherit from can device */
};

#define LOG_TAG "drv_can"
#include <drv_log.h>

/*
* N32G45x CAN1 CAN2 used APB1 (36MHz)
* baud calculation example:  baud =  Tclk / ((ss + bs1 + bs2) * brp)
* 36MHz / ((1 + 5 + 3) * 4) = 1MHz
*/
#if defined(N32G45X) /* APB1 36MHz(max) */
static const struct n32g45x_baud_rate_tab can_baud_rate_tab[] =
    {
        N32_CAN_BAUD_DEF(CAN1MBaud, CAN_RSJW_1tq, CAN_TBS1_5tq, CAN_TBS2_3tq, 4),
        N32_CAN_BAUD_DEF(CAN800kBaud, CAN_RSJW_1tq, CAN_TBS1_5tq, CAN_TBS2_3tq, 5),
        N32_CAN_BAUD_DEF(CAN500kBaud, CAN_RSJW_1tq, CAN_TBS1_5tq, CAN_TBS2_3tq, 8),
        N32_CAN_BAUD_DEF(CAN250kBaud, CAN_RSJW_1tq, CAN_TBS1_5tq, CAN_TBS2_3tq, 16),
        N32_CAN_BAUD_DEF(CAN125kBaud, CAN_RSJW_1tq, CAN_TBS1_5tq, CAN_TBS2_3tq, 32),
        N32_CAN_BAUD_DEF(CAN100kBaud, CAN_RSJW_2tq, CAN_TBS1_8tq, CAN_TBS2_8tq, 20),
        N32_CAN_BAUD_DEF(CAN50kBaud, CAN_RSJW_2tq, CAN_TBS1_8tq, CAN_TBS2_8tq, 40),
        N32_CAN_BAUD_DEF(CAN20kBaud, CAN_RSJW_2tq, CAN_TBS1_8tq, CAN_TBS2_8tq, 80),
        N32_CAN_BAUD_DEF(CAN10kBaud, CAN_RSJW_2tq, CAN_TBS1_8tq, CAN_TBS2_8tq, 160),
};
#endif

#ifdef BSP_USING_CAN1
static struct n32g45x_can drv_can1 =
    {
        .name = "can1",
        .CANx = CAN1,
};
#endif

#ifdef BSP_USING_CAN2
static struct n32g45x_can drv_can2 =
    {
        .name = "can2",
        .CANx = CAN2,
};
#endif

static uint32_t get_can_baud_index(rt_uint32_t baud)
{
    uint32_t len, index;

    len = sizeof(can_baud_rate_tab) / sizeof(can_baud_rate_tab[0]);
    for (index = 0; index < len; index++)
    {
        if (can_baud_rate_tab[index].baud_rate == baud)
            return index;
    }
    return 0; /* default baud is CAN1MBaud */
}

static uint8_t get_can_mode_rtt2n32(uint8_t rtt_can_mode)
{
    uint8_t mode = CAN_Normal_Mode;
    switch (rtt_can_mode)
    {
    case RT_CAN_MODE_NORMAL:
        mode = CAN_Normal_Mode;
        break;
    case RT_CAN_MODE_LISEN:
        mode = CAN_Silent_Mode;
        break;
    case RT_CAN_MODE_LOOPBACK:
        mode = CAN_LoopBack_Mode;
        break;
    case RT_CAN_MODE_LOOPBACKANLISEN:
        mode = CAN_Silent_LoopBack_Mode;
        break;
    }
    return mode;
}

static rt_err_t _can_filter_config(struct n32g45x_can *drv_can)
{
    if (drv_can->CANx == CAN1)
    {
        CAN1_InitFilter(&(drv_can->can_filter_init));
    }
#ifdef CAN2
    else if (drv_can->CANx == CAN2)
    {
        CAN2_InitFilter(&(drv_can->can_filter_init));
    }
#endif
    else
    {
        rt_kprintf("can filter config error\n");
        return -RT_EINVAL;
    }
    return RT_EOK;
}

static rt_err_t _can_config(struct rt_can_device *can, struct can_configure *cfg)
{
    struct n32g45x_can *drv_can;
    rt_uint32_t baud_index;

    RT_ASSERT(can);
    RT_ASSERT(cfg);
    drv_can = (struct n32g45x_can *)can->parent.user_data;
    RT_ASSERT(drv_can);

    /* Configure CAN1 and CAN2 */
    if (drv_can->CANx == CAN1)
    {
        RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_CAN1, ENABLE);
        n32_msp_can_init(CAN1);
    }
#ifdef CAN2
    else if (drv_can->CANx == CAN2)
    {
        RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_CAN2, ENABLE);
        n32_msp_can_init(CAN2);
    }
#endif
    else
    {
        rt_kprintf("can init error1\n");
        return -RT_EINVAL;
    }
    /* Struct init*/
    CAN_InitStruct(&(drv_can->can_init));

    drv_can->can_init.TTCM = DISABLE;
    drv_can->can_init.ABOM = DISABLE;
    drv_can->can_init.AWKUM = DISABLE;
    drv_can->can_init.NART = DISABLE;
    drv_can->can_init.RFLM = DISABLE;
    drv_can->can_init.TXFP = ENABLE;

    //mode
    drv_can->can_init.OperatingMode = get_can_mode_rtt2n32(cfg->mode);

    //baud
    baud_index = get_can_baud_index(cfg->baud_rate);
    drv_can->can_init.RSJW = can_baud_rate_tab[baud_index].RSJW;
    drv_can->can_init.TBS1 = can_baud_rate_tab[baud_index].TBS1;
    drv_can->can_init.TBS2 = can_baud_rate_tab[baud_index].TBS2;
    drv_can->can_init.BaudRatePrescaler = can_baud_rate_tab[baud_index].PRESCALE;

    /* init can */
    if (CAN_Init(drv_can->CANx, &(drv_can->can_init)) != CAN_InitSTS_Success)
    {
        rt_kprintf("can init error2\n");
        return -RT_ERROR;
    }
    /* default filter config */
    _can_filter_config(drv_can);
    return RT_EOK;
}

#ifndef CAN1_TX_IRQn
#define CAN1_TX_IRQn USB_HP_CAN1_TX_IRQn
#endif

#ifndef CAN1_RX0_IRQn
#define CAN1_RX0_IRQn USB_LP_CAN1_RX0_IRQn
#endif

static rt_err_t _can_control(struct rt_can_device *can, int cmd, void *arg)
{
    rt_uint32_t argval;
    struct n32g45x_can *drv_can;
    struct rt_can_filter_config *filter_cfg;

    RT_ASSERT(can != RT_NULL);
    drv_can = (struct n32g45x_can *)can->parent.user_data;
    RT_ASSERT(drv_can != RT_NULL);

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        argval = (rt_uint32_t)arg;
        if (argval == RT_DEVICE_FLAG_INT_RX)
        {
            if (CAN1 == drv_can->CANx)
            {
                NVIC_DisableIRQ(CAN1_RX0_IRQn);
                NVIC_DisableIRQ(CAN1_RX1_IRQn);
            }
#ifdef CAN2
            if (CAN2 == drv_can->CANx)
            {
                NVIC_DisableIRQ(CAN2_RX0_IRQn);
                NVIC_DisableIRQ(CAN2_RX1_IRQn);
            }
#endif
            CAN_INTConfig(drv_can->CANx, CAN_INT_FMP0, DISABLE); /*!< DATFIFO 0 message pending Interrupt*/
            CAN_INTConfig(drv_can->CANx, CAN_INT_FF0, DISABLE);  /*!< DATFIFO 0 full Interrupt*/
            CAN_INTConfig(drv_can->CANx, CAN_INT_FOV0, DISABLE); /*!< DATFIFO 0 overrun Interrupt*/
            CAN_INTConfig(drv_can->CANx, CAN_INT_FMP1, DISABLE); /*!< DATFIFO 1 message pending Interrupt*/
            CAN_INTConfig(drv_can->CANx, CAN_INT_FF1, DISABLE);  /*!< DATFIFO 1 full Interrupt*/
            CAN_INTConfig(drv_can->CANx, CAN_INT_FOV1, DISABLE); /*!< DATFIFO 1 overrun Interrupt*/
        }
        else if (argval == RT_DEVICE_FLAG_INT_TX)
        {
            if (CAN1 == drv_can->CANx)
            {
                NVIC_DisableIRQ(CAN1_TX_IRQn);
            }
#ifdef CAN2
            if (CAN2 == drv_can->CANx)
            {
                NVIC_DisableIRQ(CAN2_TX_IRQn);
            }
#endif
            CAN_INTConfig(drv_can->CANx, CAN_INT_TME, DISABLE); /*!< Transmit mailbox empty Interrupt*/
        }
        else if (argval == RT_DEVICE_CAN_INT_ERR)
        {
            if (CAN1 == drv_can->CANx)
            {
                NVIC_DisableIRQ(CAN1_SCE_IRQn);
            }
#ifdef CAN2
            if (CAN2 == drv_can->CANx)
            {
                NVIC_DisableIRQ(CAN2_SCE_IRQn);
            }
#endif
            CAN_INTConfig(drv_can->CANx, CAN_INT_EWG, DISABLE); /*!< Error warning Interrupt*/
            CAN_INTConfig(drv_can->CANx, CAN_INT_EPV, DISABLE); /*!< Error passive Interrupt*/
            CAN_INTConfig(drv_can->CANx, CAN_INT_BOF, DISABLE); /*!< Bus-off Interrupt*/
            CAN_INTConfig(drv_can->CANx, CAN_INT_LEC, DISABLE); /*!< Last error code Interrupt*/
            CAN_INTConfig(drv_can->CANx, CAN_INT_ERR, DISABLE); /*!< Error Interrupt*/
        }
        break;
    case RT_DEVICE_CTRL_SET_INT:
        argval = (rt_uint32_t)arg;
        if (argval == RT_DEVICE_FLAG_INT_RX)
        {
            CAN_INTConfig(drv_can->CANx, CAN_INT_FMP0, ENABLE); /*!< DATFIFO 0 message pending Interrupt*/
            CAN_INTConfig(drv_can->CANx, CAN_INT_FF0, ENABLE);  /*!< DATFIFO 0 full Interrupt*/
            CAN_INTConfig(drv_can->CANx, CAN_INT_FOV0, ENABLE); /*!< DATFIFO 0 overrun Interrupt*/
            CAN_INTConfig(drv_can->CANx, CAN_INT_FMP1, ENABLE); /*!< DATFIFO 1 message pending Interrupt*/
            CAN_INTConfig(drv_can->CANx, CAN_INT_FF1, ENABLE);  /*!< DATFIFO 1 full Interrupt*/
            CAN_INTConfig(drv_can->CANx, CAN_INT_FOV1, ENABLE); /*!< DATFIFO 1 overrun Interrupt*/

            if (CAN1 == drv_can->CANx)
            {
                NVIC_SetPriority(CAN1_RX0_IRQn, 1);
                NVIC_EnableIRQ(CAN1_RX0_IRQn);
                NVIC_SetPriority(CAN1_RX1_IRQn, 1);
                NVIC_EnableIRQ(CAN1_RX1_IRQn);
            }
#ifdef CAN2
            if (CAN2 == drv_can->CANx)
            {
                NVIC_SetPriority(CAN2_RX0_IRQn, 1);
                NVIC_EnableIRQ(CAN2_RX0_IRQn);
                NVIC_SetPriority(CAN2_RX1_IRQn, 1);
                NVIC_EnableIRQ(CAN2_RX1_IRQn);
            }
#endif
        }
        else if (argval == RT_DEVICE_FLAG_INT_TX)
        {
            CAN_INTConfig(drv_can->CANx, CAN_INT_TME, ENABLE); /*!< Transmit mailbox empty Interrupt*/

            if (CAN1 == drv_can->CANx)
            {
                NVIC_SetPriority(CAN1_TX_IRQn, 1);
                NVIC_EnableIRQ(CAN1_TX_IRQn);
            }
#ifdef CAN2
            if (CAN2 == drv_can->CANx)
            {
                NVIC_SetPriority(CAN2_TX_IRQn, 1);
                NVIC_EnableIRQ(CAN2_TX_IRQn);
            }
#endif
        }
        else if (argval == RT_DEVICE_CAN_INT_ERR)
        {
            CAN_INTConfig(drv_can->CANx, CAN_INT_EWG, ENABLE); /*!< Error warning Interrupt*/
            CAN_INTConfig(drv_can->CANx, CAN_INT_EPV, ENABLE); /*!< Error passive Interrupt*/
            CAN_INTConfig(drv_can->CANx, CAN_INT_BOF, ENABLE); /*!< Bus-off Interrupt*/
            CAN_INTConfig(drv_can->CANx, CAN_INT_LEC, ENABLE); /*!< Last error code Interrupt*/
            CAN_INTConfig(drv_can->CANx, CAN_INT_ERR, ENABLE); /*!< Error Interrupt*/

            if (CAN1 == drv_can->CANx)
            {
                NVIC_SetPriority(CAN1_SCE_IRQn, 1);
                NVIC_EnableIRQ(CAN1_SCE_IRQn);
            }
#ifdef CAN2
            if (CAN2 == drv_can->CANx)
            {
                NVIC_SetPriority(CAN2_SCE_IRQn, 1);
                NVIC_EnableIRQ(CAN2_SCE_IRQn);
            }
#endif
        }
        break;
    case RT_CAN_CMD_SET_FILTER:
    {
        rt_uint32_t id_h = 0;
        rt_uint32_t id_l = 0;
        rt_uint32_t mask_h = 0;
        rt_uint32_t mask_l = 0;
        rt_uint32_t mask_l_tail = 0; //CAN_FxR2 bit [2:0]

        if (RT_NULL == arg)
        {
            /* default filter config */
            _can_filter_config(drv_can);
        }
        else
        {
            filter_cfg = (struct rt_can_filter_config *)arg;
            /* get default filter */
            for (int i = 0; i < filter_cfg->count; i++)
            {
                if (filter_cfg->items[i].hdr == -1)
                {
                    drv_can->can_filter_init.Filter_Num = i;
                }
                else
                {
                    drv_can->can_filter_init.Filter_Num = filter_cfg->items[i].hdr;
                }

                if (filter_cfg->items[i].mode == 0x00)
                {
                    drv_can->can_filter_init.Filter_Mode = CAN_Filter_IdMaskMode;
                }
                else if (filter_cfg->items[i].mode == 0x01)
                {
                    drv_can->can_filter_init.Filter_Mode = CAN_Filter_IdListMode;
                }

                if (filter_cfg->items[i].ide == RT_CAN_STDID)
                {
                    id_h = ((filter_cfg->items[i].id << 18) >> 13) & 0xFFFF;
                    id_l = ((filter_cfg->items[i].id << 18) |
                            (filter_cfg->items[i].ide << 2) |
                            (filter_cfg->items[i].rtr << 1)) &
                           0xFFFF;
                    mask_h = ((filter_cfg->items[i].mask << 21) >> 16) & 0xFFFF;
                    mask_l = ((filter_cfg->items[i].mask << 21) | mask_l_tail) & 0xFFFF;
                }
                else if (filter_cfg->items[i].ide == RT_CAN_EXTID)
                {
                    id_h = (filter_cfg->items[i].id >> 13) & 0xFFFF;
                    id_l = ((filter_cfg->items[i].id << 3) |
                            (filter_cfg->items[i].ide << 2) |
                            (filter_cfg->items[i].rtr << 1)) &
                           0xFFFF;
                    mask_h = ((filter_cfg->items[i].mask << 3) >> 16) & 0xFFFF;
                    mask_l = ((filter_cfg->items[i].mask << 3) | mask_l_tail) & 0xFFFF;
                }
                drv_can->can_filter_init.Filter_Scale = CAN_Filter_32bitScale;
                drv_can->can_filter_init.Filter_HighId = id_h;
                drv_can->can_filter_init.Filter_LowId = id_l;
                drv_can->can_filter_init.FilterMask_HighId = mask_h;
                drv_can->can_filter_init.FilterMask_LowId = mask_l;
                drv_can->can_filter_init.Filter_FIFOAssignment = CAN_FIFO0;
                drv_can->can_filter_init.Filter_Act = ENABLE;

                /* Filter conf */
                _can_filter_config(drv_can);
            }
        }
        break;
    }
    case RT_CAN_CMD_SET_MODE:
        argval = (rt_uint32_t)arg;
        if (argval != RT_CAN_MODE_NORMAL &&
            argval != RT_CAN_MODE_LISEN &&
            argval != RT_CAN_MODE_LOOPBACK &&
            argval != RT_CAN_MODE_LOOPBACKANLISEN)
        {
            return -RT_ERROR;
        }
        if (argval != drv_can->device.config.mode)
        {
            drv_can->device.config.mode = argval;
            return _can_config(&drv_can->device, &drv_can->device.config);
        }
        break;
    case RT_CAN_CMD_SET_BAUD:
        argval = (rt_uint32_t)arg;
        if (argval != CAN1MBaud &&
            argval != CAN800kBaud &&
            argval != CAN500kBaud &&
            argval != CAN250kBaud &&
            argval != CAN125kBaud &&
            argval != CAN100kBaud &&
            argval != CAN50kBaud &&
            argval != CAN20kBaud &&
            argval != CAN10kBaud)
        {
            return -RT_ERROR;
        }
        if (argval != drv_can->device.config.baud_rate)
        {
            drv_can->device.config.baud_rate = argval;
            return _can_config(&drv_can->device, &drv_can->device.config);
        }
        break;
    case RT_CAN_CMD_SET_PRIV:
        argval = (rt_uint32_t)arg;
        if (argval != RT_CAN_MODE_PRIV &&
            argval != RT_CAN_MODE_NOPRIV)
        {
            return -RT_ERROR;
        }
        if (argval != drv_can->device.config.privmode)
        {
            drv_can->device.config.privmode = argval;
            return _can_config(&drv_can->device, &drv_can->device.config);
        }
        break;
    case RT_CAN_CMD_GET_STATUS:
    {
        rt_uint32_t errtype;
        errtype = drv_can->CANx->ESTS;
        drv_can->device.status.rcverrcnt = errtype >> 24;
        drv_can->device.status.snderrcnt = (errtype >> 16 & 0xFF);
        drv_can->device.status.lasterrtype = errtype & 0x70;
        drv_can->device.status.errcode = errtype & 0x07;

        rt_memcpy(arg, &drv_can->device.status, sizeof(drv_can->device.status));
    }
    break;
    }

    return RT_EOK;
}

/* CAN Mailbox Transmit Request */
#define TMIDxR_TXRQ ((uint32_t)0x00000001) /* Transmit mailbox request */

static int _can_sendmsg_rtmsg(CAN_Module *CANx, struct rt_can_msg *pmsg, uint32_t mailbox_index)
{
    CanTxMessage CAN_TxMessage = {0};
    CanTxMessage *TxMessage = &CAN_TxMessage;
    /* Check the parameters */
    assert_param(IS_CAN_ALL_PERIPH(CANx));

    if (RT_CAN_STDID == pmsg->ide)
    {
        TxMessage->IDE = CAN_Standard_Id;
        RT_ASSERT(IS_CAN_STDID(pmsg->id));
        TxMessage->StdId = pmsg->id;
    }
    else
    {
        TxMessage->IDE = CAN_Extended_Id;
        RT_ASSERT(IS_CAN_EXTID(pmsg->id));
        TxMessage->ExtId = pmsg->id;
    }

    if (RT_CAN_DTR == pmsg->rtr)
    {
        TxMessage->RTR = CAN_RTRQ_DATA;
    }
    else
    {
        TxMessage->RTR = CAN_RTRQ_REMOTE;
    }

    if (mailbox_index != CAN_TxSTS_NoMailBox)
    {
        /* Set up the Id */
        CANx->sTxMailBox[mailbox_index].TMI &= TMIDxR_TXRQ;
        if (TxMessage->IDE == CAN_Standard_Id)
        {
            assert_param(IS_CAN_STDID(TxMessage->StdId));
            CANx->sTxMailBox[mailbox_index].TMI |= ((TxMessage->StdId << 21) | TxMessage->RTR);
        }
        else
        {
            assert_param(IS_CAN_EXTID(TxMessage->ExtId));
            CANx->sTxMailBox[mailbox_index].TMI |= ((TxMessage->ExtId << 3) | TxMessage->IDE | TxMessage->RTR);
        }

        /* Set up the DLC */
        TxMessage->DLC = pmsg->len & 0x0FU;
        CANx->sTxMailBox[mailbox_index].TMDT &= (uint32_t)0xFFFFFFF0;
        CANx->sTxMailBox[mailbox_index].TMDT |= TxMessage->DLC;

        /* Set up the data field */
        CANx->sTxMailBox[mailbox_index].TMDH =
            (((uint32_t)pmsg->data[7] << 24) |
             ((uint32_t)pmsg->data[6] << 16) |
             ((uint32_t)pmsg->data[5] << 8) |
             ((uint32_t)pmsg->data[4]));
        CANx->sTxMailBox[mailbox_index].TMDL =
            (((uint32_t)pmsg->data[3] << 24) |
             ((uint32_t)pmsg->data[2] << 16) |
             ((uint32_t)pmsg->data[1] << 8) |
             ((uint32_t)pmsg->data[0]));
        /* Request transmission */
        CANx->sTxMailBox[mailbox_index].TMI |= TMIDxR_TXRQ;

        return RT_EOK;
    }
    return -RT_ERROR;
}

static int _can_sendmsg(struct rt_can_device *can, const void *buf, rt_uint32_t box_num)
{
    struct n32g45x_can *drv_can;

    RT_ASSERT(can != RT_NULL);
    RT_ASSERT(buf != RT_NULL);
    drv_can = (struct n32g45x_can *)can->parent.user_data;
    RT_ASSERT(drv_can != RT_NULL);

    /* Select one empty transmit mailbox */
    switch (box_num)
    {
    case 0:
        if ((drv_can->CANx->TSTS & CAN_TSTS_TMEM0) != CAN_TSTS_TMEM0)
        {
            /* Return function status */
            return -RT_ERROR;
        }
        break;
    case 1:
        if ((drv_can->CANx->TSTS & CAN_TSTS_TMEM1) != CAN_TSTS_TMEM1)
        {
            /* Return function status */
            return -RT_ERROR;
        }
        break;
    case 2:
        if ((drv_can->CANx->TSTS & CAN_TSTS_TMEM2) != CAN_TSTS_TMEM2)
        {
            /* Return function status */
            return -RT_ERROR;
        }
        break;
    default:
        RT_ASSERT(0);
        break;
    }
    //start send msg
    return _can_sendmsg_rtmsg(drv_can->CANx, ((struct rt_can_msg *)buf), box_num);
}

static int _can_recvmsg_rtmsg(CAN_Module *CANx, struct rt_can_msg *pmsg, uint32_t FIFONum)
{
    CanRxMessage CAN_RxMessage = {0};
    CanRxMessage *RxMessage = &CAN_RxMessage;
    /* Check the parameters */
    assert_param(IS_CAN_ALL_PERIPH(CANx));
    assert_param(IS_CAN_FIFO(FIFONum));

    /* Check the Rx FIFO */
    if (FIFONum == CAN_FIFO0) /* Rx element is assigned to Rx FIFO 0 */
    {
        /* Check that the Rx FIFO 0 is not empty */
        if ((CANx->RFF0 & CAN_RFF0_FFMP0) == 0U)
        {
            return -RT_ERROR;
        }
    }
    else /* Rx element is assigned to Rx FIFO 1 */
    {
        /* Check that the Rx FIFO 1 is not empty */
        if ((CANx->RFF1 & CAN_RFF1_FFMP1) == 0U)
        {
            return -RT_ERROR;
        }
    }

    /* Get the Id */
    RxMessage->IDE = (uint8_t)0x04 & CANx->sFIFOMailBox[FIFONum].RMI;
    if (RxMessage->IDE == CAN_Standard_Id)
    {
        RxMessage->StdId = (uint32_t)0x000007FF & (CANx->sFIFOMailBox[FIFONum].RMI >> 21);
    }
    else
    {
        RxMessage->ExtId = (uint32_t)0x1FFFFFFF & (CANx->sFIFOMailBox[FIFONum].RMI >> 3);
    }
    RxMessage->RTR = (uint8_t)0x02 & CANx->sFIFOMailBox[FIFONum].RMI;
    /* Get the DLC */
    RxMessage->DLC = (uint8_t)0x0F & CANx->sFIFOMailBox[FIFONum].RMDT;
    /* Get the FMI */
    RxMessage->FMI = (uint8_t)0xFF & (CANx->sFIFOMailBox[FIFONum].RMDT >> 8);

    /* Get the data field */
    pmsg->data[0] = (uint8_t)0xFF & CANx->sFIFOMailBox[FIFONum].RMDL;
    pmsg->data[1] = (uint8_t)0xFF & (CANx->sFIFOMailBox[FIFONum].RMDL >> 8);
    pmsg->data[2] = (uint8_t)0xFF & (CANx->sFIFOMailBox[FIFONum].RMDL >> 16);
    pmsg->data[3] = (uint8_t)0xFF & (CANx->sFIFOMailBox[FIFONum].RMDL >> 24);
    pmsg->data[4] = (uint8_t)0xFF & CANx->sFIFOMailBox[FIFONum].RMDH;
    pmsg->data[5] = (uint8_t)0xFF & (CANx->sFIFOMailBox[FIFONum].RMDH >> 8);
    pmsg->data[6] = (uint8_t)0xFF & (CANx->sFIFOMailBox[FIFONum].RMDH >> 16);
    pmsg->data[7] = (uint8_t)0xFF & (CANx->sFIFOMailBox[FIFONum].RMDH >> 24);

    /* get len */
    pmsg->len = RxMessage->DLC;

    /* get id */
    if (RxMessage->IDE == CAN_Standard_Id)
    {
        pmsg->ide = RT_CAN_STDID;
        pmsg->id = RxMessage->StdId;
    }
    else
    {
        pmsg->ide = RT_CAN_EXTID;
        pmsg->id = RxMessage->ExtId;
    }
    /* get type */
    if (CAN_RTRQ_Data == RxMessage->RTR)
    {
        pmsg->rtr = RT_CAN_DTR;
    }
    else
    {
        pmsg->rtr = RT_CAN_RTR;
    }

    /* get hdr */
    if (CANx == CAN1)
    {
        pmsg->hdr = (RxMessage->FMI + 1) >> 1;
    }
#ifdef CAN2
    else if (CANx == CAN2)
    {
        pmsg->hdr = (RxMessage->FMI >> 1) + 14;
    }
#endif
    /* Release the DATFIFO */
    /* Release FIFO0 */
    if (FIFONum == CAN_FIFO0)
    {
        CANx->RFF0 |= CAN_RFF0_RFFOM0;
    }
    /* Release FIFO1 */
    else /* FIFONum == CAN_FIFO1 */
    {
        CANx->RFF1 |= CAN_RFF1_RFFOM1;
    }
    return RT_EOK;
}

static int _can_recvmsg(struct rt_can_device *can, void *buf, rt_uint32_t fifo)
{
    struct n32g45x_can *drv_can;

    RT_ASSERT(can != RT_NULL);
    RT_ASSERT(buf != RT_NULL);
    drv_can = (struct n32g45x_can *)can->parent.user_data;
    RT_ASSERT(drv_can != RT_NULL);

    /* get data */
    return _can_recvmsg_rtmsg(drv_can->CANx, ((struct rt_can_msg *)buf), fifo);
}

static const struct rt_can_ops _can_ops =
    {
        _can_config,
        _can_control,
        _can_sendmsg,
        _can_recvmsg,
};

static void _can_rx_isr(struct rt_can_device *can, rt_uint32_t fifo)
{
    struct n32g45x_can *drv_can;
    RT_ASSERT(can != RT_NULL);
    drv_can = (struct n32g45x_can *)can->parent.user_data;
    RT_ASSERT(drv_can != RT_NULL);

    switch (fifo)
    {
    case CAN_FIFO0:
        /* save to user list */
        if (CAN_GetFlagSTS(drv_can->CANx, CAN_FLAG_FFMP0) && CAN_PendingMessage(drv_can->CANx, CAN_FIFO0))
        {
            rt_hw_can_isr(can, RT_CAN_EVENT_RX_IND | fifo << 8);
        }
        /* Check FULL flag for FIFO0 */
        if (CAN_GetFlagSTS(drv_can->CANx, CAN_FLAG_FFULL0))
        {
            /* Clear FIFO0 FULL Flag */
            CAN_ClearFlag(drv_can->CANx, CAN_FLAG_FFULL0);
        }
        /* Check Overrun flag for FIFO0 */
        if (CAN_GetFlagSTS(drv_can->CANx, CAN_FLAG_FFOVR0))
        {
            /* Clear FIFO0 Overrun Flag */
            CAN_ClearFlag(drv_can->CANx, CAN_FLAG_FFOVR0);
            rt_hw_can_isr(can, RT_CAN_EVENT_RXOF_IND | fifo << 8);
        }
        break;
    case CAN_FIFO1:
        /* save to user list */
        if (CAN_GetFlagSTS(drv_can->CANx, CAN_FLAG_FFMP1) && CAN_PendingMessage(drv_can->CANx, CAN_FIFO1))
        {
            rt_hw_can_isr(can, RT_CAN_EVENT_RX_IND | fifo << 8);
        }
        /* Check FULL flag for FIFO1 */
        if (CAN_GetFlagSTS(drv_can->CANx, CAN_FLAG_FFULL1))
        {
            /* Clear FIFO1 FULL Flag */
            CAN_ClearFlag(drv_can->CANx, CAN_FLAG_FFULL1);
        }
        /* Check Overrun flag for FIFO1 */
        if (CAN_GetFlagSTS(drv_can->CANx, CAN_FLAG_FFOVR1))
        {
            /* Clear FIFO1 Overrun Flag */
            CAN_ClearFlag(drv_can->CANx, CAN_FLAG_FFOVR1);
            rt_hw_can_isr(can, RT_CAN_EVENT_RXOF_IND | fifo << 8);
        }
        break;
    }
}

#ifdef BSP_USING_CAN1
/**
 * @brief This function handles CAN1 TX interrupts. transmit fifo0/1/2 is empty can trigger this interrupt
 */
#define CAN1_TX_IRQHandler USB_HP_CAN1_TX_IRQHandler
void CAN1_TX_IRQHandler(void)
{
    rt_interrupt_enter();
    if (CAN_GetFlagSTS(drv_can1.CANx, CAN_FLAG_RQCPM0))
    {
        rt_hw_can_isr(&drv_can1.device, RT_CAN_EVENT_TX_DONE | (0x00 << 8));
        CAN_ClearFlag(drv_can1.CANx, CAN_FLAG_RQCPM0);
    }
    if (CAN_GetFlagSTS(drv_can1.CANx, CAN_FLAG_RQCPM1))
    {
        rt_hw_can_isr(&drv_can1.device, RT_CAN_EVENT_TX_DONE | (0x01 << 8));
        CAN_ClearFlag(drv_can1.CANx, CAN_FLAG_RQCPM1);
    }
    if (CAN_GetFlagSTS(drv_can1.CANx, CAN_FLAG_RQCPM2))
    {
        rt_hw_can_isr(&drv_can1.device, RT_CAN_EVENT_TX_DONE | (0x02 << 8));
        CAN_ClearFlag(drv_can1.CANx, CAN_FLAG_RQCPM2);
    }
    rt_interrupt_leave();
}

/**
 * @brief This function handles CAN1 RX0 interrupts.
 */
#define CAN1_RX0_IRQHandler USB_LP_CAN1_RX0_IRQHandler
void CAN1_RX0_IRQHandler(void)
{
    rt_interrupt_enter();
    _can_rx_isr(&drv_can1.device, CAN_FIFO0);
    rt_interrupt_leave();
}

/**
 * @brief This function handles CAN1 RX1 interrupts.
 */
void CAN1_RX1_IRQHandler(void)
{
    rt_interrupt_enter();
    _can_rx_isr(&drv_can1.device, CAN_FIFO1);
    rt_interrupt_leave();
}

/**
 * @brief This function handles CAN1 SCE interrupts.
 */
void CAN1_SCE_IRQHandler(void)
{
    uint32_t errtype;
    rt_interrupt_enter();

    if (CAN_GetIntStatus(drv_can1.CANx, CAN_INT_ERR))
    {
        errtype = drv_can1.CANx->ESTS;
        // ESTS -> LEC
        switch ((errtype & 0x70) >> 4)
        {
        case RT_CAN_BUS_BIT_PAD_ERR:
            break;
        case RT_CAN_BUS_FORMAT_ERR:
            drv_can1.device.status.formaterrcnt++;
            break;
        case RT_CAN_BUS_ACK_ERR: /* attention !!! test ack err's unit is transmit unit */
            drv_can1.device.status.ackerrcnt++;
            if (!READ_BIT(drv_can1.CANx->TSTS, CAN_TSTS_TXOKM0))
                rt_hw_can_isr(&drv_can1.device, RT_CAN_EVENT_TX_FAIL | 0 << 8);
            else if (!READ_BIT(drv_can1.CANx->TSTS, CAN_TSTS_TXOKM1))
                rt_hw_can_isr(&drv_can1.device, RT_CAN_EVENT_TX_FAIL | 1 << 8);
            else if (!READ_BIT(drv_can1.CANx->TSTS, CAN_TSTS_TXOKM2))
                rt_hw_can_isr(&drv_can1.device, RT_CAN_EVENT_TX_FAIL | 2 << 8);
            break;
        case RT_CAN_BUS_IMPLICIT_BIT_ERR:
        case RT_CAN_BUS_EXPLICIT_BIT_ERR:
            drv_can1.device.status.biterrcnt++;
            break;
        case RT_CAN_BUS_CRC_ERR:
            drv_can1.device.status.crcerrcnt++;
            break;
        }
        drv_can1.device.status.lasterrtype = errtype & 0x70;
        drv_can1.device.status.rcverrcnt = errtype >> 24;
        drv_can1.device.status.snderrcnt = (errtype >> 16 & 0xFF);
        drv_can1.device.status.errcode = errtype & 0x07;

        CAN_ClearINTPendingBit(drv_can1.CANx, CAN_INT_ERR);
    }
    rt_interrupt_leave();
}
#endif /* BSP_USING_CAN1 */

#ifdef BSP_USING_CAN2
/**
 * @brief This function handles CAN2 TX interrupts.
 */
void CAN2_TX_IRQHandler(void)
{
    rt_interrupt_enter();
    if (CAN_GetFlagSTS(drv_can2.CANx, CAN_FLAG_RQCPM0))
    {
        CAN_ClearFlag(drv_can2.CANx, CAN_FLAG_RQCPM0);
        rt_hw_can_isr(&drv_can2.device, RT_CAN_EVENT_TX_DONE | (0x00 << 8));
    }
    if (CAN_GetFlagSTS(drv_can2.CANx, CAN_FLAG_RQCPM1))
    {
        CAN_ClearFlag(drv_can2.CANx, CAN_FLAG_RQCPM1);
        rt_hw_can_isr(&drv_can2.device, RT_CAN_EVENT_TX_DONE | (0x01 << 8));
    }
    if (CAN_GetFlagSTS(drv_can2.CANx, CAN_FLAG_RQCPM2))
    {
        CAN_ClearFlag(drv_can2.CANx, CAN_FLAG_RQCPM2);
        rt_hw_can_isr(&drv_can2.device, RT_CAN_EVENT_TX_DONE | (0x02 << 8));
    }
    rt_interrupt_leave();
}

/**
 * @brief This function handles CAN2 RX0 interrupts.
 */
void CAN2_RX0_IRQHandler(void)
{
    rt_interrupt_enter();
    _can_rx_isr(&drv_can2.device, CAN_FIFO0);
    rt_interrupt_leave();
}

/**
 * @brief This function handles CAN2 RX1 interrupts.
 */
void CAN2_RX1_IRQHandler(void)
{
    rt_interrupt_enter();
    _can_rx_isr(&drv_can2.device, CAN_FIFO1);
    rt_interrupt_leave();
}

/**
 * @brief This function handles CAN2 SCE interrupts.
 */
void CAN2_SCE_IRQHandler(void)
{
    uint32_t errtype;
    rt_interrupt_enter();

    if (CAN_GetIntStatus(drv_can2.CANx, CAN_INT_ERR))
    {
        errtype = drv_can2.CANx->ESTS;
        // ESTS -> LEC
        switch ((errtype & 0x70) >> 4)
        {
        case RT_CAN_BUS_BIT_PAD_ERR:
            break;
        case RT_CAN_BUS_FORMAT_ERR:
            drv_can2.device.status.formaterrcnt++;
            break;
        case RT_CAN_BUS_ACK_ERR: /* attention !!! test ack err's unit is transmit unit */
            drv_can2.device.status.ackerrcnt++;
            if (!READ_BIT(drv_can2.CANx->TSTS, CAN_TSTS_TXOKM0))
                rt_hw_can_isr(&drv_can2.device, RT_CAN_EVENT_TX_FAIL | 0 << 8);
            else if (!READ_BIT(drv_can2.CANx->TSTS, CAN_TSTS_TXOKM1))
                rt_hw_can_isr(&drv_can2.device, RT_CAN_EVENT_TX_FAIL | 1 << 8);
            else if (!READ_BIT(drv_can2.CANx->TSTS, CAN_TSTS_TXOKM2))
                rt_hw_can_isr(&drv_can2.device, RT_CAN_EVENT_TX_FAIL | 2 << 8);
            break;
        case RT_CAN_BUS_IMPLICIT_BIT_ERR:
        case RT_CAN_BUS_EXPLICIT_BIT_ERR:
            drv_can2.device.status.biterrcnt++;
            break;
        case RT_CAN_BUS_CRC_ERR:
            drv_can2.device.status.crcerrcnt++;
            break;
        }

        drv_can2.device.status.lasterrtype = errtype & 0x70;
        drv_can2.device.status.rcverrcnt = errtype >> 24;
        drv_can2.device.status.snderrcnt = (errtype >> 16 & 0xFF);
        drv_can2.device.status.errcode = errtype & 0x07;

        CAN_ClearINTPendingBit(drv_can2.CANx, CAN_INT_ERR);
    }
    rt_interrupt_leave();
}
#endif /* BSP_USING_CAN2 */

int rt_hw_can_init(void)
{
    struct can_configure config = CANDEFAULTCONFIG;
    config.privmode = RT_CAN_MODE_NOPRIV;
    config.ticks = 50;
#ifdef RT_CAN_USING_HDR
    config.maxhdr = 14;
#ifdef CAN2
    config.maxhdr = 28;
#endif
#endif

#ifdef BSP_USING_CAN1
    /* config default filter */
    drv_can1.can_filter_init.Filter_Num = 0;
    drv_can1.can_filter_init.Filter_Mode = CAN_Filter_IdMaskMode;
    drv_can1.can_filter_init.Filter_Scale = CAN_Filter_32bitScale;
    drv_can1.can_filter_init.Filter_HighId = 0x0000;
    drv_can1.can_filter_init.Filter_LowId = 0x0000;
    drv_can1.can_filter_init.FilterMask_HighId = 0;
    drv_can1.can_filter_init.FilterMask_LowId = 0;
    drv_can1.can_filter_init.Filter_FIFOAssignment = CAN_FIFO0;
    drv_can1.can_filter_init.Filter_Act = ENABLE;
    drv_can1.device.config = config;
    /* register CAN1 device */
    rt_hw_can_register(&drv_can1.device, drv_can1.name, &_can_ops, &drv_can1);
#endif /* BSP_USING_CAN1 */

#ifdef BSP_USING_CAN2
    /* config default filter */
    drv_can2.can_filter_init.Filter_Num = 0;
    drv_can2.can_filter_init.Filter_Mode = CAN_Filter_IdMaskMode;
    drv_can2.can_filter_init.Filter_Scale = CAN_Filter_32bitScale;
    drv_can2.can_filter_init.Filter_HighId = 0x0000;
    drv_can2.can_filter_init.Filter_LowId = 0x0000;
    drv_can2.can_filter_init.FilterMask_HighId = 0;
    drv_can2.can_filter_init.FilterMask_LowId = 0;
    drv_can2.can_filter_init.Filter_FIFOAssignment = CAN_FIFO0;
    drv_can2.can_filter_init.Filter_Act = ENABLE;
    drv_can2.device.config = config;
    /* register CAN2 device */
    rt_hw_can_register(&drv_can2.device, drv_can2.name, &_can_ops, &drv_can2);
#endif /* BSP_USING_CAN2 */

    return 0;
}

INIT_BOARD_EXPORT(rt_hw_can_init);

#endif /* BSP_USING_CAN */

/************************** end of file ******************/

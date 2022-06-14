/*
 * Copyright (C) 2022, Xiaohua Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-28     CDT          first version
 */


#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

#include <rtconfig.h>
#include "hc32_ll.h"
#include "drv_config.h"


/************************ USART port **********************/
#if defined(BSP_USING_UART1)
    #define USART1_RX_PORT                  (GPIO_PORT_H)
    #define USART1_RX_PIN                   (GPIO_PIN_13)

    #define USART1_TX_PORT                  (GPIO_PORT_H)
    #define USART1_TX_PIN                   (GPIO_PIN_15)
#endif

#if defined(BSP_USING_UART6)
    #define USART6_RX_PORT                  (GPIO_PORT_H)
    #define USART6_RX_PIN                   (GPIO_PIN_06)

    #define USART6_TX_PORT                  (GPIO_PORT_E)
    #define USART6_TX_PIN                   (GPIO_PIN_06)
#endif

/*********************** PWM port *************************/
#if defined(BSP_USING_PWM1)

    #if defined(BSP_USING_PWM1_CH1)
        #define PWM1_CH1_PORT                   (GPIO_PORT_A)
        #define PWM1_CH1_PIN                    (GPIO_PIN_08)
        #define PWM1_CH1_FUNC                   (GPIO_FUNC_4)
    #endif

    #if defined(BSP_USING_PWM1_CH2)
        #define PWM1_CH2_PORT                   (GPIO_PORT_A)
        #define PWM1_CH2_PIN                    (GPIO_PIN_09)
        #define PWM1_CH2_FUNC                   (GPIO_FUNC_4)
    #endif

#endif

/****************** Pulse encoder port ********************/
#if defined(BSP_USING_PULSE_ENCODER9)
    #define PULSE_ENCODER9_CLKA_PORT        (GPIO_PORT_G)
    #define PULSE_ENCODER9_CLKA_PIN         (GPIO_PIN_04)
    #define PULSE_ENCODER9_CLKA_FUNC        (GPIO_FUNC_4)

    #define PULSE_ENCODER9_CLKB_PORT        (GPIO_PORT_G)
    #define PULSE_ENCODER9_CLKB_PIN         (GPIO_PIN_05)
    #define PULSE_ENCODER9_CLKB_FUNC        (GPIO_FUNC_4)
#endif

/***********  ADC configure *********/
#if defined(BSP_USING_ADC1)
    #define ADC1_CH_PORT                     (GPIO_PORT_C)
    #define ADC1_CH_PIN                      (GPIO_PIN_00)
#endif

#if defined(BSP_USING_ADC2)
    #define ADC2_CH_PORT                     (GPIO_PORT_C)
    #define ADC2_CH_PIN                      (GPIO_PIN_01)
#endif

#if defined(BSP_USING_ADC3)
    #define ADC3_CH_PORT                     (GPIO_PORT_C)
    #define ADC3_CH_PIN                      (GPIO_PIN_02)
#endif

/***********  CAN configure *********/
#if defined(BSP_USING_CAN1)
    #define CAN1_TX_PORT                     (GPIO_PORT_D)
    #define CAN1_TX_PIN                      (GPIO_PIN_05)
    #define CAN1_TX_PIN_FUNC                 (GPIO_FUNC_60)

    #define CAN1_RX_PORT                     (GPIO_PORT_D)
    #define CAN1_RX_PIN                      (GPIO_PIN_04)
    #define CAN1_RX_PIN_FUNC                 (GPIO_FUNC_61)

    #define CAN1_INT_PRIO                    (DDL_IRQ_PRIO_03)
    #define CAN1_INT_SRC                     (INT_SRC_CAN1_HOST)
    #define CAN1_INT_IRQn                    (INT004_IRQn)
#endif

#if defined(BSP_USING_CAN2)
    #define CAN2_TX_PORT                     (GPIO_PORT_D)
    #define CAN2_TX_PIN                      (GPIO_PIN_07)
    #define CAN2_TX_PIN_FUNC                 (GPIO_FUNC_62)

    #define CAN2_RX_PORT                     (GPIO_PORT_D)
    #define CAN2_RX_PIN                      (GPIO_PIN_06)
    #define CAN2_RX_PIN_FUNC                 (GPIO_FUNC_63)

    #define CAN2_INT_PRIO                    (DDL_IRQ_PRIO_03)
    #define CAN2_INT_SRC                     (INT_SRC_CAN2_HOST)
    #define CAN2_INT_IRQn                    (INT005_IRQn)
#endif

#endif

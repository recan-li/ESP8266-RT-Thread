/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-08-14     Jonas        first version
 */

#include <hk32f0xx.h>
#include <rtthread.h>
#include "hk32_msp.h"

#ifdef BSP_USING_UART
void hk32_msp_usart_init(void *Instance)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_TypeDef *USARTx = (USART_TypeDef *)Instance;

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
#ifdef BSP_USING_UART1
    if (USART1 == USARTx)
    {
#define USART1_REMAP
#ifndef USART1_REMAP
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
        GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
        GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

#else
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
        GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
        GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_0);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_0);
#endif
    }
#endif
#ifdef BSP_USING_UART2
    if (USART2 == USARTx)
    {
        RCC_APB2PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
        GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
        GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
    }
#endif
    /* Add others */
}
#endif /* BSP_USING_SERIAL */
#ifdef BSP_USING_I2C
void hk32_msp_i2c_init(void *Instance)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    I2C_TypeDef *I2Cx = (I2C_TypeDef *)Instance;

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
#ifdef BSP_USING_I2C1
    if (I2C1 == I2Cx)
    {
#ifndef I2C1_REMAP
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
        GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
        GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_1);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_1);
#else
        RCC_APB2PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
        GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
        GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_1);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_1);
#endif
    }
#endif
    /* Add others */
}
#endif /* BSP_USING_I2C */


#ifdef BSP_USING_SPI
void hk32_msp_spi_init(void *Instance)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    SPI_TypeDef *SPIx = (SPI_TypeDef *)Instance;

    GPIO_StructInit(&GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
#ifdef BSP_USING_SPI1
    if (SPI1 == SPIx)
    {
#ifndef SPI1_REMAP
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
        GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
        GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_0);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_0);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_0);
#else
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
        GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
        GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_0);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_0);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_0);
#endif
    }
#endif
    /* Add others */
}
#endif /* BSP_USING_I2C */

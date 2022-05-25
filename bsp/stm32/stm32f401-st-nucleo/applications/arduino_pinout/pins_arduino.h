/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-19     Meco Man     first version
 */

#ifndef Pins_Arduino_h
#define Pins_Arduino_h

/* pins alias. Must keep in sequence */
#define D0   (0)
#define D1   (1)
#define D2   (2)
#define D3   (3)
#define D4   (4)
#define D5   (5)
#define D6   (6)
#define D7   (7)
#define D8   (8)
#define D9   (9)
#define D10  (10)
#define D11  (11)
#define D12  (12)
#define D13  (13)
#define D14  (14)
#define D15  (15)
#define D16  (16)
#define A0   (17)
#define A1   (18)
#define A2   (19)
#define A3   (20)
#define A4   (21)
#define A5   (22)
#define A6   (23)
#define A7   (24)

#define LED_BUILTIN  D13 /* Default Built-in LED */

#define RTDUINO_DEFAULT_IIC_BUS_NAME            "i2c1"
#define RTDUINO_DEFAULT_SPI_BUS_NAME            "spi1"
#define RTDUINO_DEFAULT_HWTIMER_DEVICE_NAME     "timer11"

#endif /* Pins_Arduino_h */

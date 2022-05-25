/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-04-19     Meco Man     first version
 */
#include <Arduino.h>
#include <board.h>
#include "pins_arduino.h"

/*
    {Arduino Pin, RT-Thread Pin [, Device Name(PWM or ADC), Channel]}
    [] means optional
    Digital pins must NOT give the device name and channel.
    Analog pins MUST give the device name and channel(ADC, PWM or DAC).
    Arduino Pin must keep in sequence.
*/
const pin_map_t pin_map_table[]=
{
    {D0}, /* RX */
    {D1}, /* TX */
    {D2, GET_PIN(A,10)},
    {D3, GET_PIN(B,3), "pwm2", 2}, /* PWM */
    {D4, GET_PIN(B,5)},
    {D5, GET_PIN(B,4), "pwm3", 1}, /* PWM */
    {D6, GET_PIN(B,10), "pwm2", 3}, /* PWM */
    {D7, GET_PIN(A,8)},
    {D8, GET_PIN(A,9)},
    {D9, GET_PIN(C,7), "pwm3", 2}, /* PWM */
    {D10}, /* SPI1-CS */
    {D11}, /* SPI1-MOSI */
    {D12}, /* SPI1-MISO */
    {D13}, /* SPI1-SCK */
    {D14}, /* I2C1-SDA */
    {D15}, /* I2C1-SCL */
    {D16, GET_PIN(C,13)}, /* user button */
    {A0, GET_PIN(A,0), "adc1", 0}, /* ADC */
    {A1, GET_PIN(A,1), "adc1", 1}, /* ADC */
    {A2, GET_PIN(A,4), "adc1", 4}, /* ADC */
    {A3, GET_PIN(B,0), "adc1", 8}, /* ADC */
    {A4, GET_PIN(C,1), "adc1", 11}, /* ADC */
    {A5, GET_PIN(C,0), "adc1", 10}, /* ADC */
    {A6, RT_NULL, "adc1", 17}, /* ADC, On-Chip: internal reference voltage, ADC_CHANNEL_VREFINT */
    {A7, RT_NULL, "adc1", 16} /* ADC, On-Chip: internal temperature sensor, ADC_CHANNEL_TEMPSENSOR */
};

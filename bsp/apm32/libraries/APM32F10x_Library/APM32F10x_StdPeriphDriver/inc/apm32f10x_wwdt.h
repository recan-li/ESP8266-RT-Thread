/*!
 * @file        apm32f10x_wwdt.h
 *
 * @brief       This file contains all the functions prototypes for the WWDT firmware library
 *
 * @version     V1.0.2
 *
 * @date        2022-01-05
 *
 * @attention
 *
 *  Copyright (C) 2020-2022 Geehy Semiconductor
 *
 *  You may not use this file except in compliance with the
 *  GEEHY COPYRIGHT NOTICE (GEEHY SOFTWARE PACKAGE LICENSE).
 *
 *  The program is only for reference, which is distributed in the hope
 *  that it will be usefull and instructional for customers to develop
 *  their software. Unless required by applicable law or agreed to in
 *  writing, the program is distributed on an "AS IS" BASIS, WITHOUT
 *  ANY WARRANTY OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the GEEHY SOFTWARE PACKAGE LICENSE for the governing permissions
 *  and limitations under the License.
 */

#ifndef __APM32F10X_WWDT_H
#define __APM32F10X_WWDT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "apm32f10x.h"

/** @addtogroup Peripherals_Library Standard Peripheral Library
  @{
*/

/** @addtogroup WWDT_Driver  WWDT Driver
  @{
*/

/** @addtogroup  WWDT_Enumerations Enumerations
  @{
*/

/**
 * @brief    WWDT Timebase(Prescaler) define
 */
typedef enum
{
    WWDT_TIME_BASE_1 = 0x00000000,
    WWDT_TIME_BASE_2 = 0x00000080,
    WWDT_TIME_BASE_4 = 0x00000100,
    WWDT_TIME_BASE_8 = 0x00000180
} WWDT_TIME_BASE_T;

/**@} end of group WWDT_Enumerations*/

/** @addtogroup  WWDT_Fuctions Fuctions
  @{
*/

/** WWDT reset */
void WWDT_Reset(void);

/** Config WWDT Timebase */
void WWDT_ConfigTimebase(WWDT_TIME_BASE_T timeBase);

/** Config Window Data */
void WWDT_ConfigWindowData(uint8_t windowData);

/** Config Couter */
void WWDT_ConfigCounter(uint8_t counter);

/** Enable WWDT and Early Wakeup interrupt */
void WWDT_EnableEWI(void);
void WWDT_Enable(uint8_t count);

/** Read Flag and Clear Flag */
uint8_t WWDT_ReadFlag(void);
void WWDT_ClearFlag(void);

/**@} end of group WWDT_Fuctions*/
/**@} end of group WWDT_Driver*/
/**@} end of group Peripherals_Library*/

#ifdef __cplusplus
}
#endif

#endif /* __APM32F10X_WWDT_H */

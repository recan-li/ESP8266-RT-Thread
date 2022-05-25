/*!
 * @file        apm32f10x_iwdt.h
 *
 * @brief       This file contains all the functions prototypes for the IWDT firmware library
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

#ifndef __APM32F10X_IWDT_H
#define __APM32F10X_IWDT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "apm32f10x.h"

/** @addtogroup Peripherals_Library Standard Peripheral Library
  @{
*/

/** @addtogroup IWDT_Driver IWDT Driver
  @{
*/

/** @addtogroup IWDT_Enumerations Enumerations
  @{
*/

/**
 * @brief    IWDT KEYWORD define
 */
typedef enum
{
    IWDT_KEYWORD_RELOAD = 0xAAAA,
    IWDT_KEYWORD_ENABLE = 0xCCCC
} IWDT_KEYWORD_T;

/**
 * @brief    IWDT Write Access define
 */
typedef enum
{
    IWDT_WRITEACCESS_ENABLE  = 0x5555,
    IWDT_WRITEACCESS_DISABLE = 0x0000
} IWDT_WRITEACCESS_T;

/**
 * @brief    IWDT Divider
 */
typedef enum
{
    IWDT_DIVIDER_4   = 0x00,
    IWDT_DIVIDER_8   = 0x01,
    IWDT_DIVIDER_16  = 0x02,
    IWDT_DIVIDER_32  = 0x03,
    IWDT_DIVIDER_64  = 0x04,
    IWDT_DIVIDER_128 = 0x05,
    IWDT_DIVIDER_256 = 0x06
} IWDT_DIVIDER_T;

/**
 * @brief    IWDT Flag
 */
typedef enum
{
    IWDT_FLAG_PSCU = BIT0,
    IWDT_FLAG_CNTU = BIT1
} IWDT_FLAG_T;

/**@} end of group IWDT_Enumerations*/


/** @addtogroup IWDT_Fuctions Fuctions
  @{
*/

/** Enable IWDT */
void IWDT_Enable(void);

/** Refresh IWDT */
void IWDT_Refresh(void);

/** Counter reload */
void IWDT_ConfigReload(uint16_t reload);

/** Divider */
void IWDT_ConfigDivider(uint8_t div);

/** Write Access */
void IWDT_EnableWriteAccess(void);
void IWDT_DisableWriteAccess(void);

/** flag */
uint8_t IWDT_ReadStatusFlag(uint16_t flag);

/**@} end of group IWDT_Fuctions*/
/**@} end of group IWDT_Driver*/
/**@} end of group Peripherals_Library*/

#ifdef __cplusplus
}
#endif

#endif /* __APM32F10X_IWDT_H */

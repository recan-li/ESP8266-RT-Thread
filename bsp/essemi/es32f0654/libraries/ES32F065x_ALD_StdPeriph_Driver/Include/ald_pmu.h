/**
  *********************************************************************************
  *
  * @file    ald_pmu.h
  * @brief   Header file of PMU module driver.
  *
  * @version V1.0
  * @date    04 Dec 2017
  * @author  AE Team
  * @note
  *
  * Copyright (C) Shanghai Eastsoft Microelectronics Co. Ltd. All rights reserved.
  *
  * SPDX-License-Identifier: Apache-2.0
  *
  * Licensed under the Apache License, Version 2.0 (the License); you may
  * not use this file except in compliance with the License.
  * You may obtain a copy of the License at
  *
  * www.apache.org/licenses/LICENSE-2.0
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an AS IS BASIS, WITHOUT
  * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ********************************************************************************
  */

#ifndef __ALD_PMU_H__
#define __ALD_PMU_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "utils.h"
#include "ald_syscfg.h"
#include "ald_bkpc.h"


/** @addtogroup ES32FXXX_ALD
  * @{
  */

/** @addtogroup PMU
  * @{
  */

/** @defgroup PMU_Public_Macros PMU Public Macros
  * @{
  */
#define PMU_SRAM0_ENABLE()				\
do {							\
	SYSCFG_UNLOCK();				\
	SET_BIT(PMU->PWRCR, BIT(PMU_PWRCR_SRAM_POSS));	\
	SYSCFG_LOCK();					\
} while (0)
#define PMU_SRAM0_DISABLE()				\
do {							\
	SYSCFG_UNLOCK();				\
	CLEAR_BIT(PMU->PWRCR, BIT(PMU_PWRCR_SRAM_POSS));\
	SYSCFG_LOCK();					\
} while (0)
#define PMU_SRAM1_ENABLE()				\
do {							\
	SYSCFG_UNLOCK();				\
	SET_BIT(PMU->PWRCR, BIT(PMU_PWRCR_SRAM_POSE));	\
	SYSCFG_LOCK();					\
} while (0)
#define PMU_SRAM1_DISABLE()				\
do {							\
	SYSCFG_UNLOCK();				\
	CLEAR_BIT(PMU->PWRCR, BIT(PMU_PWRCR_SRAM_POSE));\
	SYSCFG_LOCK();					\
} while (0)
#define PMU_BXCAN_ENABLE()				\
do {							\
	SYSCFG_UNLOCK();				\
	SET_BIT(PMU->PWRCR, PMU_PWRCR_BXCAN_MSK);	\
	SYSCFG_LOCK();					\
} while (0)
#define PMU_BXCAN_DISABLE()				\
do {							\
	SYSCFG_UNLOCK();				\
	CLEAR_BIT(PMU->PWRCR, PMU_PWRCR_BXCAN_MSK);	\
	SYSCFG_LOCK();					\
} while (0)

#define PMU_GET_LVD_STATUS()	(READ_BITS(PMU->LVDCR, PMU_LVDCR_LVDO_MSK, PMU_LVDCR_LVDO_POS))
/**
  * @}
  */


/** @defgroup PMU_Public_Types PMU Public Types
  * @{
  */
/**
  * @brief Low power mode
  */
typedef enum {
	PMU_LP_STOP1   = 0x0U,	/**< Stop1 */
	PMU_LP_STOP2   = 0x1U,	/**< Stop2 */
	PMU_LP_STANDBY = 0x2U,	/**< Standby */
} pmu_lp_mode_t;

typedef enum {
	PMU_SR_WUF     = (1U << 0),
	PMU_SR_STANDBY = (1U << 1),
} pmu_status_t;

/**
  * @brief LVD voltage select
  */
typedef enum {
	PMU_LVD_VOL_SEL_2_0 = 0x0U,	/**< 2.0V ~ 2.05V */
	PMU_LVD_VOL_SEL_2_1 = 0x1U,	/**< 2.1V ~ 2.15V */
	PMU_LVD_VOL_SEL_2_2 = 0x2U,	/**< 2.2V ~ 2.25V */
	PMU_LVD_VOL_SEL_2_4 = 0x3U,	/**< 2.4V ~ 2.45V */
	PMU_LVD_VOL_SEL_2_6 = 0x4U,	/**< 2.6V ~ 2.65V */
	PMU_LVD_VOL_SEL_2_8 = 0x5U,	/**< 2.8V ~ 2.85V */
	PMU_LVD_VOL_SEL_3_0 = 0x6U,	/**< 3.0V ~ 3.05V */
	PMU_LVD_VOL_SEL_3_6 = 0x7U,	/**< 3.6V ~ 3.65V */
	PMU_LVD_VOL_SEL_4_0 = 0x8U,	/**< 4.0V ~ 4.05V */
	PMU_LVD_VOL_SEL_4_6 = 0x9U,	/**< 4.6V ~ 4.65V */
	PMU_LVD_VOL_SEL_2_3 = 0xAU,	/**< 2.3V ~ 2.35V */
	PMU_LVD_VOL_SEL_EXT = 0xFU,	/**< Select external input. It must be 1.2V */
} pmu_lvd_voltage_sel_t;

/**
  * @brief LVD trigger mode
  */
typedef enum {
	PMU_LVD_TRIGGER_RISING_EDGE    = 0x0U,	/**< Rising edge */
	PMU_LVD_TRIGGER_FALLING_EDGE   = 0x1U,	/**< Falling edge */
	PMU_LVD_TRIGGER_HIGH_LEVEL     = 0x2U,	/**< High level */
	PMU_LVD_TRIGGER_LOW_LEVEL      = 0x3U,	/**< Low level */
	PMU_LVD_TRIGGER_RISING_FALLING = 0x4U,	/**< Rising and falling edge */
} pmu_lvd_trigger_mode_t;

/**
  * @}
  */

/**
  * @defgroup PMU_Private_Macros PMU Private Macros
  * @{
  */
#define IS_PMU_LP_MODE(x)		(((x) == PMU_LP_STOP1) || \
                                         ((x) == PMU_LP_STOP2) || \
                                         ((x) == PMU_LP_STANDBY))
#define IS_PMU_STATUS(x)		(((x) == PMU_SR_WUF) || ((x) == PMU_SR_STANDBY))
#define IS_PMU_LVD_VOL_SEL(x)		(((x) == PMU_LVD_VOL_SEL_2_0) || \
                                         ((x) == PMU_LVD_VOL_SEL_2_1) || \
                                         ((x) == PMU_LVD_VOL_SEL_2_2) || \
                                         ((x) == PMU_LVD_VOL_SEL_2_4) || \
                                         ((x) == PMU_LVD_VOL_SEL_2_6) || \
                                         ((x) == PMU_LVD_VOL_SEL_2_8) || \
                                         ((x) == PMU_LVD_VOL_SEL_3_0) || \
                                         ((x) == PMU_LVD_VOL_SEL_3_6) || \
                                         ((x) == PMU_LVD_VOL_SEL_4_0) || \
                                         ((x) == PMU_LVD_VOL_SEL_4_6) || \
                                         ((x) == PMU_LVD_VOL_SEL_2_3) || \
                                         ((x) == PMU_LVD_VOL_SEL_EXT))
#define IS_PMU_LVD_TRIGGER_MODE(x)	(((x) == PMU_LVD_TRIGGER_RISING_EDGE)  || \
                                         ((x) == PMU_LVD_TRIGGER_FALLING_EDGE) || \
                                         ((x) == PMU_LVD_TRIGGER_HIGH_LEVEL)   || \
                                         ((x) == PMU_LVD_TRIGGER_LOW_LEVEL)    || \
                                         ((x) == PMU_LVD_TRIGGER_RISING_FALLING))
/**
  * @}
  */

/** @addtogroup PMU_Public_Functions
  * @{
  */
/** @addtogroup PMU_Public_Functions_Group1
  * @{
  */
/* Low power mode select */
__STATIC_INLINE__ void ald_pmu_sleep()
{
	__WFI();
}

__STATIC_INLINE__ void ald_pmu_sleep_deep()
{
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__WFI();
}

void ald_pmu_stop1_enter(void);
void ald_pmu_stop2_enter(void);
void ald_pmu_standby_enter(bkpc_wakeup_port_t port, bkpc_wakeup_level_t level);
flag_status_t ald_pmu_get_status(pmu_status_t sr);
void ald_pmu_clear_status(pmu_status_t sr);
/**
  * @}
  */
/** @addtogroup PMU_Public_Functions_Group2
  * @{
  */
/* LVD configure */
void ald_pmu_lvd_config(pmu_lvd_voltage_sel_t sel, pmu_lvd_trigger_mode_t mode, type_func_t state);
void ald_lvd_irq_handler(void);
/**
  * @}
  */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#ifdef __cplusplus
}
#endif

#endif /* __ALD_PMU_H__ */

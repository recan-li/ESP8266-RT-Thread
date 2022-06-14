/**
  *********************************************************************************
  *
  * @file    ald_rmu.c
  * @brief   RMU module driver.
  *
  * @version V1.0
  * @date    04 Dec 2019
  * @author  AE Team
  * @note
  *          Change Logs:
  *          Date            Author          Notes
  *          04 Dec 2019     AE Team         The first version
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
  **********************************************************************************
  */

#include "ald_conf.h"

/** @addtogroup ES32FXXX_ALD
  * @{
  */

/** @defgroup RMU RMU
  * @brief RMU module driver
  * @{
  */
#ifdef ALD_RMU

/** @defgroup RMU_Public_Functions RMU Public Functions
  * @{
  */

/**
  * @brief  Configure BOR parameters.
  * @param  flt: filter time.
  * @param  vol: The voltage.
  * @param  state: The new status: ENABLE/DISABLE.
  * @retval None
  */
void ald_rmu_bor_config(rmu_bor_filter_t flt, rmu_bor_vol_t vol, type_func_t state)
{
	assert_param(IS_FUNC_STATE(state));

	SYSCFG_UNLOCK();

	if (state) {
		assert_param(IS_RMU_BORFLT(flt));
		assert_param(IS_RMU_BORVOL(vol));

		MODIFY_REG(RMU->CR, RMU_CR_BORFLT_MSK, flt << RMU_CR_BORFLT_POSS);
		MODIFY_REG(RMU->CR, RMU_CR_BORVS_MSK, vol << RMU_CR_BORVS_POSS);
		SET_BIT(RMU->CR, RMU_CR_BOREN_MSK);
	}
	else {
		CLEAR_BIT(RMU->CR, RMU_CR_BOREN_MSK);
	}

	SYSCFG_LOCK();
	return;
}

/**
  * @brief  Get specified reset status
  * @param  state: Speicifies the type of the reset,
  * @retval The status.
  */
uint32_t ald_rmu_get_reset_status(rmu_state_t state)
{
	assert_param(IS_RMU_STATE(state));

	if (state == RMU_RST_ALL)
		return RMU->RSTSR;

	if (READ_BIT(RMU->RSTSR, state))
		return SET;

	return RESET;
}

/**
  * @brief  Clear the specified reset status
  * @param  state: Specifies the type of the reset,
  * @retval None
  */
void ald_rmu_clear_reset_status(rmu_state_t state)
{
	assert_param(IS_RMU_STATE_CLEAR(state));

	SYSCFG_UNLOCK();
	WRITE_REG(RMU->CRSTSR, state);
	SYSCFG_LOCK();

	return;
}
/**
  * @brief  Reset peripheral device
  * @param  perh: The peripheral device,
  * @retval None
  */
void ald_rmu_reset_periperal(rmu_peripheral_t perh)
{
	uint32_t idx, pos;

	assert_param(IS_RMU_PERH(perh));

	idx = ((uint32_t)perh >> 27) & 0x7;
	pos = perh & ~(0x7 << 27);
	SYSCFG_UNLOCK();

	switch (idx) {
	case 0:
		WRITE_REG(RMU->AHB1RSTR, pos);
		break;

	case 1:
		WRITE_REG(RMU->AHB2RSTR, pos);
		break;

	case 2:
		WRITE_REG(RMU->APB1RSTR, pos);
		break;

	case 4:
		WRITE_REG(RMU->APB2RSTR, pos);
		break;

	default:
		break;
	}

	SYSCFG_LOCK();
	return;
}

/**
  * @}
  */
#endif /* ALD_RMU */
/**
  * @}
  */

/**
  * @}
  */

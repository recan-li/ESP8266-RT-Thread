/**
  *******************************************************************************************************
  * @file    fm33lc0xx_fl_crc.c
  * @author  FMSH Application Team
  * @brief   Src file of CRC FL Module
  *******************************************************************************************************
  * @attention
  *
  * Copyright (c) [2019] [Fudan Microelectronics]
  * THIS SOFTWARE is licensed under the Mulan PSL v1.
  * can use this software according to the terms and conditions of the Mulan PSL v1.
  * You may obtain a copy of Mulan PSL v1 at:
  * http://license.coscl.org.cn/MulanPSL
  * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND, EITHER EXPRESS OR
  * IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT, MERCHANTABILITY OR FIT FOR A PARTICULAR
  * PURPOSE.
  * See the Mulan PSL v1 for more details.
  *
  *******************************************************************************************************
  */
#include "fm33lc0xx_fl_rcc.h"
#include "fm33lc0xx_fl_crc.h"
#include "fm33_assert.h"

/** @addtogroup FM33LC0XX_FL_Driver
  * @{
  */

/** @addtogroup CRC
  * @{
  */

/* Private macros ------------------------------------------------------------*/
/** @addtogroup CRC_FL_Private_Macros
  * @{
  */

#define         IS_FL_CRC_INSTANCE(INTANCE)                         ((INTANCE) == CRC)

#define         IS_FL_CRC_POLYNOMIAL_WIDTH(__VALUE__)               (((__VALUE__) == FL_CRC_POLYNOMIAL_16B)||\
                                                                     ((__VALUE__) == FL_CRC_POLYNOMIAL_32B)||\
                                                                     ((__VALUE__) == FL_CRC_POLYNOMIAL_8B)||\
                                                                     ((__VALUE__) == FL_CRC_POLYNOMIAL_7B))

#define         IS_FL_CRC_DR_WIDTH(__VALUE__)                       (((__VALUE__) == FL_CRC_DATA_WIDTH_8B)||\
                                                                    ((__VALUE__) == FL_CRC_DATA_WIDTH_32B))


#define         IS_FL_CRC_OUPUT_REFLECTE_MODE(__VALUE__)            (((__VALUE__) == FL_CRC_OUPUT_INVERT_NONE)||\
                                                                    ((__VALUE__) == FL_CRC_OUPUT_INVERT_BYTE))

#define         IS_FL_CRC_INPUT_REFLECTE_MODE(__VALUE__)            (((__VALUE__) == FL_CRC_INPUT_INVERT_NONE)||\
                                                                    ((__VALUE__) == FL_CRC_INPUT_INVERT_BYTE)||\
                                                                    ((__VALUE__) == FL_CRC_INPUT_INVERT_HALF_WORD)||\
                                                                    ((__VALUE__) == FL_CRC_INPUT_INVERT_WORD))

#define         IS_FL_CRC_CALCULA_MODE(__VALUE__)                   (((__VALUE__) == FL_CRC_CALCULATE_SERIAL)||\
                                                                    ((__VALUE__) == FL_CRC_CALCULATE_PARALLEL))

/**
  * @}
  */

/** @addtogroup CRC_FL_EF_Init
  * @{
  */

/**
  * @brief  复位CRC外设
  * @param  CRCx 外设入口地址
  * @retval 错误状态，可能值：
  *         -FL_PASS 外设寄存器值恢复复位值
  *         -FL_FAIL 未成功执行
  */
FL_ErrorStatus FL_CRC_DeInit(CRC_Type *CRCx)
{
    assert_param(IS_FL_CRC_INSTANCE(CRCx));
    /* 外设复位使能 */
    FL_RCC_EnablePeripheralReset();
    /* 复位外设寄存器 */
    FL_RCC_EnableResetAPB2Peripheral(FL_RCC_RSTAPB_CRC);
    FL_RCC_DisableResetAPB2Peripheral(FL_RCC_RSTAPB_CRC);
    /* 关闭总线时钟 */
    FL_RCC_DisableGroup2BusClock(FL_RCC_GROUP2_BUSCLK_CRC);
    /* 锁定外设复位功能 */
    FL_RCC_DisablePeripheralReset();
    return FL_PASS;
}

/**
  * @brief  根据 CRC_InitStruct 的配置信息初始化对应外设入口地址的寄存器值.
  *
  * @param  CRCx 外设入口地址
  * @param  CRC_InitStruct 指向一个 @ref FL_CRC_InitTypeDef 结构体其中包含了外设的相关配置信息.
  *
  * @retval 错误状态，可能值：
  *         -FL_PASS 配置成功
  *         -FL_FAIL 配置过程发生错误
  */
FL_ErrorStatus FL_CRC_Init(CRC_Type *CRCx, FL_CRC_InitTypeDef *CRC_InitStruct)
{
    /* 参数检查 */
    assert_param(IS_FL_CRC_INSTANCE(CRCx));
    assert_param(IS_FL_CRC_DR_WIDTH(CRC_InitStruct->dataWidth));
    assert_param(IS_FL_CRC_CALCULA_MODE(CRC_InitStruct->calculatMode));
    assert_param(IS_FL_CRC_POLYNOMIAL_WIDTH(CRC_InitStruct->polynomialWidth));
    assert_param(IS_FL_CRC_INPUT_REFLECTE_MODE(CRC_InitStruct->reflectIn));
    assert_param(IS_FL_CRC_OUPUT_REFLECTE_MODE(CRC_InitStruct->reflectOut));
    FL_RCC_EnableGroup2BusClock(FL_RCC_GROUP2_BUSCLK_CRC);
    FL_CRC_SetCalculateMode(CRCx, CRC_InitStruct->calculatMode);
    FL_CRC_SetInputInvertMode(CRCx, CRC_InitStruct->reflectIn);
    FL_CRC_SetOutputInvertMode(CRCx, CRC_InitStruct->reflectOut);
    FL_CRC_SetPolynomialWidth(CRCx, CRC_InitStruct->polynomialWidth);
    FL_CRC_WriteXORValue(CRCx, CRC_InitStruct->xorReg);
    FL_CRC_WritePolynominalParam(CRCx, CRC_InitStruct->polynomial);
    FL_CRC_WriteInitialValue(CRCx, CRC_InitStruct->initVal);
    FL_CRC_SetDataWidth(CRCx, CRC_InitStruct->dataWidth);
    if(CRC_InitStruct->xorRegState == FL_ENABLE)
    {
        FL_CRC_EnableOutputXOR(CRCx);
    }
    else
    {
        FL_CRC_DisableOutputXOR(CRCx);
    }
    return FL_PASS;
}


/**
  * @brief  将 @ref FL_CRC_InitTypeDef 结构体初始化为默认配置
  * @param  CRC_InitStruct 指向 @ref FL_CRC_InitTypeDef 结构体的指针
  *
  * @retval None
  */
void FL_CRC_StructInit(FL_CRC_InitTypeDef *CRC_InitStruct)
{
    CRC_InitStruct->polynomial      =  0x00000000;
    CRC_InitStruct->polynomialWidth =  FL_CRC_POLYNOMIAL_16B;
    CRC_InitStruct->dataWidth       =  FL_CRC_DATA_WIDTH_8B;
    CRC_InitStruct->calculatMode    =  FL_CRC_CALCULATE_SERIAL;
    CRC_InitStruct->reflectIn       =  FL_CRC_INPUT_INVERT_NONE;
    CRC_InitStruct->reflectOut      =  FL_CRC_OUPUT_INVERT_NONE;
    CRC_InitStruct->xorReg          =  0x00000000;
    CRC_InitStruct->xorRegState     =  FL_DISABLE;
}


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************************************* END OF FILE *******************************************/

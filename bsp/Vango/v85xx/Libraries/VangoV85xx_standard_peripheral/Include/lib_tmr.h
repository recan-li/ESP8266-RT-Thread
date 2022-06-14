/**
  ******************************************************************************
  * @file    lib_tmr.h 
  * @author  Application Team
  * @version V4.4.0
  * @date    2018-09-27
  * @brief   Timer library.
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#ifndef __LIB_TMR_H
#define __LIB_TMR_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "target.h"
   
typedef struct
{
  uint32_t Period;
  uint32_t ClockSource;
  uint32_t EXTGT;
} TMR_InitType;
//ClockSource
#define TMR_CLKSRC_INTERNAL     0
#define TMR_CLKSRC_EXTERNAL     TMR_CTRL_EXTCLK
//ClockGate
#define TMR_EXTGT_DISABLE       0
#define TMR_EXTGT_ENABLE        TMR_CTRL_EXTEN
                   
/* Private macros ------------------------------------------------------------*/
#define  IS_TMR_CLKSRC(__CLKSRC__)  (((__CLKSRC__) == TMR_CLKSRC_INTERNAL) || ((__CLKSRC__) == TMR_CLKSRC_EXTERNAL))

#define  IS_TMR_EXTGT(__EXTGT__)  (((__EXTGT__) == TMR_EXTGT_DISABLE) || ((__EXTGT__) == TMR_EXTGT_ENABLE))


/* Exported Functions ------------------------------------------------------- */
/* Timer Exported Functions Group1: 
                                    (De)Initialization  ----------------------*/
void TMR_DeInit(TMR_TypeDef *TMRx);
void TMR_Init(TMR_TypeDef *TMRx, TMR_InitType *InitStruct);
void TMR_StructInit(TMR_InitType *InitStruct);
/* Timer Exported Functions Group2: 
                                    Interrupt (flag) -------------------------*/
void TMR_INTConfig(TMR_TypeDef *TMRx, uint32_t NewState);
uint8_t TMR_GetINTStatus(TMR_TypeDef *TMRx);
void TMR_ClearINTStatus(TMR_TypeDef *TMRx);
/* Timer Exported Functions Group3: 
                                    MISC Configuration -----------------------*/
void TMR_Cmd(TMR_TypeDef *TMRx, uint32_t NewState);
uint32_t TMR_GetCurrentValue(TMR_TypeDef *TMRx);
                           
#ifdef __cplusplus
}
#endif
     
#endif  /* __LIB_TMR_H */

/*********************************** END OF FILE ******************************/

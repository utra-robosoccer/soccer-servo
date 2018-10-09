/**
  ******************************************************************************
  * @file    sto_speed_pos_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          State Observer + PLL Speed & Position Feedback component of the Motor
  *          Control SDK.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  * @ingroup SpeednPosFdbk_STO
  */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STO_SPEEDNPOSFDBK_H
#define __STO_SPEEDNPOSFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */

/** @addtogroup SpeednPosFdbk_STO
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @brief PWM & Current Sensing component handle type */
typedef struct STO_Handle STO_Handle_t;

typedef void ( *STO_ForceConvergency1_Cb_t )( STO_Handle_t * pHandle );
typedef void ( *STO_ForceConvergency2_Cb_t )( STO_Handle_t * pHandle );
typedef void ( *STO_OtfResetPLL_Cb_t )( STO_Handle_t * pHandle );
typedef bool ( *STO_SpeedReliabilityCheck_Cb_t )( STO_Handle_t * pHandle );

/**
  * @brief  SpeednPosFdbk  handle definition
  */
struct STO_Handle
{
  SpeednPosFdbk_Handle_t     *     _Super;
  STO_ForceConvergency1_Cb_t       pFctForceConvergency1;
  STO_ForceConvergency2_Cb_t       pFctForceConvergency2;
  STO_OtfResetPLL_Cb_t             pFctStoOtfResetPLL;
  STO_SpeedReliabilityCheck_Cb_t   pFctSTO_SpeedReliabilityCheck;
};

/**
  * @}
  */

/**
  * @}
  */

/** @} */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*__STO_SPEEDNPOSFDBK_H*/

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/

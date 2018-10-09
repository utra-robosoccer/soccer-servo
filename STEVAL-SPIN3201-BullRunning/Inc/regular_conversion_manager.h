
/**
  ******************************************************************************
  * @file    regular_conversion_manager.h.ftl
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          regular_conversion_manager component of the Motor Control SDK.
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
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __regular_conversion_manager_h
#define __regular_conversion_manager_h

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"
#include "mc_stm_types.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup RCM
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief RegConv_t contains all the parameters required to execute a regular conversion
  *
  * it is used by all regular_conversion_manager's client
  *
  */
typedef struct 
{
  ADC_TypeDef * regADC;
  uint8_t  channel;
  uint32_t samplingTime;
} RegConv_t;

typedef enum
{
  RCM_USERCONV_IDLE,
  RCM_USERCONV_REQUESTED,
  RCM_USERCONV_EOC  
}RCM_UserConvState_t;

typedef void (*RCM_exec_cb_t)(uint8_t handle, uint16_t data, void *UserData); 

/* Exported functions ------------------------------------------------------- */

/*  Function used to register a regular conversion */
uint8_t RCM_RegisterRegConv(RegConv_t * regConv);

/*  Function used to register a regular conversion with a callback attached*/
uint8_t RCM_RegisterRegConv_WithCB (RegConv_t * regConv, RCM_exec_cb_t fctCB, void *data);

/*  Function used to execute an already registered regular conversion */
uint16_t RCM_ExecRegularConv (uint8_t handle);

/* select the handle conversion to be executed during the next call to RCM_ExecUserConv */
bool RCM_RequestUserConv(uint8_t handle);

/* return the latest user conversion value*/
uint16_t RCM_GetUserConv(void);

/* Must be called by MC_TASK only to grantee proper scheduling*/
void RCM_ExecUserConv (void);

/* return the state of the user conversion state machine*/
RCM_UserConvState_t RCM_GetUserConvState(void);

/* Function used to un-schedule a regular conversion exectuted after current sampling in HF task */
bool RCM_PauseRegularConv (uint8_t handle);

/* non blocking function to start conversion inside HF task */
void RCM_ExecNextConv (void);

/* non blocking function used to read back already started regular conversion*/
void RCM_ReadOngoingConv (void);

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __regular_conversion_manager_h */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

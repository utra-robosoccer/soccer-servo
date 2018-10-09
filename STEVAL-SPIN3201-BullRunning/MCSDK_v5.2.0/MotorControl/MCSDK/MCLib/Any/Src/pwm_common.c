/**
  ******************************************************************************
  * @file    pwm_common.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement common features
  *          of the PWM & Current Feedback component of the Motor Control SDK:
  *
  *           * start timers (main and auxiliary) synchronously
  *
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

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup pwm_curr_fdbk PWM & Current Feedback
  *
  * @brief PWM & Current Feedback components of the Motor Control SDK
  *
  * These components fulfill two functions in a Motor Control subsystem:
  *
  * - The generation of the Space Vector Pulse Width Modulation on the motor's phases
  * - The sampling of the actual motor's phases current
  *
  * Both these features are closely related as the instants when the values of the phase currents
  * should be sampled by the ADC channels are basically triggered by the timers used to generate
  * the duty cycles for the PWM.
  *
  * Several implementation of PWM and Current Feedback components are provided by the Motor Control
  * SDK to account for the specificities of the application:
  *
  * - The selected MCU: the number of ADCs available on a given MCU, the presence of internal
  * compoarators or OpAmps, for instance, lead to different implementation of this feature
  * - The Current sensing topology also has an impact on the firmware: implementations are provided
  * for Insulated Current Sensors, Single Shunt and Three Shunt resistors current sensing topologies
  *
  * The choice of the implementation mostly depend on these two factors and is performed by the
  * Motor Control Workbench tool.
  *
  * All these implementations are built on a base PWM & Current Feedback component that they extend
  * and that provides the functions and data that are common to all of them. This base component is
  * never used directly as it does not provide a complete implementation of the features. Rather,
  * its handle structure (PWMC_Handle) is reused by all the PWM & Current Feedback specific
  * implementations and the functions it provides form the API of the PWM and Current feedback feature.
  * Calling them results in calling functions of the component that actually implement the feature.
  * See PWMC_Handle for more details on this mechanism.
  * @{
  */


/**
 * @brief  It perform the start of all the timers required by the control.
 *          It utilizes TIM2 as temporary timer to achieve synchronization between
 *          PWM signals.
 *          When this function is called, TIM1 and/or TIM8 must be in frozen state
 *          with CNT, ARR, REP RATE and trigger correctly set (these setting are
 *          usually performed in the Init method accordingly with the configuration)
 * @param  none
 * @retval none
 */
void startTimers( void )
{
  uint32_t isTIM2ClockOn;
  uint32_t trigOut;

  isTIM2ClockOn = LL_APB1_GRP1_IsEnabledClock ( LL_APB1_GRP1_PERIPH_TIM2 );
  if ( isTIM2ClockOn == 0 )
  {
    /* Temporary Enable TIM2 clock if not already on */
    LL_APB1_GRP1_EnableClock ( LL_APB1_GRP1_PERIPH_TIM2 );
    LL_TIM_GenerateEvent_UPDATE ( TIM2 );
    LL_APB1_GRP1_DisableClock ( LL_APB1_GRP1_PERIPH_TIM2 );
  }
  else
  {
    trigOut = LL_TIM_ReadReg( TIM2, CR2 ) & TIM_CR2_MMS;
    LL_TIM_SetTriggerOutput( TIM2, LL_TIM_TRGO_UPDATE );
    LL_TIM_GenerateEvent_UPDATE ( TIM2 );
    LL_TIM_SetTriggerOutput( TIM2, trigOut );
  }
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/


/**
  ******************************************************************************
  * @file    mc_parameters.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides definitions of HW parameters specific to the 
  *          configuration of the subsystem.
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
#include "main.h"
#include "parameters_conversion.h"

#include "r3_f0xx_pwm_curr_fdbk.h"
 
 

  #define MAX_TWAIT 0                 /* Dummy value for single drive */
  #define FREQ_RATIO 1                /* Dummy value for single drive */
  #define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */

/**
  * @brief  Current sensor parameters Single Drive - three shunt, STM32F0X
  */
const R3_F0XX_Params_t R3_F0XX_Params =
{
/* Current reading A/D Conversions initialization -----------------------------*/
  .b_ISamplingTime =  LL_ADC_SAMPLINGTIME_7CYCLES_5,
  .bIaChannel = MC_ADC_CHANNEL_5,
  .bIbChannel = MC_ADC_CHANNEL_4,
  .bIcChannel = MC_ADC_CHANNEL_3,

/* PWM generation parameters --------------------------------------------------*/
  .hDeadTime = DEAD_TIME_COUNTS,
  .bRepetitionCounter = REP_COUNTER,
  .hTafter = TW_AFTER,
  .hTbefore = TW_BEFORE, 
  .TIMx = PWM_TIM1,
  
/* PWM Driving signals initialization ----------------------------------------*/
  .LowSideOutputs= (LowSideOutputsFunction_t)LOW_SIDE_SIGNALS_ENABLING,  
                    
};
   
    
 
 
 
 

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/

/**
  ******************************************************************************
  * @file    parameters_conversion_f0xx.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the definitions needed to convert MC SDK parameters
  *          so as to target the STM32F0 Family.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PARAMETERS_CONVERSION_F0XX_H
#define __PARAMETERS_CONVERSION_F0XX_H

#include "pmsm_motor_parameters.h"
#include "power_stage_parameters.h"
#include "drive_parameters.h"

/************************* CPU & ADC PERIPHERAL CLOCK CONFIG ******************/
#define SYSCLK_FREQ      48000000uL
#define TIM_CLOCK_DIVIDER  1 
#define ADV_TIM_CLK_MHz    48
#define ADC_CLK_MHz    14uL /* Maximum ADC Clock Frequency expressed in MHz */
#define HALL_TIM_CLK       48000000uL
#define ADC1_2  ADC1

/*********************** SENSORLESS REV-UP PARAMETERS *************************/
#define FIRST_SLESS_ALGO_PHASE (ENABLE_SL_ALGO_FROM_PHASE-1u)  

/* Legacy for WB 4.0 Beta */
#if !defined(OPEN_LOOP_VF)
#define OPEN_LOOP_VF false
#endif
#if !defined(OPEN_LOOP_OFF)
#define OPEN_LOOP_OFF 4400
#endif
#if !defined(OPEN_LOOP_K)
#define OPEN_LOOP_K 44
#endif

/*************************  IRQ Handler Mapping  *********************/														  
 #define CURRENT_REGULATION_IRQHandler          DMA1_Channel1_IRQHandler
#define TIMx_UP_BRK_M1_IRQHandler               TIM1_BRK_UP_TRG_COM_IRQHandler
#define DMAx_R1_M1_IRQHandler                   DMA1_Channel4_5_IRQHandler

/**********  AUXILIARY TIMER (SINGLE SHUNT) *************/
 
#define R1_PWM_AUX_TIM                  TIM3

#define TRIG_CONV_LATENCY_NS	259ul /* Referred to Maximum value indicated 
                                    in the Datasheet Table 50 if ADC clock = HSI14 */ 				   
#define SAMPLING_TIME_CORRECTION_FACTOR_NS           500ul/ADC_CLK_MHz                 /* 0.5*1000/ADC_CLK_MHz */ 
#define SAMPLING_TIME_NS ((7 * 1000uL/ADC_CLK_MHz)+SAMPLING_TIME_CORRECTION_FACTOR_NS)

#define ADC_CONV_NB_CK 13u
#define ADC_CONV_TIME_NS    (uint16_t) (((ADC_CONV_NB_CK*1000ul)-500ul)/ADC_CLK_MHz)

#define TW_BEFORE (((uint16_t)(((2*SAMPLING_TIME_NS)+ TRIG_CONV_LATENCY_NS + ADC_CONV_TIME_NS)*ADV_TIM_CLK_MHz)/1000ul)+1u)

#define M1_VBUS_SW_FILTER_BW_FACTOR      10u

/* Sampling time allowed for F0xx are: 1, 7, 13, 28 ADC clock cycle */    
#define M1_VBUS_SAMPLING_TIME  LL_ADC_SAMPLINGTIME_28CYCLES_5

/* Sampling time allowed for F0xx are:  1, 7, 13, 28 ADC clock cycle */    
#define M1_TEMP_SAMPLING_TIME  LL_ADC_SAMPLINGTIME_1CYCLE_5

#endif /*__PARAMETERS_CONVERSION_F0XX_H*/

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/

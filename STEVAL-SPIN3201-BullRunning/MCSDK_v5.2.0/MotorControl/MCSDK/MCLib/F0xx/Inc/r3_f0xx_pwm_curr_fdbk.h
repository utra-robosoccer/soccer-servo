/**
  ******************************************************************************
  * @file    r3_f0xx_pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          r3_f0xx_pwm_curr_fdbk component of the Motor Control SDK.
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
  * @ingroup r3_f0XX_pwm_curr_fdbk
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __R3_F0XX_PWMNCURRFDBK_H
#define __R3_F0XX_PWMNCURRFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "pwm_curr_fdbk.h"

/**
 * @addtogroup MCSDK
 * @{
 */

/**
 * @addtogroup pwm_curr_fdbk
 * @{
 */

/**
 * @addtogroup r3_f0XX_pwm_curr_fdbk
 * @{
 */

/* Exported constants --------------------------------------------------------*/

/**
* @brief  Maximum number of "regular" conversions handled by the component
*/
#define MAX_REG_CONVERSIONS 17u /*!< Is the maximum number of "regular" (this
                                    means not related to the current reading)
                                    conversions that can be requested to this
                                    component */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  R3_F0XX parameters definition
  */
typedef struct
{
  /* Current reading A/D Conversions initialization -----------------------------*/
  uint8_t b_ISamplingTime;        /*!< Sampling time used to convert hI[a|b|c]Channel.
                                       It must be equal to ADC_SampleTime_xCycles5
                                       x= 1, 7, ...*/

  uint8_t bIaChannel;                  /*!< ADC channel used for conversion of
                                           current Ia. It must be equal to
                                           ADC_CHANNEL_x x= 0, ..., 15*/

  uint8_t bIbChannel;                  /*!< ADC channel used for conversion of
                                           current Ib. It must be equal to
                                           ADC_CHANNEL_x x= 0, ..., 15*/

  uint8_t bIcChannel;                  /*!< ADC channel used for conversion of
                                           current Ia. It must be equal to
                                           ADC_CHANNEL_x x= 0, ..., 15*/

  /* PWM generation parameters --------------------------------------------------*/
  uint16_t hDeadTime;                  /*!< Dead time in number of TIM clock
                                            cycles. If CHxN are enabled, it must
                                            contain the dead time to be generated
                                            by the microcontroller, otherwise it
                                            expresses the maximum dead time
                                            generated by driving network */
  uint8_t  bRepetitionCounter;         /*!< It expresses the number of PWM
                                            periods to be elapsed before compare
                                            registers are updated again. In
                                            particular:
                                            RepetitionCounter= (2* #PWM periods)-1*/
  uint16_t hTafter;                    /*!< It is the sum of dead time plus max
                                            value between rise time and noise time
                                            express in number of TIM clocks.*/
  uint16_t hTbefore;                   /*!< It is the sampling time express in
                                            number of TIM clocks.*/
  TIM_TypeDef * TIMx;                  /*!< It contains the pointer to the timer
                                           used for PWM generation. */
  /* PWM Driving signals initialization ----------------------------------------*/

  LowSideOutputsFunction_t LowSideOutputs; /*!< Low side or enabling signals
                                                generation method are defined
                                                here.*/

  GPIO_TypeDef * pwm_en_u_port;
  uint32_t      pwm_en_u_pin;
  GPIO_TypeDef * pwm_en_v_port;
  uint32_t      pwm_en_v_pin;
  GPIO_TypeDef * pwm_en_w_port;
  uint32_t      pwm_en_w_pin;

  /* PWM Driving signals initialization ----------------------------------------*/

  /* Add here extra parameters required by F0xx micro ex. AF settings*/


} R3_F0XX_Params_t;

/**
  * @brief  Handle structure of the r1_f0xx_pwm_curr_fdbk Component
  */
typedef struct
{
  PWMC_Handle_t _Super;         /*!< Offset of current sensing network  */
  uint32_t wPhaseAOffset;       /*!< Offset of Phase A current sensing network  */
  uint32_t wPhaseBOffset;       /*!< Offset of Phase B current sensing network  */
  uint32_t wPhaseCOffset;       /*!< Offset of Phase C current sensing network  */
  uint16_t Half_PWMPeriod;      /*!< Half PWM Period in timer clock counts */
  uint16_t hRegConv;            /*!< Variable used to store regular conversions
                                 *   result*/
  /* Add here extra variables required by F0xx micro ex. User conversions.*/

  bool OverCurrentFlag;         /*!< This flag is set when an overcurrent occurs.*/
  bool OverVoltageFlag;         /*!< This flag is set when an overvoltage occurs.*/
  bool BrakeActionLock;         /*!< This flag is set to avoid that brake action is
                                 *   interrupted.*/
  volatile uint8_t bIndex;               /*!< Number of conversions performed during the
                                 *   calibration phase*/

  uint16_t hFlags;            /*!< Flags
                                   CURRENT_READING_ON:
                                   CALIB: This flag is used to indicate the ADC calibration
                                          phase in order to avoid concurrent regular conversions */                                

  __IO uint16_t ADC1_DMA_converted[3]; /* Buffer used for DMA data transfer after the ADC conversion */

  uint8_t bCalib_A_index;  /*!< Index of the A-Phase inside the DMA_ADC_BUFFER during the calibration phase*/
  uint8_t bCalib_B_index;  /*!< Index of the B-Phase inside the DMA_ADC_BUFFER during the calibration phase*/
  uint8_t bCalib_C_index;  /*!< Index of the C-Phase inside the DMA_ADC_BUFFER during the calibration phase*/
  bool ADCRegularLocked;        /*!< When it's true, we do not allow usage of ADC to do regular conversion on systick*/
  
  uint32_t ADC_TriggerEdge; /* temporary trigger edge storing */
  uint32_t tmp_CCR4;      /* temporary value to be stored in CCR4, shoadowing trick*/
  uint32_t wADC_CHSELR_AB; /* Pre computed scheduler value to be faster*/
  uint32_t wADC_CHSELR_AC; /* Pre computed scheduler value to be faster*/
  uint32_t wADC_CHSELR_BC; /* Pre computed scheduler value to be faster*/
  uint32_t wADC_CHSELR; 
  
  R3_F0XX_Params_t const * pParams_str;

} PWMC_R3_F0_Handle_t;

/* Exported functions ------------------------------------------------------- */

/**
  * @brief  It perform the start of all the timers required by the control.
  *          It utilizes TIM2 as temporary timer to achieve synchronization between
  *          PWM signals.
  *          When this function is called, TIM1 and/or TIM8 must be in frozen state
  *          with CNT, ARR, REP RATE and trigger correctly set (these setting are
  *          usually performed in the Init method accordingly with the configuration)
  *
  */
void R3F0XX_StartTimers( void );

/**
  * R3_F0XX implementation MC IRQ functions
  */
void * R3F0XX_IRQHandler( PWMC_Handle_t * pHdl, unsigned char flag );

/**
  * It initializes TIM1, ADC1, GPIO, DMA1 and NVIC for three shunt current
  * reading configuration using STM32F0x.
  */
void R3F0XX_Init( PWMC_R3_F0_Handle_t * pHandle );

/**
  * It stores into the handler the voltage present on the
  * current feedback analog channel when no current is flowin into the
  * motor
  */
void R3F0XX_CurrentReadingCalibration( PWMC_Handle_t * pHdl );

/**
 * It computes and return latest converted motor phase currents
 */
void R3F0XX_GetPhaseCurrents( PWMC_Handle_t * pHdl, Curr_Components * pStator_Currents );

/**
  * Configure the ADC for the current sampling related to sector 1.
  * It means set the sampling point via TIM1_Ch4 value and polarity
  * ADC sequence length and channels.
  */
uint16_t R3F0XX_SetADCSampPointSect1( PWMC_Handle_t * pHdl );

/**
  * Configure the ADC for the current sampling related to sector 2.
  * It means set the sampling point via TIM1_Ch4 value and polarity
  * ADC sequence length and channels.
  */
uint16_t R3F0XX_SetADCSampPointSect2( PWMC_Handle_t * pHdl );

/**
  * Configure the ADC for the current sampling related to sector 3.
  * It means set the sampling point via TIM1_Ch4 value and polarity
  * ADC sequence length and channels.
  */
uint16_t R3F0XX_SetADCSampPointSect3( PWMC_Handle_t * pHdl );

/**
  * Configure the ADC for the current sampling related to sector 4.
  * It means set the sampling point via TIM1_Ch4 value and polarity
  * ADC sequence length and channels.
  */
uint16_t R3F0XX_SetADCSampPointSect4( PWMC_Handle_t * pHdl );

/**
  * Configure the ADC for the current sampling related to sector 5.
  * It means set the sampling point via TIM1_Ch4 value and polarity
  * ADC sequence length and channels.
  */
uint16_t R3F0XX_SetADCSampPointSect5( PWMC_Handle_t * pHdl );

/**
  * Configure the ADC for the current sampling related to sector 6.
  * It means set the sampling point via TIM1_Ch4 value and polarity
  * ADC sequence length and channels.
  */
uint16_t R3F0XX_SetADCSampPointSect6( PWMC_Handle_t * pHdl );


/**
  * Configure the ADC for the current sampling during calibration.
  * It means set the sampling point via TIMx_Ch4 value and polarity
  * ADC sequence length and channels.
  */
uint16_t R3F0XX_SetADCSampPointCalibration( PWMC_Handle_t * pHdl );

/**
  * It turns on low sides switches. This function is intended to be
  * used for charging boot capacitors of driving section. It has to be
  * called each motor start-up when using high voltage drivers
  */
void R3F0XX_TurnOnLowSides( PWMC_Handle_t * pHdl );

/**
  * This function enables the PWM outputs
  */
void R3F0XX_SwitchOnPWM( PWMC_Handle_t * pHdl );

/**
  * It disables PWM generation on the proper Timer peripheral acting on
  * MOE bit and reset the TIM status
  */
void R3F0XX_SwitchOffPWM( PWMC_Handle_t * pHdl );

/**
  * Execute a regular conversion.
  * The function is not re-entrant (can't executed twice at the same time)
  * It returns 0xFFFF in case of conversion error.
  */
uint16_t R3F0XX_ExecRegularConv( PWMC_Handle_t * pHdl, uint8_t bChannel );

/**
  * @brief  It sets the specified sampling time for the specified ADC channel
  *         on ADC1. It must be called once for each channel utilized by user
  */
void R3F0XX_ADC_SetSamplingTime( PWMC_Handle_t * pHdl, ADConv_t ADConv_struct );

/**
  * It is used to check if an overcurrent occurred since last call.
  */
uint16_t R3F0XX_IsOverCurrentOccurred( PWMC_Handle_t * pHdl );

/**
 * @brief  It contains the Break event interrupt
 */
void * F0XX_BRK_IRQHandler( PWMC_R3_F0_Handle_t * pHdl );

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

#endif /*__R3_F0XX_PWMNCURRFDBK_H*/

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/

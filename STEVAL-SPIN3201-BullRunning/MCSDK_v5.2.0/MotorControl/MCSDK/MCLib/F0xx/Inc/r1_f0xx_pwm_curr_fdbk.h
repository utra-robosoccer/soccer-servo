/**
  ******************************************************************************
  * @file    r1_f0xx_pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          r1_f0xx_pwm_curr_fdbk component of the Motor Control SDK.
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
  * @ingroup r1_f0XX_pwm_curr_fdbk
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __R1_F0XX_PWMNCURRFDBK_H
#define __R1_F0XX_PWMNCURRFDBK_H

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
 * @addtogroup r1_f0XX_pwm_curr_fdbk
 * @{
 */

/* Exported constants --------------------------------------------------------*/

/**
* @brief  Flags definition
*/
#define EOFOC 0x0001u /*!< Flag to indicate end of FOC duty available */
#define STBD3 0x0002u /*!< Flag to indicate which phase has been distorted
                           in boudary 3 zone (A or B)*/
#define DSTEN 0x0004u /*!< Flag to indicate if the distortion must be performed
                           or not (in case of charge of bootstrap capacitor phase
                           is not required)*/
#define SOFOC 0x0008u /*!< This flag will be reset to zero at the begin of FOC
                           and will be set in the UP IRQ. If at the end of
                           FOC it is set the software error must be generated*/
#define CALIB 0x0010u /*!< This flag is used to indicate the ADC calibration
                           phase in order to avoid concurrent regular conversions*/

/**
* @brief  Maximum number of "regular" conversions handled by the component
*/
#define MAX_REG_CONVERSIONS 17u /*!< Is the maximum number of "regular" (this
                                    means not related to the current reading)
                                    conversions that can be requested to this
                                    component */

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  R1_F0XX parameters definition
  */
typedef struct
{
  /* Current reading A/D Conversions initialization --------------------------*/
  uint8_t b_ISamplingTime;        /*!< Sampling time used to convert hIChannel.
                                       It must be equal to ADC_SampleTime_xCycles5
                                       x= 1, 7, ...*/
  uint8_t hIChannel;              /*!< ADC channel used for conversion of
                                       current. It must be equal to
                                       ADC_CHANNEL_x x= 0, ..., 15*/

  /* PWM generation parameters --------------------------------------------------*/
  uint16_t hDeadTime;             /*!< Dead time in number of TIM clock
                                       cycles. If CHxN are enabled, it must
                                       contain the dead time to be generated
                                       by the microcontroller, otherwise it
                                       expresses the maximum dead time
                                       generated by driving network*/
  uint8_t  bRepetitionCounter;    /*!< It expresses the number of PWM
                                       periods to be elapsed before compare
                                       registers are updated again. In
                                       particular:
                                       RepetitionCounter= (2* PWM periods) -1*/
  uint16_t hTafter;               /*!< It is the sum of dead time plus rise time
                                       express in number of TIM clocks.*/
  uint16_t hTbefore;              /*!< It is the value of sampling time
                                       expressed in numbers of TIM clocks.*/
  uint16_t hTMin;                 /*!< It is the sum of dead time plus rise time
                                       plus sampling time express in numbers of
                                       TIM clocks.*/
  uint16_t hHTMin;                /*!< It is the half of hTMin value.*/
  uint16_t hTSample;              /*!< It is the sampling time express in
                                       numbers of TIM clocks.*/
  uint16_t hMaxTrTs;              /*!< It is the maximum between twice of rise
                                       time express in number of TIM clocks and
                                       twice of sampling time express in numbers
                                       of TIM clocks.*/

  /* PWM Driving signals initialization ----------------------------------------*/
  TIM_TypeDef * TIMx;                   /*!< timer used for PWM generation.*/
  TIM_TypeDef * AuxTIM;            /*!< Auxiliary timer used for single shunt ADC
                                       triggering. It should be TIM3 or TIM4.*/

  LowSideOutputsFunction_t LowSideOutputs; /*!< Low side or enabling signals
                                                generation method are defined
                                                here.*/
  GPIO_TypeDef * pwm_en_u_port;
  uint32_t      pwm_en_u_pin;
  GPIO_TypeDef * pwm_en_v_port;
  uint32_t      pwm_en_v_pin;
  GPIO_TypeDef * pwm_en_w_port;
  uint32_t      pwm_en_w_pin;

} R1_F0XX_Params_t;

/**
  * @brief  Handle structure of the r1_f0xx_pwm_curr_fdbk Component
  */
typedef struct
{
  PWMC_Handle_t _Super;       /*!< Offset of current sensing network  */
  uint16_t Half_PWMPeriod;    /*!< Half PWM Period in timer clock counts  */
  uint16_t hPhaseOffset;      /*!< Offset of current sensing network  */
  uint16_t hDmaBuff[2];       /*!< Buffer used for PWM distortion points*/
  uint16_t hCCDmaBuffCh4[4];  /*!< Buffer used for dual ADC sampling points*/
  uint16_t hCntSmp1;          /*!< First sampling point express in timer counts*/
  uint16_t hCntSmp2;          /*!< Second sampling point express in timer counts*/
  uint8_t sampCur1;           /*!< Current sampled in the first sampling point*/
  uint8_t sampCur2;           /*!< Current sampled in the second sampling point*/
  int16_t hCurrAOld;          /*!< Previous measured value of phase A current*/
  int16_t hCurrBOld;          /*!< Previous measured value of phase B current*/
  int16_t hCurrCOld;          /*!< Previous measured value of phase C current*/
  uint8_t bInverted_pwm;      /*!< This value indicates the type of the previous
                                   PWM period (Regular, Distort PHA, PHB or PHC)*/
  uint8_t bInverted_pwm_new;  /*!< This value indicates the type of the current
                                   PWM period (Regular, Distort PHA, PHB or PHC)*/
  uint16_t hPreloadCCMR2Set;  /*!< Preload value for TIMx->CCMR2 register used to
                                    set the mode of TIMx CH4*/
  uint8_t bDMATot;            /*!< Value to indicate the total number of expected
                                   DMA TC events*/
  uint8_t bDMACur;            /*!< Current number of DMA TC events occurred */
  uint16_t hFlags;            /*!< Flags
                                   EOFOC: Flag to indicate end of FOC duty available
                                   STBD3: Flag to indicate which phase has been distorted
                                          in boudary 3 zone (A or B)
                                   DSTEN: Flag to indicate if the distortion must be
                                          performed or not (charge of bootstrap
                                          capacitor phase)
                                   SOFOC: This flag will be reset to zero at the begin of FOC
                                          and will be set in the UP IRQ. If at the end of
                                          FOC it is set the software error must be generated
                                   CALIB: This flag is used to indicate the ADC calibration
                                          phase in order to avoid concurrent regular conversions
                                   REGCONVONGOING: This flag will be set when a
                                          regconversion is requested by
                                          ExecRegularConv and the ADC has been
                                          set for the conversion
                                          (GetPhaseCurrents), it will be reset
                                          after reading conversion
                                          (CalcDutyCycles).*/
  uint16_t hCurConv[2];       /*!< Array used to store phase current conversions*/
  uint32_t wADC_ExtTrigConv;  /*!< ADC external trigger_sources for channels
                                   conversion. Depends on AuxTim used.*/

  bool OverCurrentFlag;         /*!< This flag is set when an overcurrent occurs.*/
  bool ADCRegularLocked;        /*!< When it's true, we do not allow usage of ADC to do regular conversion on systick*/

  R1_F0XX_Params_t const * pParams_str;

} PWMC_R1_F0_Handle_t;

/* Exported functions ------------------------------------------------------- */

/**
  * It initializes TIM1, ADC, GPIO, DMA1 and NVIC for single shunt current
  * reading configuration using STM32F0XX family.
  */
void R1F0XX_Init( PWMC_R1_F0_Handle_t * pHandle );

/**
  * It stores into the handler the voltage present on the
  * current feedback analog channel when no current is flowin into the
  * motor
  */
void R1F0XX_CurrentReadingCalibration( PWMC_Handle_t * pHdl );

/**
  * It computes and return latest converted motor phase currents motor
  */
void R1F0XX_GetPhaseCurrents( PWMC_Handle_t * pHdl, Curr_Components * pStator_Currents );

/**
  * It turns on low sides switches. This function is intended to be
  * used for charging boot capacitors of driving section. It has to be
  * called each motor start-up when using high voltage drivers
  */
void R1F0XX_TurnOnLowSides( PWMC_Handle_t * pHdl );

/**
  * It enables PWM generation on the proper Timer peripheral acting on
  * MOE bit, enaables the single shunt distortion and reset the TIM status
  */
void R1F0XX_SwitchOnPWM( PWMC_Handle_t * pHdl );

/**
  * It disables PWM generation on the proper Timer peripheral acting on
  * MOE bit, disables the single shunt distortion and reset the TIM status
  */
void R1F0XX_SwitchOffPWM( PWMC_Handle_t * pHdl );

/**
  * Implementation of the single shunt algorithm to setup the
  * TIM1 register and DMA buffers values for the next PWM period.
  */
uint16_t R1F0XX_CalcDutyCycles( PWMC_Handle_t * pHdl );

/**
  * Execute a regular conversion.
  * The function is not re-entrant (can't executed twice at the same time)
  * It returns 0xFFFF in case of conversion error.
  */
uint16_t R1F0XX_ExecRegularConv( PWMC_Handle_t * pHdl, uint8_t bChannel );

/**
  * It sets the specified sampling time for the specified ADC channel
  * on ADC1. It must be called once for each channel utilized by user
  */
void R1F0XX_ADC_SetSamplingTime( PWMC_Handle_t * pHdl, ADConv_t ADConv_struct );

/**
  * It sets the specified sampling time for the specified ADC channel
  * on ADC1. It must be called once for each channel utilized by user
  */
uint16_t R1F0XX_IsOverCurrentOccurred( PWMC_Handle_t * pHdl );

/**
 * @brief  It contains the TIMx Update event interrupt
 */
void * R1F0XX_TIMx_UP_IRQHandler( PWMC_R1_F0_Handle_t * pHdl );

/**
 * @brief  It contains the Break event interrupt
 */
void * F0XX_BRK_IRQHandler( PWMC_R1_F0_Handle_t * pHdl );

/**
 * @brief  It contains the DMA transfer complete event interrupt
 */
void * R1F0XX_DMA_TC_IRQHandler( PWMC_R1_F0_Handle_t * pHdl );

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

#endif /*__R1_F0XX_PWMNCURRFDBK_H*/

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/

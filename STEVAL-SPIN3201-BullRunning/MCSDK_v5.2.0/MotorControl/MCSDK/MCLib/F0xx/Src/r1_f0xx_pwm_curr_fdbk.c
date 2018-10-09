/**
  ******************************************************************************
  * @file    r1_f0xx_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement current sensor
  *          class to be stantiated when the single shunt current sensing
  *          topology is used. It is specifically designed for STM32F0XX
  *          microcontrollers and implements the successive sampling of two motor
  *          current using only one ADC.
  *           + MCU peripheral and handle initialization fucntion
  *           + three shunt current sesnsing
  *           + space vector modulation function
  *           + ADC sampling function
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
#include "pwm_curr_fdbk.h"
#include "r1_f0xx_pwm_curr_fdbk.h"
#include "mc_irq_handler.h"

#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/**
 * @defgroup r1_f0XX_pwm_curr_fdbk R1 F0xx PWM & Current Feedback
 *
 * @brief STM32F0, 1-Shunt PWM & Current Feedback implementation
 *
 * This component is used in applications based on an STM32F0 MCU
 * and using a single shunt resistor current sensing topology.
 *
 * @todo: TODO: complete documentation.
 * @{
 */

/* Private Defines -----------------------------------------------------------*/

/* Direct address of the registers used by DMA */
#define CCR1_OFFSET 0x34u
#define CCR2_OFFSET 0x38u
#define CCR3_OFFSET 0x3Cu
#define CCR4_OFFSET 0x40u
#define TIM1_CCR1_Address   TIM1_BASE + CCR1_OFFSET
#define TIM1_CCR2_Address   TIM1_BASE + CCR2_OFFSET
#define TIM1_CCR3_Address   TIM1_BASE + CCR3_OFFSET
#define TIM3_CCR4_Address   TIM3_BASE + CCR4_OFFSET
#define TIM15_CCR1_Address  TIM15_BASE + CCR1_OFFSET

#define DR_OFFSET 0x40u
#define ADC1_DR_Address     ADC1_BASE + DR_OFFSET

#define NB_CONVERSIONS 16u

#define REGULAR         ((uint8_t)0u)
#define BOUNDARY_1      ((uint8_t)1u)  /* Two small, one big */
#define BOUNDARY_2      ((uint8_t)2u)  /* Two big, one small */
#define BOUNDARY_3      ((uint8_t)3u)  /* Three equal        */

#define INVERT_NONE 0u
#define INVERT_A 1u
#define INVERT_B 2u
#define INVERT_C 3u

#define SAMP_NO 0u
#define SAMP_IA 1u
#define SAMP_IB 2u
#define SAMP_IC 3u
#define SAMP_NIA 4u
#define SAMP_NIB 5u
#define SAMP_NIC 6u
#define SAMP_OLDA 7u
#define SAMP_OLDB 8u
#define SAMP_OLDC 9u

#define CH1NORMAL           0x0060u
#define CH2NORMAL           0x6000u
#define CH3NORMAL           0x0060u
#define CH4NORMAL           0x7000u

#define CCMR1_PRELOAD_DISABLE_MASK 0xF7F7u
#define CCMR2_PRELOAD_DISABLE_MASK 0xFFF7u

#define CCMR1_PRELOAD_ENABLE_MASK 0x0808u
#define CCMR2_PRELOAD_ENABLE_MASK 0x0008u

/* DMA ENABLE mask */
#define CCR_ENABLE_Set          ((uint32_t)0x00000001u)
#define CCR_ENABLE_Reset        ((uint32_t)0xFFFFFFFEu)

#define CR2_JEXTSEL_Reset       ((uint32_t)0xFFFF8FFFu)
#define CR2_JEXTTRIG_Set        ((uint32_t)0x00008000u)
#define CR2_JEXTTRIG_Reset      ((uint32_t)0xFFFF7FFFu)

#define TIM_DMA_ENABLED_CC1 0x0200u
#define TIM_DMA_ENABLED_CC2 0x0400u
#define TIM_DMA_ENABLED_CC3 0x0800u

#define CR2_ADON_Set                ((uint32_t)0x00000001u)

/* ADC SMPx mask */
#define SMPR1_SMP_Set              ((uint32_t) (0x00000007u))
#define SMPR2_SMP_Set              ((uint32_t) (0x00000007u))
#define CR2_EXTTRIG_SWSTART_Set    ((uint32_t)0x00500000)

#define ADC1_CR2_EXTTRIG_SWSTART_BB 0x42248158u

#define ADCx_IRQn     ADC1_COMP_IRQn
#define TIMx_UP_IRQn  TIM1_BRK_UP_TRG_COM_IRQn

/* Constant values -----------------------------------------------------------*/
static const uint8_t REGULAR_SAMP_CUR1[6] = {SAMP_NIC, SAMP_NIC, SAMP_NIA, SAMP_NIA, SAMP_NIB, SAMP_NIB};
static const uint8_t REGULAR_SAMP_CUR2[6] = {SAMP_IA, SAMP_IB, SAMP_IB, SAMP_IC, SAMP_IC, SAMP_IA};
static const uint8_t BOUNDR1_SAMP_CUR2[6] = {SAMP_IB, SAMP_IB, SAMP_IC, SAMP_IC, SAMP_IA, SAMP_IA};
static const uint8_t BOUNDR2_SAMP_CUR1[6] = {SAMP_IA, SAMP_IB, SAMP_IB, SAMP_IC, SAMP_IC, SAMP_IA};
static const uint8_t BOUNDR2_SAMP_CUR2[6] = {SAMP_IC, SAMP_IA, SAMP_IA, SAMP_IB, SAMP_IB, SAMP_IC};

/* Private function prototypes -----------------------------------------------*/
static void R1F0XX_1ShuntMotorVarsInit( PWMC_Handle_t * pHdl );
static void R1F0XX_1ShuntMotorVarsRestart( PWMC_Handle_t * pHdl );
static void R1F0XX_TIMxInit( TIM_TypeDef * TIMx, TIM_TypeDef * TIMx_2, PWMC_Handle_t * pHdl );

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  It initializes TIM1, ADC, GPIO, DMA1 and NVIC for single shunt current
  *         reading configuration using STM32F0XX family.
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R1F0XX_Init( PWMC_R1_F0_Handle_t * pHandle )
{
  uint16_t hAux;
  uint16_t hTIM1_CR1;
  uint16_t hAuxTIM_CR1;
  TIM_TypeDef * AuxTIM;

  if ( ( uint32_t )pHandle == ( uint32_t )&pHandle->_Super )
  {

    /* disable IT and flags in case of LL driver usage
     * workaround for unwanted interrupt enabling done by LL driver */
    LL_ADC_DisableIT_EOC( ADC1 );
    LL_ADC_ClearFlag_EOC( ADC1 );
    LL_ADC_DisableIT_EOS( ADC1 );
    LL_ADC_ClearFlag_EOS( ADC1 );

    AuxTIM = pHandle->pParams_str->AuxTIM;

    R1F0XX_1ShuntMotorVarsInit( &pHandle->_Super );

    R1F0XX_TIMxInit( TIM1, AuxTIM, &pHandle->_Super );

    /* DMA Event related to R1 - Active Vector insertion (TIM1 Channel 4) */
    /* DMA Channel configuration ----------------------------------------------*/
    LL_DMA_SetMemoryAddress( DMA1, LL_DMA_CHANNEL_4, ( uint32_t )pHandle->hDmaBuff );
    LL_DMA_SetPeriphAddress( DMA1, LL_DMA_CHANNEL_4, ( uint32_t ) & ( TIM1->CCR1 ) );
    LL_DMA_SetDataLength( DMA1, LL_DMA_CHANNEL_4, 2u );

    /* Enable DMA Channel */
    LL_DMA_EnableChannel( DMA1, LL_DMA_CHANNEL_4 );

    /* Settings related to AuxTIM used */
    if ( AuxTIM == TIM3 )
    {
      LL_DBGMCU_APB1_GRP1_FreezePeriph( LL_DBGMCU_APB1_GRP1_TIM3_STOP );

      /* DMA Event related to AUX_TIM - dual triggering */
      /* DMA channel configuration ----------------------------------------------*/
      LL_DMA_SetMemoryAddress( DMA1, LL_DMA_CHANNEL_3, ( uint32_t )pHandle->hCCDmaBuffCh4 );
      LL_DMA_SetPeriphAddress( DMA1, LL_DMA_CHANNEL_3, ( uint32_t )TIM3_CCR4_Address );
      LL_DMA_SetDataLength( DMA1, LL_DMA_CHANNEL_3, 3u );
      /* Enable DMA Channel */
      LL_DMA_EnableChannel( DMA1, LL_DMA_CHANNEL_3 );

      pHandle->wADC_ExtTrigConv = LL_ADC_REG_TRIG_EXT_TIM3_TRGO;
    }
#ifdef TIM15
    else /* TIM15 */
    {
      LL_DBGMCU_APB1_GRP2_FreezePeriph( LL_DBGMCU_APB1_GRP2_TIM15_STOP );
      /* DMA Event related to AUX_TIM - dual triggering */
      /* DMA channel configuration ----------------------------------------------*/
      LL_DMA_SetMemoryAddress( DMA1, LL_DMA_CHANNEL_5, ( uint32_t )pHandle->hCCDmaBuffCh4 );
      LL_DMA_SetPeriphAddress( DMA1, LL_DMA_CHANNEL_5, ( uint32_t )TIM15_CCR1_Address );
      LL_DMA_SetDataLength( DMA1, LL_DMA_CHANNEL_5, 3u );
      LL_DMA_EnableChannel( DMA1, LL_DMA_CHANNEL_5 );

      pHandle->wADC_ExtTrigConv = LL_ADC_REG_TRIG_EXT_TIM15_TRGO;
    }
#endif
    /* DMA Event related to ADC conversion*/
    /* DMA channel configuration ----------------------------------------------*/
    LL_DMA_SetMemoryAddress( DMA1, LL_DMA_CHANNEL_2, ( uint32_t )pHandle->hCurConv );
    LL_DMA_SetPeriphAddress( DMA1, LL_DMA_CHANNEL_2, ( uint32_t )ADC1_DR_Address );
    LL_DMA_SetDataLength( DMA1, LL_DMA_CHANNEL_2, 2u );

    /* DMA1 channel 2 will be enabled after the CurrentReadingCalibration */

    if ( pHandle->pParams_str->bRepetitionCounter > 1u )
    {
      /* Only if REP RATE > 1 - Active Vector insertion (TIM1 Channel 4)*/
      /* enable the DMA1_CH4 TC interrupt */
      /* Enable DMA1 CH4 TC IRQ */
      LL_DMA_EnableIT_TC( DMA1, LL_DMA_CHANNEL_4 );

      pHandle->bDMATot = ( pHandle->pParams_str->bRepetitionCounter + 1u ) / 2u;
    }
    else
    {
      /* REP RATE = 1 */
      LL_DMA_DisableIT_TC( DMA1, LL_DMA_CHANNEL_4 );
      pHandle->bDMATot = 0u;
    }

    /* Start calibration of ADC1 */
    LL_ADC_StartCalibration( ADC1 );
    while ((LL_ADC_IsCalibrationOnGoing(ADC1) == SET) ||
           (LL_ADC_REG_IsConversionOngoing(ADC1) == SET) ||
           (LL_ADC_REG_IsStopConversionOngoing(ADC1) == SET) ||
           (LL_ADC_IsDisableOngoing(ADC1) == SET))
    {
      /* wait */
    }

    /* Enable ADC */
    LL_ADC_Enable( ADC1 );

    /* Wait ADC Ready */
    while ( LL_ADC_IsActiveFlag_ADRDY( ADC1 ) == RESET )
    {}


    R1F0XX_1ShuntMotorVarsRestart( &pHandle->_Super );

    /* Set AUX TIM channel first trigger (dummy) - DMA enabling */
    hAux = ( pHandle->Half_PWMPeriod >> 1 ) - pHandle->pParams_str->hTbefore;
    if ( AuxTIM == TIM3 )
    {
      TIM3->CCR4 = hAux;
      LL_TIM_EnableDMAReq_CC4( TIM3 );
    }
#ifdef TIM15
    else /* TIM15 */
    {
      TIM15->CCR1 = hAux;
      LL_TIM_EnableDMAReq_CC1( TIM15 );
    }
#endif
    LL_DMA_EnableIT_TC( DMA1, LL_DMA_CHANNEL_2 );

    LL_TIM_EnableCounter( TIM1 );
    LL_TIM_EnableCounter( AuxTIM );

    hTIM1_CR1 = TIM1->CR1;
    hTIM1_CR1 |= TIM_CR1_CEN;
    hAuxTIM_CR1 = AuxTIM->CR1;
    hAuxTIM_CR1 |= TIM_CR1_CEN;

    AuxTIM->CNT += 3u;

    __disable_irq();
    TIM1->CR1 = hTIM1_CR1;
    AuxTIM->CR1 = hAuxTIM_CR1;
    __enable_irq();

    pHandle->ADCRegularLocked=false; /* We allow ADC usage for regular conversion on Systick*/
    pHandle->_Super.DTTest = 0u;
    pHandle->_Super.DTCompCnt = pHandle->_Super.hDTCompCnt;

  }
}

/**
  * @brief  It initializes TIMx and TIMx_2 peripheral for PWM generation,
  *          active vector insertion and adc triggering.
  * @param  TIMx Timer to be initialized
  * @param  TIMx_2 Auxiliary timer to be initialized used for adc triggering
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
static void R1F0XX_TIMxInit( TIM_TypeDef * TIMx, TIM_TypeDef * TIMx_2, PWMC_Handle_t * pHdl )
{

  PWMC_R1_F0_Handle_t * pHandle = ( PWMC_R1_F0_Handle_t * )pHdl;

  LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH1 );
  LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH2 );
  LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH3 );

  if ( ( pHandle->pParams_str->LowSideOutputs ) == LS_PWM_TIMER )
  {
    LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH1N );
    LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH2N );
    LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH3N );
  }

  LL_TIM_OC_DisablePreload( TIMx, LL_TIM_CHANNEL_CH1 );
  LL_TIM_OC_DisablePreload( TIMx, LL_TIM_CHANNEL_CH2 );
  LL_TIM_OC_DisablePreload( TIMx, LL_TIM_CHANNEL_CH3 );
  LL_TIM_OC_DisablePreload( TIMx, LL_TIM_CHANNEL_CH4 );

  LL_TIM_OC_SetDeadTime( TIMx, ( pHandle->pParams_str->hDeadTime ) / 2u );

  if ( TIMx_2 == TIM3 )
  {
    LL_TIM_OC_SetMode( TIMx_2, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM2 );
    LL_TIM_CC_EnableChannel( TIMx_2, LL_TIM_CHANNEL_CH4 );
  }
  else /* TIM15 */
  {
    LL_TIM_OC_SetMode( TIMx_2, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM2 );
    LL_TIM_CC_EnableChannel( TIMx_2, LL_TIM_CHANNEL_CH1 );
  }

}

/**
  * @brief  It stores into handler the voltage present on the
  *         current feedback analog channel when no current is flowin into the
  *         motor
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R1F0XX_CurrentReadingCalibration( PWMC_Handle_t * pHdl )
{
  uint8_t bIndex = 0u;
  uint32_t wPhaseOffset = 0u;

  PWMC_R1_F0_Handle_t * pHandle = ( PWMC_R1_F0_Handle_t * )pHdl;

  /* Set the CALIB flags to indicate the ADC calibartion phase*/
  pHandle->hFlags |= CALIB;

  /* We forbid ADC usage for regular conversion on Systick*/
  pHandle->ADCRegularLocked=true; 
  /* ADC Channel and sampling time config for current reading */
  LL_ADC_REG_SetSequencerChannels ( ADC1, __LL_ADC_DECIMAL_NB_TO_CHANNEL ( pHandle->pParams_str->hIChannel ));
  LL_ADC_SetSamplingTimeCommonChannels ( ADC1, pHandle->pParams_str->b_ISamplingTime );

  /* Disable DMA1 Channel2 */
  LL_DMA_DisableChannel( DMA1, LL_DMA_CHANNEL_2 );

  /* ADC Channel used for current reading are read
  in order to get zero currents ADC values*/
  while ( bIndex < NB_CONVERSIONS )
  {
    /* Software start of conversion */
    LL_ADC_REG_StartConversion( ADC1 );

    /* Wait until end of regular conversion */
    while ( LL_ADC_IsActiveFlag_EOC( ADC1 ) == RESET )
    {}

    wPhaseOffset += LL_ADC_REG_ReadConversionData12( ADC1 );
    bIndex++;
  }

  pHandle->hPhaseOffset = ( uint16_t )( wPhaseOffset / NB_CONVERSIONS );

  /* Reset the CALIB flags to indicate the end of ADC calibartion phase*/
  pHandle->hFlags &= ( ~CALIB );

  /* Enable DMA1 Channel2 */
  LL_DMA_EnableChannel( DMA1, LL_DMA_CHANNEL_2 );
}

/**
  * @brief  First initialization of class members
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
static void R1F0XX_1ShuntMotorVarsInit( PWMC_Handle_t * pHdl )
{
  PWMC_R1_F0_Handle_t * pHandle = ( PWMC_R1_F0_Handle_t * )pHdl;

  /* Init motor vars */
  pHandle->hPhaseOffset = 0u;
  pHandle->bInverted_pwm = INVERT_NONE;
  pHandle->bInverted_pwm_new = INVERT_NONE;
  pHandle->hFlags &= ( ~STBD3 );
  pHandle->hFlags &= ( ~DSTEN );

  /* After reset value of DMA buffers */
  pHandle->hDmaBuff[0] = pHandle->Half_PWMPeriod + 1u;
  pHandle->hDmaBuff[1] = pHandle->Half_PWMPeriod >> 1;

  /* After reset value of dvDutyValues */
  pHandle->_Super.hCntPhA = pHandle->Half_PWMPeriod >> 1;
  pHandle->_Super.hCntPhB = pHandle->Half_PWMPeriod >> 1;
  pHandle->_Super.hCntPhC = pHandle->Half_PWMPeriod >> 1;

  /* Default value of DutyValues */
  pHandle->hCntSmp1 = ( pHandle->Half_PWMPeriod >> 1 ) - pHandle->pParams_str->hTbefore;
  pHandle->hCntSmp2 = ( pHandle->Half_PWMPeriod >> 1 ) + pHandle->pParams_str->hTafter;

  /* Default value of sampling point */
  pHandle->hCCDmaBuffCh4[0] = pHandle->hCntSmp2; /* Second point */
  pHandle->hCCDmaBuffCh4[1] = ( pHandle->Half_PWMPeriod * 2u ) - 1u;       /* Update */
  pHandle->hCCDmaBuffCh4[2] = pHandle->hCntSmp1; /* First point */

  LL_TIM_DisableDMAReq_CC4( TIM1 );
}

/**
  * @brief  Initialization of class members after each motor start
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
static void R1F0XX_1ShuntMotorVarsRestart( PWMC_Handle_t * pHdl )
{
  PWMC_R1_F0_Handle_t * pHandle = ( PWMC_R1_F0_Handle_t * )pHdl;

  /* Default value of DutyValues */
  pHandle->hCntSmp1 = ( pHandle->Half_PWMPeriod >> 1 ) - pHandle->pParams_str->hTbefore;
  pHandle->hCntSmp2 = ( pHandle->Half_PWMPeriod >> 1 ) + pHandle->pParams_str->hTafter;

  /* Default value of sampling point */
  pHandle->hCCDmaBuffCh4[0] = pHandle->hCntSmp2; /* Second point */
  pHandle->hCCDmaBuffCh4[2] = pHandle->hCntSmp1; /* First point */

  /* After start value of DMA buffers */
  pHandle->hDmaBuff[0] = pHandle->Half_PWMPeriod + 1u;
  pHandle->hDmaBuff[1] = pHandle->Half_PWMPeriod >> 1;

  /* After start value of dvDutyValues */
  pHandle->_Super.hCntPhA = pHandle->Half_PWMPeriod >> 1;
  pHandle->_Super.hCntPhB = pHandle->Half_PWMPeriod >> 1;
  pHandle->_Super.hCntPhC = pHandle->Half_PWMPeriod >> 1;

  /* Set the default previous value of Phase A,B,C current */
  pHandle->hCurrAOld = 0;
  pHandle->hCurrBOld = 0;
  pHandle->hCurrCOld = 0;

  LL_TIM_DisableDMAReq_CC4( TIM1 );
}

/**
  * @brief  It computes and return latest converted motor phase currents motor
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval Curr_Components Ia and Ib current in Curr_Components format
  */
void R1F0XX_GetPhaseCurrents( PWMC_Handle_t * pHdl, Curr_Components * pStator_Currents )
{
  int32_t wAux;
  int16_t hCurrA = 0;
  int16_t hCurrB = 0;
  int16_t hCurrC = 0;
  uint8_t bCurrASamp = 0u;
  uint8_t bCurrBSamp = 0u;
  uint8_t bCurrCSamp = 0u;

  PWMC_R1_F0_Handle_t * pHandle = ( PWMC_R1_F0_Handle_t * )pHdl;

  /* Disabling the External triggering for ADCx*/
  ADC1->CFGR1 &= ~LL_ADC_REG_TRIG_EXT_RISINGFALLING;

  /* Reset the bSOFOC flags to indicate the start of FOC algorithm*/
  pHandle->hFlags &= ( ~SOFOC );

  /* First sampling point */
  wAux = ( int32_t )( pHandle->hCurConv[0] );
  wAux -= ( int32_t )( pHandle->hPhaseOffset );

  /* Check saturation */
  if ( wAux > -INT16_MAX )
  {
    if ( wAux < INT16_MAX )
    {
    }
    else
    {
      wAux = INT16_MAX;
    }
  }
  else
  {
    wAux = -INT16_MAX;
  }

  switch ( pHandle->sampCur1 )
  {
    case SAMP_IA:
      hCurrA = ( int16_t )( wAux );
      bCurrASamp = 1u;
      break;
    case SAMP_IB:
      hCurrB = ( int16_t )( wAux );
      bCurrBSamp = 1u;
      break;
    case SAMP_IC:
      hCurrC = ( int16_t )( wAux );
      bCurrCSamp = 1u;
      break;
    case SAMP_NIA:
      wAux = -wAux;
      hCurrA = ( int16_t )( wAux );
      bCurrASamp = 1u;
      break;
    case SAMP_NIB:
      wAux = -wAux;
      hCurrB = ( int16_t )( wAux );
      bCurrBSamp = 1u;
      break;
    case SAMP_NIC:
      wAux = -wAux;
      hCurrC = ( int16_t )( wAux );
      bCurrCSamp = 1u;
      break;
    case SAMP_OLDA:
      hCurrA = pHandle->hCurrAOld;
      bCurrASamp = 1u;
      break;
    case SAMP_OLDB:
      hCurrB = pHandle->hCurrBOld;
      bCurrBSamp = 1u;
      break;
    default:
      break;
  }

  /* Second sampling point */
  wAux = ( int32_t )( pHandle->hCurConv[1] );
  wAux -= ( int32_t )( pHandle->hPhaseOffset );

  /* Check saturation */
  if ( wAux > -INT16_MAX )
  {
    if ( wAux < INT16_MAX )
    {
    }
    else
    {
      wAux = INT16_MAX;
    }
  }
  else
  {
    wAux = -INT16_MAX;
  }

  switch ( pHandle->sampCur2 )
  {
    case SAMP_IA:
      hCurrA = ( int16_t )( wAux );
      bCurrASamp = 1u;
      break;
    case SAMP_IB:
      hCurrB = ( int16_t )( wAux );
      bCurrBSamp = 1u;
      break;
    case SAMP_IC:
      hCurrC = ( int16_t )( wAux );
      bCurrCSamp = 1u;
      break;
    case SAMP_NIA:
      wAux = -wAux;
      hCurrA = ( int16_t )( wAux );
      bCurrASamp = 1u;
      break;
    case SAMP_NIB:
      wAux = -wAux;
      hCurrB = ( int16_t )( wAux );
      bCurrBSamp = 1u;
      break;
    case SAMP_NIC:
      wAux = -wAux;
      hCurrC = ( int16_t )( wAux );
      bCurrCSamp = 1u;
      break;
    default:
      break;
  }

  /* Computation of the third value */
  if ( bCurrASamp == 0u )
  {
    wAux = -( ( int32_t )( hCurrB ) ) - ( ( int32_t )( hCurrC ) );

    /* Check saturation */
    if ( wAux > -INT16_MAX )
    {
      if ( wAux < INT16_MAX )
      {
      }
      else
      {
        wAux = INT16_MAX;
      }
    }
    else
    {
      wAux = -INT16_MAX;
    }

    hCurrA = ( int16_t )wAux;
  }
  if ( bCurrBSamp == 0u )
  {
    wAux = -( ( int32_t )( hCurrA ) ) - ( ( int32_t )( hCurrC ) );

    /* Check saturation */
    if ( wAux > -INT16_MAX )
    {
      if ( wAux < INT16_MAX )
      {
      }
      else
      {
        wAux = INT16_MAX;
      }
    }
    else
    {
      wAux = -INT16_MAX;
    }

    hCurrB = ( int16_t )wAux;
  }
  if ( bCurrCSamp == 0u )
  {
    wAux = -( ( int32_t )( hCurrA ) ) - ( ( int32_t )( hCurrB ) );

    /* Check saturation */
    if ( wAux > -INT16_MAX )
    {
      if ( wAux < INT16_MAX )
      {
      }
      else
      {
        wAux = INT16_MAX;
      }
    }
    else
    {
      wAux = -INT16_MAX;
    }

    hCurrC = ( int16_t )wAux;
  }

  /* hCurrA, hCurrB, hCurrC values are the sampled values */

  pHandle->hCurrAOld = hCurrA;
  pHandle->hCurrBOld = hCurrB;
  pHandle->hCurrCOld = hCurrC;

  pStator_Currents->qI_Component1 = hCurrA;
  pStator_Currents->qI_Component2 = hCurrB;

  pHandle->_Super.hIa = pStator_Currents->qI_Component1;
  pHandle->_Super.hIb = pStator_Currents->qI_Component2;
  pHandle->_Super.hIc = -pStator_Currents->qI_Component1 - pStator_Currents->qI_Component2;

}

/**
  * @brief  It turns on low sides switches. This function is intended to be
  *         used for charging boot capacitors of driving section. It has to be
  *         called each motor start-up when using high voltage drivers
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R1F0XX_TurnOnLowSides( PWMC_Handle_t * pHdl )
{
  PWMC_R1_F0_Handle_t * pHandle = ( PWMC_R1_F0_Handle_t * )pHdl;

  pHandle->_Super.bTurnOnLowSidesAction = true;

  TIM1->CCR1 = 0u;
  TIM1->CCR2 = 0u;
  TIM1->CCR3 = 0u;

  LL_TIM_ClearFlag_UPDATE( TIM1 );
  while ( LL_TIM_IsActiveFlag_UPDATE( TIM1 ) == RESET )
  {}

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs( TIM1 );
  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
    LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
    LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
  }
  return;
}

/**
  * @brief  It enables PWM generation on the proper Timer peripheral acting on
  *         MOE bit, enaables the single shunt distortion and reset the TIM status
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R1F0XX_SwitchOnPWM( PWMC_Handle_t * pHdl )
{
  PWMC_R1_F0_Handle_t * pHandle = ( PWMC_R1_F0_Handle_t * )pHdl;

  pHandle->_Super.bTurnOnLowSidesAction = false;

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs( TIM1 );
  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
    LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
    LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
  }

  /* Enable UPDATE ISR */
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIM1 );
  LL_TIM_EnableIT_UPDATE( TIM1 );

  /* Enabling distortion for single shunt */
  pHandle->hFlags |= DSTEN;
  /* We forbid ADC usage for regular conversion on Systick*/
  pHandle->ADCRegularLocked=true; 
  return;
}

/**
  * @brief  It disables PWM generation on the proper Timer peripheral acting on
  *         MOE bit, disables the single shunt distortion and reset the TIM status
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R1F0XX_SwitchOffPWM( PWMC_Handle_t * pHdl )
{
  uint16_t hAux;

  PWMC_R1_F0_Handle_t * pHandle = ( PWMC_R1_F0_Handle_t * )pHdl;

  pHandle->_Super.bTurnOnLowSidesAction = false;

  /* Main PWM Output Disable */
  LL_TIM_DisableAllOutputs( TIM1 );
  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
  }

  /* Disable UPDATE ISR */
  LL_TIM_DisableIT_UPDATE( TIM1 );

  /* Disabling distortion for single */
  pHandle->hFlags &= ( ~DSTEN );

  while ( LL_TIM_IsActiveFlag_UPDATE( TIM1 ) == RESET )
  {}
  /* Disabling all DMA previous setting */
  LL_TIM_DisableDMAReq_CC4( TIM1 );
  
  /* We allow ADC usage for regular conversion on Systick*/
  pHandle->ADCRegularLocked=false; 

  /* Set all duty to 50% */
  hAux = pHandle->Half_PWMPeriod >> 1;
  TIM1->CCR1 = hAux;
  TIM1->CCR2 = hAux;
  TIM1->CCR3 = hAux;

  return;
}

/**
  * @brief  Implementation of the single shunt algorithm to setup the
  *         TIM1 register and DMA buffers values for the next PWM period.
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval uint16_t It returns MC_FOC_DURATION if the TIMx update occurs
  *          before the end of FOC algorithm else returns MC_NO_ERROR
  */
uint16_t R1F0XX_CalcDutyCycles( PWMC_Handle_t * pHdl )
{
  int16_t hDeltaDuty_0;
  int16_t hDeltaDuty_1;
  uint16_t hDutyV_0 = 0u;
  uint16_t hDutyV_1 = 0u;
  uint16_t hDutyV_2 = 0u;
  uint8_t bSector;
  uint8_t bStatorFluxPos;
  uint16_t hAux;

  PWMC_R1_F0_Handle_t * pHandle = ( PWMC_R1_F0_Handle_t * )pHdl;

  bSector = ( uint8_t )pHandle->_Super.hSector;

  if ( ( pHandle->hFlags & DSTEN ) != 0u )
  {
    switch ( bSector )
    {
      case SECTOR_1:
        hDutyV_2 = pHandle->_Super.hCntPhA;
        hDutyV_1 = pHandle->_Super.hCntPhB;
        hDutyV_0 = pHandle->_Super.hCntPhC;
        break;
      case SECTOR_2:
        hDutyV_2 = pHandle->_Super.hCntPhB;
        hDutyV_1 = pHandle->_Super.hCntPhA;
        hDutyV_0 = pHandle->_Super.hCntPhC;
        break;
      case SECTOR_3:
        hDutyV_2 = pHandle->_Super.hCntPhB;
        hDutyV_1 = pHandle->_Super.hCntPhC;
        hDutyV_0 = pHandle->_Super.hCntPhA;
        break;
      case SECTOR_4:
        hDutyV_2 = pHandle->_Super.hCntPhC;
        hDutyV_1 = pHandle->_Super.hCntPhB;
        hDutyV_0 = pHandle->_Super.hCntPhA;
        break;
      case SECTOR_5:
        hDutyV_2 = pHandle->_Super.hCntPhC;
        hDutyV_1 = pHandle->_Super.hCntPhA;
        hDutyV_0 = pHandle->_Super.hCntPhB;
        break;
      case SECTOR_6:
        hDutyV_2 = pHandle->_Super.hCntPhA;
        hDutyV_1 = pHandle->_Super.hCntPhC;
        hDutyV_0 = pHandle->_Super.hCntPhB;
        break;
      default:
        break;
    }

    /* Compute delta duty */
    hDeltaDuty_0 = ( int16_t )( hDutyV_1 ) - ( int16_t )( hDutyV_0 );
    hDeltaDuty_1 = ( int16_t )( hDutyV_2 ) - ( int16_t )( hDutyV_1 );

    /* Check region */
    if ( ( uint16_t )hDeltaDuty_0 <= pHandle->pParams_str->hTMin )
    {
      if ( ( uint16_t )hDeltaDuty_1 <= pHandle->pParams_str->hTMin )
      {
        bStatorFluxPos = BOUNDARY_3;
      }
      else
      {
        bStatorFluxPos = BOUNDARY_2;
      }
    }
    else
    {
      if ( ( uint16_t )hDeltaDuty_1 > pHandle->pParams_str->hTMin )
      {
        bStatorFluxPos = REGULAR;
      }
      else
      {
        bStatorFluxPos = BOUNDARY_1;
      }
    }

    if ( bStatorFluxPos == REGULAR )
    {
      pHandle->bInverted_pwm_new = INVERT_NONE;
    }
    else if ( bStatorFluxPos == BOUNDARY_1 ) /* Adjust the lower */
    {
      switch ( bSector )
      {
        case SECTOR_5:
        case SECTOR_6:
          if ( pHandle->_Super.hCntPhA - pHandle->pParams_str->hHTMin - hDutyV_0 > pHandle->pParams_str->hTMin )
          {
            pHandle->bInverted_pwm_new = INVERT_A;
            pHandle->_Super.hCntPhA -= pHandle->pParams_str->hHTMin;
            if ( pHandle->_Super.hCntPhA < hDutyV_1 )
            {
              hDutyV_1 = pHandle->_Super.hCntPhA;
            }
          }
          else
          {
            bStatorFluxPos = BOUNDARY_3;
            if ( ( pHandle->hFlags & STBD3 ) == 0u )
            {
              pHandle->bInverted_pwm_new = INVERT_A;
              pHandle->_Super.hCntPhA -= pHandle->pParams_str->hHTMin;
              pHandle->hFlags |= STBD3;
            }
            else
            {
              pHandle->bInverted_pwm_new = INVERT_B;
              pHandle->_Super.hCntPhB -= pHandle->pParams_str->hHTMin;
              pHandle->hFlags &= ( ~STBD3 );
            }
          }
          break;
        case SECTOR_2:
        case SECTOR_1:
          if ( pHandle->_Super.hCntPhB - pHandle->pParams_str->hHTMin - hDutyV_0 > pHandle->pParams_str->hTMin )
          {
            pHandle->bInverted_pwm_new = INVERT_B;
            pHandle->_Super.hCntPhB -= pHandle->pParams_str->hHTMin;
            if ( pHandle->_Super.hCntPhB < hDutyV_1 )
            {
              hDutyV_1 = pHandle->_Super.hCntPhB;
            }
          }
          else
          {
            bStatorFluxPos = BOUNDARY_3;
            if ( ( pHandle->hFlags & STBD3 ) == 0u )
            {
              pHandle->bInverted_pwm_new = INVERT_A;
              pHandle->_Super.hCntPhA -= pHandle->pParams_str->hHTMin;
              pHandle->hFlags |= STBD3;
            }
            else
            {
              pHandle->bInverted_pwm_new = INVERT_B;
              pHandle->_Super.hCntPhB -= pHandle->pParams_str->hHTMin;
              pHandle->hFlags &= ( ~STBD3 );
            }
          }
          break;
        case SECTOR_4:
        case SECTOR_3:
          if ( pHandle->_Super.hCntPhC - pHandle->pParams_str->hHTMin - hDutyV_0 > pHandle->pParams_str->hTMin )
          {
            pHandle->bInverted_pwm_new = INVERT_C;
            pHandle->_Super.hCntPhC -= pHandle->pParams_str->hHTMin;
            if ( pHandle->_Super.hCntPhC < hDutyV_1 )
            {
              hDutyV_1 = pHandle->_Super.hCntPhC;
            }
          }
          else
          {
            bStatorFluxPos = BOUNDARY_3;
            if ( ( pHandle->hFlags & STBD3 ) == 0u )
            {
              pHandle->bInverted_pwm_new = INVERT_A;
              pHandle->_Super.hCntPhA -= pHandle->pParams_str->hHTMin;
              pHandle->hFlags |= STBD3;
            }
            else
            {
              pHandle->bInverted_pwm_new = INVERT_B;
              pHandle->_Super.hCntPhB -= pHandle->pParams_str->hHTMin;
              pHandle->hFlags &= ( ~STBD3 );
            }
          }
          break;
        default:
          break;
      }
    }
    else if ( bStatorFluxPos == BOUNDARY_2 ) /* Adjust the middler */
    {
      switch ( bSector )
      {
        case SECTOR_4:
        case SECTOR_5: /* Invert B */
          pHandle->bInverted_pwm_new = INVERT_B;
          pHandle->_Super.hCntPhB -= pHandle->pParams_str->hHTMin;
          if ( pHandle->_Super.hCntPhB > 0xEFFFu )
          {
            pHandle->_Super.hCntPhB = 0u;
          }
          break;
        case SECTOR_2:
        case SECTOR_3: /* Invert A */
          pHandle->bInverted_pwm_new = INVERT_A;
          pHandle->_Super.hCntPhA -= pHandle->pParams_str->hHTMin;
          if ( pHandle->_Super.hCntPhA > 0xEFFFu )
          {
            pHandle->_Super.hCntPhA = 0u;
          }
          break;
        case SECTOR_6:
        case SECTOR_1: /* Invert C */
          pHandle->bInverted_pwm_new = INVERT_C;
          pHandle->_Super.hCntPhC -= pHandle->pParams_str->hHTMin;
          if ( pHandle->_Super.hCntPhC > 0xEFFFu )
          {
            pHandle->_Super.hCntPhC = 0u;
          }
          break;
        default:
          break;
      }
    }
    else
    {
      if ( ( pHandle->hFlags & STBD3 ) == 0u )
      {
        pHandle->bInverted_pwm_new = INVERT_A;
        pHandle->_Super.hCntPhA -= pHandle->pParams_str->hHTMin;
        pHandle->hFlags |= STBD3;
      }
      else
      {
        pHandle->bInverted_pwm_new = INVERT_B;
        pHandle->_Super.hCntPhB -= pHandle->pParams_str->hHTMin;
        pHandle->hFlags &= ( ~STBD3 );
      }
    }

    if ( bStatorFluxPos == REGULAR ) /* Regular zone */
    {
      /* First point */
      if ( ( hDutyV_1 - hDutyV_0 - pHandle->pParams_str->hDeadTime ) > pHandle->pParams_str->hMaxTrTs )
      {
        pHandle->hCntSmp1 = hDutyV_0 + hDutyV_1 + pHandle->pParams_str->hDeadTime;
        pHandle->hCntSmp1 >>= 1;
      }
      else
      {
        pHandle->hCntSmp1 = hDutyV_1 - pHandle->pParams_str->hTbefore;
      }
      /* Second point */
      if ( ( hDutyV_2 - hDutyV_1 - pHandle->pParams_str->hDeadTime ) > pHandle->pParams_str->hMaxTrTs )
      {
        pHandle->hCntSmp2 = hDutyV_1 + hDutyV_2 + pHandle->pParams_str->hDeadTime;
        pHandle->hCntSmp2 >>= 1;
      }
      else
      {
        pHandle->hCntSmp2 = hDutyV_2 - pHandle->pParams_str->hTbefore;
      }
    }

    if ( bStatorFluxPos == BOUNDARY_1 ) /* Two small, one big */
    {
      /* First point */
      if ( ( hDutyV_1 - hDutyV_0 - pHandle->pParams_str->hDeadTime ) > pHandle->pParams_str->hMaxTrTs )
      {
        pHandle->hCntSmp1 = hDutyV_0 + hDutyV_1 + pHandle->pParams_str->hDeadTime;
        pHandle->hCntSmp1 >>= 1;
      }
      else
      {
        pHandle->hCntSmp1 = hDutyV_1 - pHandle->pParams_str->hTbefore;
      }
      /* Second point */
      pHandle->hCntSmp2 = pHandle->Half_PWMPeriod + pHandle->pParams_str->hHTMin - pHandle->pParams_str->hTSample;
    }

    if ( bStatorFluxPos == BOUNDARY_2 ) /* Two big, one small */
    {
      /* First point */
      if ( ( hDutyV_2 - hDutyV_1 - pHandle->pParams_str->hDeadTime ) >= pHandle->pParams_str->hMaxTrTs )
      {
        pHandle->hCntSmp1 = hDutyV_1 + hDutyV_2 + pHandle->pParams_str->hDeadTime;
        pHandle->hCntSmp1 >>= 1;
      }
      else
      {
        pHandle->hCntSmp1 = hDutyV_2 - pHandle->pParams_str->hTbefore;
      }
      /* Second point */
      pHandle->hCntSmp2 = pHandle->Half_PWMPeriod + pHandle->pParams_str->hHTMin - pHandle->pParams_str->hTSample;
    }

    if ( bStatorFluxPos == BOUNDARY_3 )
    {
      /* First point */
      pHandle->hCntSmp1 = hDutyV_0 - pHandle->pParams_str->hTbefore; /* Dummy trigger */
      /* Second point */
      pHandle->hCntSmp2 = pHandle->Half_PWMPeriod + pHandle->pParams_str->hHTMin - pHandle->pParams_str->hTSample;
    }
  }
  else
  {
    pHandle->bInverted_pwm_new = INVERT_NONE;
    bStatorFluxPos = REGULAR;
  }

  /* Update Timer Ch 1,2,3 (These value are required before update event) */

  pHandle->hFlags |= EOFOC;
  /* Check if DMA transition has been completed */
  if ( pHandle->bDMACur == 0u )
  {
    /* Preload Enable */
    TIM1->CCMR1 |= CCMR1_PRELOAD_ENABLE_MASK;
    TIM1->CCMR2 |= CCMR2_PRELOAD_ENABLE_MASK;

    TIM1->CCR1 = pHandle->_Super.hCntPhA;
    TIM1->CCR2 = pHandle->_Super.hCntPhB;
    TIM1->CCR3 = pHandle->_Super.hCntPhC;

    /* Update ADC Trigger DMA buffer */
    pHandle->hCCDmaBuffCh4[0] = pHandle->hCntSmp2; /* Second point */
    pHandle->hCCDmaBuffCh4[2] = pHandle->hCntSmp1; /* First point */
  }

  LL_ADC_REG_SetSequencerChannels ( ADC1, __LL_ADC_DECIMAL_NB_TO_CHANNEL ( pHandle->pParams_str->hIChannel ));
  LL_ADC_SetSamplingTimeCommonChannels ( ADC1, pHandle->pParams_str->b_ISamplingTime );
  
   /* Limit for update event */

  /* Check the status of bSOFOC flags if is set the next update event has been
  occurred so an error will be reported*/
  if ( ( pHandle->hFlags & SOFOC ) != 0u )
  {
    hAux = MC_FOC_DURATION;
  }
  else
  {
    hAux = MC_NO_ERROR;
  }

  /* The following instruction can be executed after Update handler
     before the get phase current (Second EOC) */

  /* Set the current sampled */
  if ( bStatorFluxPos == REGULAR ) /* Regual zone */
  {
    pHandle->sampCur1 = REGULAR_SAMP_CUR1[bSector];
    pHandle->sampCur2 = REGULAR_SAMP_CUR2[bSector];
  }

  if ( bStatorFluxPos == BOUNDARY_1 ) /* Two small, one big */
  {
    pHandle->sampCur1 = REGULAR_SAMP_CUR1[bSector];
    pHandle->sampCur2 = BOUNDR1_SAMP_CUR2[bSector];
  }

  if ( bStatorFluxPos == BOUNDARY_2 ) /* Two big, one small */
  {
    pHandle->sampCur1 = BOUNDR2_SAMP_CUR1[bSector];
    pHandle->sampCur2 = BOUNDR2_SAMP_CUR2[bSector];
  }

  if ( bStatorFluxPos == BOUNDARY_3 )
  {
    if ( pHandle->bInverted_pwm_new == INVERT_A )
    {
      pHandle->sampCur1 = SAMP_OLDB;
      pHandle->sampCur2 = SAMP_IA;
    }
    if ( pHandle->bInverted_pwm_new == INVERT_B )
    {
      pHandle->sampCur1 = SAMP_OLDA;
      pHandle->sampCur2 = SAMP_IB;
    }
  }

  /* Limit for the Get Phase current (Second EOC Handler) */

  return ( hAux );
}

/**
 * @brief  It contains the TIMx Update event interrupt
 * @param  pHandle: handler of the current instance of the PWM component
 * @retval none
 */
void * R1F0XX_TIMx_UP_IRQHandler( PWMC_R1_F0_Handle_t * pHandle )
{
  uint8_t bInverted_pwm_new;
  uint32_t wAux;

  /* Critical point start */
  /* Enabling the External triggering for ADCx*/
  wAux = ADC1->CFGR1;
  wAux &= ( ~( ADC_CFGR1_EXTEN | ADC_CFGR1_EXTSEL ) );
  wAux |= ( LL_ADC_REG_TRIG_EXT_RISING | pHandle->wADC_ExtTrigConv );
  ADC1->CFGR1 = wAux;

  /* Enable ADC triggering */
  ADC1->CR |= ( uint32_t )ADC_CR_ADSTART;

  /* Critical point stop */

  /* TMP var to speedup the execution */
  bInverted_pwm_new = pHandle->bInverted_pwm_new;

  if ( bInverted_pwm_new != pHandle->bInverted_pwm )
  {
    /* Set the DMA destination */
    switch ( bInverted_pwm_new )
    {
      case INVERT_A:
        LL_DMA_SetPeriphAddress( DMA1, LL_DMA_CHANNEL_4, TIM1_CCR1_Address );
        LL_TIM_EnableDMAReq_CC4( TIM1 );
        break;

      case INVERT_B:
        LL_DMA_SetPeriphAddress( DMA1, LL_DMA_CHANNEL_4, TIM1_CCR2_Address );
        LL_TIM_EnableDMAReq_CC4( TIM1 );
        break;

      case INVERT_C:
        LL_DMA_SetPeriphAddress( DMA1, LL_DMA_CHANNEL_4, TIM1_CCR3_Address );
        LL_TIM_EnableDMAReq_CC4( TIM1 );
        break;

      default:
        LL_TIM_DisableDMAReq_CC4( TIM1 );
        break;
    }
  }

  /* Clear of End of FOC Flags */
  pHandle->hFlags &= ( ~EOFOC );

  /* Preload Disable */
  TIM1->CCMR1 &= CCMR1_PRELOAD_DISABLE_MASK;
  TIM1->CCMR2 &= CCMR2_PRELOAD_DISABLE_MASK;

  switch ( bInverted_pwm_new )
  {
    case INVERT_A:
      pHandle->hDmaBuff[1] = pHandle->_Super.hCntPhA;
      pHandle->bDMACur = pHandle->bDMATot;
      break;

    case INVERT_B:
      pHandle->hDmaBuff[1] = pHandle->_Super.hCntPhB;
      pHandle->bDMACur = pHandle->bDMATot;
      break;

    case INVERT_C:
      pHandle->hDmaBuff[1] = pHandle->_Super.hCntPhC;
      pHandle->bDMACur = pHandle->bDMATot;
      break;

    default:
      pHandle->bDMACur = 0u;
      break;
  }

  pHandle->bInverted_pwm = bInverted_pwm_new;

  /* Set the bSOFOC flags to indicate the execution of Update IRQ*/
  pHandle->hFlags |= SOFOC;

  return MC_NULL;

}

/**
 * @brief  It contains the Break event interrupt
 * @param  pHandle: handler of the current instance of the PWM component
 * @retval none
 */
void * F0XX_BRK_IRQHandler( PWMC_R1_F0_Handle_t * pHandle )
{

  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
    LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
  }
  pHandle->OverCurrentFlag = true;

  return MC_NULL;
}

/**
 * @brief  It contains the DMA transfer complete event interrupt
 * @param  pHandle: handler of the current instance of the PWM component
 * @retval none
 */
void * R1F0XX_DMA_TC_IRQHandler( PWMC_R1_F0_Handle_t * pHandle )
{

  pHandle->bDMACur--;
  if ( pHandle->bDMACur == 0u )
  {
    if ( ( pHandle->hFlags & EOFOC ) != 0u )
    {
      /* Preload Enable */
      TIM1->CCMR1 |= CCMR1_PRELOAD_ENABLE_MASK;
      TIM1->CCMR2 |= CCMR2_PRELOAD_ENABLE_MASK;

      /* Compare register update */
      TIM1->CCR1 = pHandle->_Super.hCntPhA;
      TIM1->CCR2 = pHandle->_Super.hCntPhB;
      TIM1->CCR3 = pHandle->_Super.hCntPhC;

      /* Update ADC Trigger DMA buffer */
      pHandle->hCCDmaBuffCh4[0] = pHandle->hCntSmp2; /* Second point */
      pHandle->hCCDmaBuffCh4[2] = pHandle->hCntSmp1; /* First point */
    }
  }

  return MC_NULL;
}

/**
  * @brief  It is used to check if an overcurrent occurred since last call.
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval uint16_t It returns MC_BREAK_IN whether an overcurrent has been
  *                  detected since last method call, MC_NO_FAULTS otherwise.
  */
uint16_t R1F0XX_IsOverCurrentOccurred( PWMC_Handle_t * pHdl )
{
  uint16_t retVal = MC_NO_FAULTS;

  if ( LL_TIM_IsActiveFlag_BRK( TIM1 ) )
  {
    retVal = MC_BREAK_IN;
    LL_TIM_ClearFlag_BRK( TIM1 );
  }
  return retVal;
}

/**
  * @}
  */

/**
  * @}
  */

/** @} */

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/

/**
  ******************************************************************************
  * @file    r3_f0xx_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement current sensor
  *          class to be stantiated when the three shunts current sensing
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
#include "r3_f0xx_pwm_curr_fdbk.h"
#include "mc_irq_handler.h"

#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/**
 * @defgroup r3_f0XX_pwm_curr_fdbk R3 F0xx PWM & Current Feedback
 *
 * @brief STM32F0, 3-Shunt PWM & Current Feedback implementation
 *
 * This component is used in applications based on an STM32F0 MCU
 * and using a three shunt resistors current sensing topology.
 *
 * @todo: TODO: complete documentation.
 * @{
 */

/* Private defines -----------------------------------------------------------*/

#define TIMxCCER_MASK_CH123       (TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC3E|\
                                   TIM_CCER_CC1NE|TIM_CCER_CC2NE|TIM_CCER_CC3NE)
#define NB_CONVERSIONS 16u
#define ADC1_DR_Address   0x40012440  /* Value from Memory Mapping Section of the STM32F030 DataSheets */
#define CCMR2_CH4_DISABLE 0x8FFFu
#define CCMR2_CH4_PWM1    0x6000u
#define ADC_CLEAR_TRIG_EDGE_Mask  (~LL_ADC_REG_TRIG_EXT_RISINGFALLING) /* 32 bit Mask */
#define CURRENT_READING_ON   0x0002u /*!< Flag to indicate the active states
                                          conditions (as START and RUN states) when
                                          the currents reading is executed, to avoid the
                                          the regular conversions during these states. */

#define CALIB 0x0010u                /*!< This flag is used to indicate the ADC
                                          calibration phase in order to avoid
                                          concurrent regular conversions*/

/* Private function prototypes -----------------------------------------------*/
static void R3F0XX_TIMxInit( PWMC_Handle_t * pHdl );
static void R3F0XX_HFCurrentsCalibration( PWMC_Handle_t * pHdl, Curr_Components * pStator_Currents );
static uint16_t R3F0XX_WriteTIMRegisters( PWMC_Handle_t * pHdl );

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  It initializes TIM1, ADC1, GPIO, DMA1 and NVIC for three shunt current
  *         reading configuration using STM32F0x.
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R3F0XX_Init( PWMC_R3_F0_Handle_t * pHandle )
{

  if ( ( uint32_t )pHandle == ( uint32_t )&pHandle->_Super )
  {

    /* disable IT and flags in case of LL driver usage
     * workaround for unwanted interrupt enabling done by LL driver */
    LL_ADC_DisableIT_EOC( ADC1 );
    LL_ADC_ClearFlag_EOC( ADC1 );
    LL_ADC_DisableIT_EOS( ADC1 );
    LL_ADC_ClearFlag_EOS( ADC1 );

    /* Enable the CCS */
    LL_RCC_HSE_EnableCSS();

    /* Peripheral clocks enabling END ----------------------------------------*/

    R3F0XX_TIMxInit( &pHandle->_Super );

    /* Clear TIMx break flag. */
    LL_TIM_ClearFlag_BRK( TIM1 );

    /* TIM1 Counter Clock stopped when the core is halted */
    LL_APB1_GRP2_EnableClock (LL_APB1_GRP2_PERIPH_DBGMCU);
    LL_DBGMCU_APB1_GRP2_FreezePeriph( LL_DBGMCU_APB1_GRP2_TIM1_STOP );

    /* ADC Calibration */
    LL_ADC_StartCalibration( ADC1 );
    while ((LL_ADC_IsCalibrationOnGoing(ADC1) == SET) ||
           (LL_ADC_REG_IsConversionOngoing(ADC1) == SET) ||
           (LL_ADC_REG_IsStopConversionOngoing(ADC1) == SET) ||
           (LL_ADC_IsDisableOngoing(ADC1) == SET))
    {
      /* wait */
    }

    /* Enables the ADC peripheral */
    LL_ADC_Enable( ADC1 );

    /* Wait ADC Ready */
    while ( LL_ADC_IsActiveFlag_ADRDY( ADC1 ) == RESET )
    {
      /* wait */
    }

    /* DMA1 Channel1 Config */
    LL_DMA_SetMemoryAddress( DMA1, LL_DMA_CHANNEL_1, ( uint32_t )pHandle->ADC1_DMA_converted );
    LL_DMA_SetPeriphAddress( DMA1, LL_DMA_CHANNEL_1, ( uint32_t )ADC1_DR_Address );
    LL_DMA_SetDataLength( DMA1, LL_DMA_CHANNEL_1, 3 );

    /* Enables the DMA1 Channel1 peripheral */
    LL_DMA_EnableChannel( DMA1, LL_DMA_CHANNEL_1 );
    
    /* Pre Configure ADC scheduler to go faster */
    pHandle->wADC_CHSELR_AB = (( __LL_ADC_DECIMAL_NB_TO_CHANNEL ( pHandle->pParams_str->bIaChannel ) 
                                 | __LL_ADC_DECIMAL_NB_TO_CHANNEL ( pHandle->pParams_str->bIbChannel ) )
                                 & ADC_CHANNEL_ID_BITFIELD_MASK) ;
    pHandle->wADC_CHSELR_AC = (( __LL_ADC_DECIMAL_NB_TO_CHANNEL ( pHandle->pParams_str->bIaChannel ) 
                                 | __LL_ADC_DECIMAL_NB_TO_CHANNEL ( pHandle->pParams_str->bIcChannel ) )
                                 & ADC_CHANNEL_ID_BITFIELD_MASK) ;
    pHandle->wADC_CHSELR_BC = (( __LL_ADC_DECIMAL_NB_TO_CHANNEL ( pHandle->pParams_str->bIbChannel ) 
                                 | __LL_ADC_DECIMAL_NB_TO_CHANNEL ( pHandle->pParams_str->bIcChannel ) )
                                 & ADC_CHANNEL_ID_BITFIELD_MASK) ;

    /* Clear the flags */
    pHandle->OverVoltageFlag = false;
    pHandle->OverCurrentFlag = false;

    /* We allow ADC usage for regular conversion on Systick*/
    pHandle->ADCRegularLocked=false; 

    pHandle->_Super.DTTest = 0u;
    pHandle->_Super.DTCompCnt = pHandle->_Super.hDTCompCnt;
  }
}

/**
* @brief  It initializes TIMx peripheral for PWM generation
* @param  TIMx Timer to be initialized
* @param  pHdl: handler of the current instance of the PWM component
* @retval none
*/
static void R3F0XX_TIMxInit( PWMC_Handle_t * pHdl )
{

  PWMC_R3_F0_Handle_t * pHandle = ( PWMC_R3_F0_Handle_t * )pHdl;

  LL_TIM_EnableCounter( TIM1 );

  LL_TIM_CC_EnableChannel( TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3 );

  if ( ( pHandle->pParams_str->LowSideOutputs ) == LS_PWM_TIMER )
  {
    LL_TIM_CC_EnableChannel( TIM1, LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3N );
  }

  LL_TIM_ClearFlag_BRK( TIM1 );

  LL_TIM_EnableIT_BRK( TIM1 );

  /* Prepare timer for synchronization */
  LL_TIM_GenerateEvent_UPDATE( TIM1 );

  LL_TIM_SetCounter( TIM1, ( uint32_t )( pHandle->Half_PWMPeriod ) - 1u );

  pHandle->_Super.DTTest = 0u;
  pHandle->_Super.DTCompCnt = pHandle->_Super.hDTCompCnt;
}

/**
  * @brief  It stores into the handler the voltage present on the
  *         current feedback analog channel when no current is flowin into the
  *         motor
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R3F0XX_CurrentReadingCalibration( PWMC_Handle_t * pHdl )
{
  PWMC_R3_F0_Handle_t * pHandle = ( PWMC_R3_F0_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = TIM1;

  uint16_t hCalibrationPeriodCounter;
  uint16_t hMaxPeriodsNumber;

  /* Set the CALIB flags to indicate the ADC calibartion phase*/
  pHandle->hFlags |= CALIB;
  
  pHandle-> wPhaseAOffset = 0u;
  pHandle-> wPhaseBOffset = 0u;
  pHandle-> wPhaseCOffset = 0u;

  pHandle->bIndex = 0u;

  /* ADC sequence conversion detection to a correct DMA Buffer reading */
  if ( pHandle->pParams_str->bIaChannel < pHandle->pParams_str->bIbChannel )
  {
    if ( pHandle->pParams_str->bIbChannel < pHandle->pParams_str->bIcChannel )
    {
      pHandle->bCalib_A_index = 0u;
      pHandle->bCalib_B_index = 1u;
      pHandle->bCalib_C_index = 2u;
    }
    else
    {
      if ( pHandle->pParams_str->bIaChannel < pHandle->pParams_str->bIcChannel )
      {
        pHandle->bCalib_A_index = 0u;
        pHandle->bCalib_B_index = 2u;
        pHandle->bCalib_C_index = 1u;
      }
      else
      {
        pHandle->bCalib_A_index = 1u;
        pHandle->bCalib_B_index = 2u;
        pHandle->bCalib_C_index = 0u;
      }
    }
  }
  else
  {
    if ( pHandle->pParams_str->bIaChannel < pHandle->pParams_str->bIcChannel )
    {
      pHandle->bCalib_A_index = 1u;
      pHandle->bCalib_B_index = 0u;
      pHandle->bCalib_C_index = 2u;
    }
    else
    {
      if ( pHandle->pParams_str->bIbChannel < pHandle->pParams_str->bIcChannel )
      {
        pHandle->bCalib_A_index = 2u;
        pHandle->bCalib_B_index = 0u;
        pHandle->bCalib_C_index = 1u;
      }
      else
      {
        pHandle->bCalib_A_index = 2u;
        pHandle->bCalib_B_index = 1u;
        pHandle->bCalib_C_index = 0u;
      }
    }
  }

  /* It forces inactive level on TIMx CHy and CHyN */
  LL_TIM_CC_DisableChannel( TIMx, TIMxCCER_MASK_CH123 );

  /* Offset calibration for A B c phases */
  /* Change function to be executed in ADCx_ISR */
  pHandle->_Super.pFctGetPhaseCurrents     = &R3F0XX_HFCurrentsCalibration;
  pHandle->_Super.pFctSetADCSampPointSect1 = &R3F0XX_SetADCSampPointCalibration;
  pHandle->_Super.pFctSetADCSampPointSect2 = &R3F0XX_SetADCSampPointCalibration;
  pHandle->_Super.pFctSetADCSampPointSect3 = &R3F0XX_SetADCSampPointCalibration;
  pHandle->_Super.pFctSetADCSampPointSect4 = &R3F0XX_SetADCSampPointCalibration;
  pHandle->_Super.pFctSetADCSampPointSect5 = &R3F0XX_SetADCSampPointCalibration;
  pHandle->_Super.pFctSetADCSampPointSect6 = &R3F0XX_SetADCSampPointCalibration;

  R3F0XX_SwitchOnPWM( &pHandle->_Super );

  /* Wait for NB_CONVERSIONS to be executed */
  hMaxPeriodsNumber = ( NB_CONVERSIONS + 1u ) * ( ( ( uint16_t )( pHandle->pParams_str->bRepetitionCounter ) + 1u ) >>
                      1 );

  LL_TIM_ClearFlag_CC1( TIMx );

  hCalibrationPeriodCounter = 0u;
  while ( pHandle->bIndex < NB_CONVERSIONS )
  {
    if ( LL_TIM_IsActiveFlag_CC1( TIMx ) )
    {
      LL_TIM_ClearFlag_CC1( TIMx );
      hCalibrationPeriodCounter++;
      if ( hCalibrationPeriodCounter >= hMaxPeriodsNumber )
      {
        if ( pHandle->bIndex < NB_CONVERSIONS )
        {
          pHandle->_Super.SWerror = 1u;
          break;
        }
      }
    }
    else
    {

    }
  }

  R3F0XX_SwitchOffPWM( &pHandle->_Super );

  /* Mean Value of PhaseCurrents Offset calculation by 4bit shifting operation
  *  instead division by NB_CONVERSIONS value fixed to 16. */

  pHandle->wPhaseAOffset = pHandle->wPhaseAOffset / NB_CONVERSIONS;
  pHandle->wPhaseBOffset = pHandle->wPhaseBOffset / NB_CONVERSIONS;
  pHandle->wPhaseCOffset = pHandle->wPhaseCOffset / NB_CONVERSIONS;

  /* Change back function to be executed in ADCx_ISR */
  pHandle->_Super.pFctGetPhaseCurrents     = &R3F0XX_GetPhaseCurrents;
  pHandle->_Super.pFctSetADCSampPointSect1 = &R3F0XX_SetADCSampPointSect1;
  pHandle->_Super.pFctSetADCSampPointSect2 = &R3F0XX_SetADCSampPointSect2;
  pHandle->_Super.pFctSetADCSampPointSect3 = &R3F0XX_SetADCSampPointSect3;
  pHandle->_Super.pFctSetADCSampPointSect4 = &R3F0XX_SetADCSampPointSect4;
  pHandle->_Super.pFctSetADCSampPointSect5 = &R3F0XX_SetADCSampPointSect5;
  pHandle->_Super.pFctSetADCSampPointSect6 = &R3F0XX_SetADCSampPointSect6;

  /* It over write TIMx CCRy wrongly written by FOC during calibration so as to
     force 50% duty cycle on the three inverer legs */
  /* Disable TIMx preload */

  TIMx->CCMR1 &= 0xF7F7u;
  TIMx->CCMR2 &= 0xF7F7u;
  TIMx->CCR1 = pHandle->Half_PWMPeriod;
  TIMx->CCR2 = pHandle->Half_PWMPeriod;
  TIMx->CCR3 = pHandle->Half_PWMPeriod;

  /* Enable TIMx preload */
  TIMx->CCMR1 |= 0x0808u;
  TIMx->CCMR2 |= 0x0808u;

  /* It re-enable drive of TIMx CHy and CHyN by TIMx CHyRef*/
  LL_TIM_CC_EnableChannel( TIMx, TIMxCCER_MASK_CH123 );

  pHandle->BrakeActionLock = false;

  /* Reset the CALIB flags to indicate the end of ADC calibartion phase*/
  pHandle->hFlags &= ( ~CALIB );

}

/**
 * @brief  It computes and return latest converted motor phase currents
 * @param  pHdl: handler of the current instance of the PWM component
 * @retval Curr_Components Ia and Ib current in Curr_Components format
 */
void R3F0XX_GetPhaseCurrents( PWMC_Handle_t * pHdl, Curr_Components * pStator_Currents )
{
  uint8_t bSector;
  int32_t wAux;
  uint16_t hReg1;
  uint16_t hReg2;

  PWMC_R3_F0_Handle_t * pHandle = ( PWMC_R3_F0_Handle_t * )pHdl;

  /* Clear the flag to indicate the start of FOC algorithm*/
  LL_TIM_ClearFlag_UPDATE( TIM1 );
  
  /* Move CCR4 above ARR to disable triggering. 
     Required in case of repetition counter > 1 */
  LL_TIM_OC_DisablePreload( TIM1, LL_TIM_CHANNEL_CH4 );
  TIM1->CCR4 = 0xFFFFu;

  bSector = ( uint8_t ) pHandle->_Super.hSector;

  switch ( bSector )
  {
    case SECTOR_4:
    case SECTOR_5:
      /* Current on Phase C is not accessible     */

      /* Phase Currents conversion sequence detection. It depends on the value of
         ADC_Channel. It permits to read from ADC1_Converted DMA Buffer the correct
         value of Pase Currents */
      if ( pHandle->pParams_str->bIaChannel < pHandle->pParams_str->bIbChannel )
      {
        hReg1 = pHandle->ADC1_DMA_converted[0];
        hReg2 = pHandle->ADC1_DMA_converted[1];
      }
      else
      {
        hReg1 = pHandle->ADC1_DMA_converted[1];
        hReg2 = pHandle->ADC1_DMA_converted[0];
      }
      /* ----------------------------------------------------------------------- */

      /* Ia = PhaseAOffset - ADC converted value) ------------------------------*/

      wAux = ( int32_t )( pHandle->wPhaseAOffset ) - ( int32_t )( hReg1 );

      /* Saturation of Ia */
      if ( wAux < -INT16_MAX )
      {
        pStator_Currents->qI_Component1 = -INT16_MAX;
      }
      else  if ( wAux > INT16_MAX )
      {
        pStator_Currents->qI_Component1 = INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component1 = ( int16_t )wAux;
      }

      /* Ib = PhaseBOffset - ADC converted value) ------------------------------*/

      wAux = ( int32_t )( pHandle->wPhaseBOffset ) - ( int32_t )( hReg2 );

      /* Saturation of Ib */
      if ( wAux < -INT16_MAX )
      {
        pStator_Currents->qI_Component2 = -INT16_MAX;
      }
      else  if ( wAux > INT16_MAX )
      {
        pStator_Currents->qI_Component2 = INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component2 = ( int16_t )wAux;
      }
      break;

    case SECTOR_6:
    case SECTOR_1:
      /* Current on Phase A is not accessible     */

      /* Phase Currents conversion sequence detection. It depends on the value of
         ADC_Channel. It permits to read from ADC1_Converted DMA Buffer the correct
         value of Pase Currents */
      if ( pHandle->pParams_str->bIbChannel < pHandle->pParams_str->bIcChannel )
      {
        hReg1 = pHandle->ADC1_DMA_converted[0];
        hReg2 = pHandle->ADC1_DMA_converted[1];
      }
      else
      {
        hReg1 = pHandle->ADC1_DMA_converted[1];
        hReg2 = pHandle->ADC1_DMA_converted[0];
      }
      /* ----------------------------------------------------------------------- */

      /* Ib = (PhaseBOffset - ADC converted value) ------------------------------*/
      wAux = ( int32_t )( pHandle->wPhaseBOffset ) - ( int32_t )( hReg1 );

      /* Saturation of Ib */
      if ( wAux < -INT16_MAX )
      {
        pStator_Currents->qI_Component2 = -INT16_MAX;
      }
      else  if ( wAux > INT16_MAX )
      {
        pStator_Currents->qI_Component2 = INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component2 = ( int16_t )wAux;
      }

      wAux = ( int32_t )( pHandle->wPhaseCOffset ) - ( int32_t )( hReg2 );
      /* Ia = -Ic -Ib ----------------------------------------------------------*/
      wAux = -wAux - ( int32_t )pStator_Currents->qI_Component2;        /* Ia  */

      /* Saturation of Ia */
      if ( wAux > INT16_MAX )
      {
        pStator_Currents->qI_Component1 = INT16_MAX;
      }
      else  if ( wAux < -INT16_MAX )
      {
        pStator_Currents->qI_Component1 = -INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component1 = ( int16_t )wAux;
      }
      break;

    case SECTOR_2:
    case SECTOR_3:
      /* Current on Phase B is not accessible     */

      /* Phase Currents conversion sequence detection. It depends on the value of
       ADC_Channel. It permits to read from ADC1_Converted DMA Buffer the correct
       value of Pase Currents */
      if ( pHandle->pParams_str->bIaChannel < pHandle->pParams_str->bIcChannel )
      {
        hReg1 = pHandle->ADC1_DMA_converted[0];
        hReg2 = pHandle->ADC1_DMA_converted[1];
      }
      else
      {
        hReg1 = pHandle->ADC1_DMA_converted[1];
        hReg2 = pHandle->ADC1_DMA_converted[0];
      }
      /* ----------------------------------------------------------------------- */


      /* Ia = PhaseAOffset - ADC converted value) ------------------------------*/
      wAux = ( int32_t )( pHandle->wPhaseAOffset ) - ( int32_t )( hReg1 );

      /* Saturation of Ia */
      if ( wAux < -INT16_MAX )
      {
        pStator_Currents->qI_Component1 = -INT16_MAX;
      }
      else  if ( wAux > INT16_MAX )
      {
        pStator_Currents->qI_Component1 = INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component1 = ( int16_t )wAux;
      }

      /* Ic = PhaseCOffset - ADC converted value) ------------------------------*/
      wAux = ( int32_t )( pHandle->wPhaseCOffset ) - ( int32_t )( hReg2 );

      /* Ib = -Ic -Ia */
      wAux = -wAux -  ( int32_t )pStator_Currents->qI_Component1;         /* Ib  */

      /* Saturation of Ib */
      if ( wAux > INT16_MAX )
      {
        pStator_Currents->qI_Component2 = INT16_MAX;
      }
      else  if ( wAux < -INT16_MAX )
      {
        pStator_Currents->qI_Component2 = -INT16_MAX;
      }
      else
      {
        pStator_Currents->qI_Component2 = ( int16_t )wAux;
      }
      break;

    default:
      break;
  }

  pHandle->_Super.hIa = pStator_Currents->qI_Component1;
  pHandle->_Super.hIb = pStator_Currents->qI_Component2;
  pHandle->_Super.hIc = -pStator_Currents->qI_Component1 - pStator_Currents->qI_Component2;
}

/**
  * @brief  Configure the ADC for the current sampling during calibration.
  *         It means set the sampling point via TIMx_Ch4 value and polarity
  *         ADC sequence length and channels.
  *         And call the WriteTIMRegisters method.
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
uint16_t R3F0XX_SetADCSampPointCalibration( PWMC_Handle_t * pHdl )
{
  PWMC_R3_F0_Handle_t * pHandle = ( PWMC_R3_F0_Handle_t * )pHdl;
  pHandle->ADC_TriggerEdge = LL_ADC_REG_TRIG_EXT_RISING;
  pHandle-> tmp_CCR4 = ( uint32_t )( pHandle->Half_PWMPeriod ) - 1u;
  pHandle->wADC_CHSELR = (( __LL_ADC_DECIMAL_NB_TO_CHANNEL ( pHandle->pParams_str->bIaChannel ) 
                          | __LL_ADC_DECIMAL_NB_TO_CHANNEL ( pHandle->pParams_str->bIbChannel ) 
                          | __LL_ADC_DECIMAL_NB_TO_CHANNEL ( pHandle->pParams_str->bIcChannel ))
                                 & ADC_CHANNEL_ID_BITFIELD_MASK) ;
 /* comparator 1 is used to count the number of PWM cycles, so better to keep it at
  *  a known value */
  pHandle->_Super.hCntPhA = ( uint32_t )( pHandle->Half_PWMPeriod ) >> 1;
  return R3F0XX_WriteTIMRegisters( pHdl );
}

/**
  * @brief  Configure the ADC for the current sampling related to sector 1.
  *         It means set the sampling point via TIM1_Ch4 value, the ADC sequence
  *         and channels.
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
uint16_t R3F0XX_SetADCSampPointSect1( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp;
  uint16_t hDeltaDuty;
  PWMC_R3_F0_Handle_t * pHandle = ( PWMC_R3_F0_Handle_t * )pHdl;

  /* Check if sampling AB in the middle of PWM is possible */
  if ( ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhA ) > pHandle->pParams_str->hTafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4 (could be also sector 5) */
    pHandle->_Super.hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = ( uint32_t )( pHandle->Half_PWMPeriod ) - 1u;

    pHandle->wADC_CHSELR = pHandle->wADC_CHSELR_AB;

  }
  else
  {
    /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/

    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
        duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
        one with minimum complementary duty and one with variable complementary duty. In this case, phases
        with variable complementary duty and with maximum duty are converted and the first will be always
        the phase with variable complementary duty cycle */

    /* Crossing Point Searching */
    hDeltaDuty = ( uint16_t )( pHandle->_Super.hCntPhA - pHandle->_Super.hCntPhB );

    /* Definition of crossing point */
    if ( hDeltaDuty > ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhA ) * 2u )
    {
      hCntSmp = pHandle->_Super.hCntPhA - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHandle->_Super.hCntPhA + pHandle->pParams_str->hTafter;

      if ( hCntSmp >= pHandle->Half_PWMPeriod )
      {
        pHandle->ADC_TriggerEdge = LL_ADC_REG_TRIG_EXT_FALLING;

        hCntSmp = ( 2u * pHandle->Half_PWMPeriod ) - hCntSmp - 1u;
      }
    }
    pHandle->wADC_CHSELR = pHandle->wADC_CHSELR_BC;
  }
  /* Set TIMx_CH4 value */
  pHandle -> tmp_CCR4 = hCntSmp;
  

  return R3F0XX_WriteTIMRegisters( &pHandle->_Super );
}

/**
* @brief  Configure the ADC for the current sampling related to sector 2.
*         It means set the sampling point via TIM1_Ch4 value and polarity
*         ADC sequence length and channels.
* @param  pHdl: handler of the current instance of the PWM component
* @retval none
*/
uint16_t R3F0XX_SetADCSampPointSect2( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp;
  uint16_t hDeltaDuty;
  PWMC_R3_F0_Handle_t * pHandle = ( PWMC_R3_F0_Handle_t * )pHdl;


  /* Check if sampling AB in the middle of PWM is possible */
  if ( ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhB ) > pHandle->pParams_str->hTafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4 (could be also sector 5) */
    pHandle->_Super.hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = ( uint32_t )( pHandle->Half_PWMPeriod ) - 1u;

    pHandle->wADC_CHSELR = pHandle->wADC_CHSELR_AB;
  }
  else
  {
    /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/

    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
        duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
        one with minimum complementary duty and one with variable complementary duty. In this case, phases
        with variable complementary duty and with maximum duty are converted and the first will be always
        the phase with variable complementary duty cycle */

    /* Crossing Point Searching */
    hDeltaDuty = ( uint16_t )( pHandle->_Super.hCntPhB - pHandle->_Super.hCntPhA );

    /* Definition of crossing point */
    if ( hDeltaDuty > ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhB ) * 2u )
    {
      hCntSmp = pHandle->_Super.hCntPhB - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHandle->_Super.hCntPhB + pHandle->pParams_str->hTafter;

      if ( hCntSmp >= pHandle->Half_PWMPeriod )
      {

        pHandle->ADC_TriggerEdge = LL_ADC_REG_TRIG_EXT_FALLING;


        hCntSmp = ( 2u * pHandle->Half_PWMPeriod ) - hCntSmp - 1u;
      }
    }

    pHandle->wADC_CHSELR = pHandle->wADC_CHSELR_AC;
  }
  /* Set TIMx_CH4 value */
  pHandle -> tmp_CCR4 = hCntSmp;

  return R3F0XX_WriteTIMRegisters( &pHandle->_Super );
}

/**
* @brief  Configure the ADC for the current sampling related to sector 3.
*         It means set the sampling point via TIM1_Ch4 value and polarity
*         ADC sequence length and channels.
*         And call the WriteTIMRegisters method.
* @param  pHdl: handler of the current instance of the PWM component
* @retval none
*/
uint16_t R3F0XX_SetADCSampPointSect3( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp;
  uint16_t hDeltaDuty;
  PWMC_R3_F0_Handle_t * pHandle = ( PWMC_R3_F0_Handle_t * )pHdl;


  /* Check if sampling AB in the middle of PWM is possible */
  if ( ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhB ) > pHandle->pParams_str->hTafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4 (could be also sector 5) */
    pHandle->_Super.hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = ( uint32_t )( pHandle->Half_PWMPeriod ) - 1u;

    pHandle->wADC_CHSELR = pHandle->wADC_CHSELR_AB;

  }
  else
  {
    /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/

    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
        duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
        one with minimum complementary duty and one with variable complementary duty. In this case, phases
        with variable complementary duty and with maximum duty are converted and the first will be always
        the phase with variable complementary duty cycle */

    /* Crossing Point Searching */
    hDeltaDuty = ( uint16_t )( pHandle->_Super.hCntPhB - pHandle->_Super.hCntPhC );

    /* Definition of crossing point */
    if ( hDeltaDuty > ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhB ) * 2u )
    {
      hCntSmp = pHandle->_Super.hCntPhB - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHandle->_Super.hCntPhB + pHandle->pParams_str->hTafter;

      if ( hCntSmp >= pHandle->Half_PWMPeriod )
      {
        pHandle->ADC_TriggerEdge = LL_ADC_REG_TRIG_EXT_FALLING;

        hCntSmp = ( 2u * pHandle->Half_PWMPeriod ) - hCntSmp - 1u;
      }
    }

    pHandle->wADC_CHSELR = pHandle->wADC_CHSELR_AC;
  }
  /* Set TIMx_CH4 value */
  pHandle -> tmp_CCR4 = hCntSmp;

  return R3F0XX_WriteTIMRegisters( &pHandle->_Super );
}

/**
* @brief  Configure the ADC for the current sampling related to sector 4.
*         It means set the sampling point via TIM1_Ch4 value and polarity
*         ADC sequence length and channels.
*         And call the WriteTIMRegisters method.
* @param  pHdl: handler of the current instance of the PWM component
* @retval none
*/
uint16_t R3F0XX_SetADCSampPointSect4( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp;
  uint16_t hDeltaDuty;
  
  PWMC_R3_F0_Handle_t * pHandle = ( PWMC_R3_F0_Handle_t * )pHdl;

  /* Check if sampling AB in the middle of PWM is possible */
  if ( ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhC ) > pHandle->pParams_str->hTafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4 (could be also sector 5) */
    pHandle->_Super.hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = ( uint32_t )( pHandle->Half_PWMPeriod ) - 1u;

    pHandle->wADC_CHSELR = pHandle->wADC_CHSELR_AB;

  }
  else
  {
    /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/

    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
        duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
        one with minimum complementary duty and one with variable complementary duty. In this case, phases
        with variable complementary duty and with maximum duty are converted and the first will be always
        the phase with variable complementary duty cycle */

    /* Crossing Point Searching */
    hDeltaDuty = ( uint16_t )( pHandle->_Super.hCntPhC - pHandle->_Super.hCntPhB );

    /* Definition of crossing point */
    if ( hDeltaDuty > ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhC ) * 2u )
    {
      hCntSmp = pHandle->_Super.hCntPhC - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHandle->_Super.hCntPhC + pHandle->pParams_str->hTafter;

      if ( hCntSmp >= pHandle->Half_PWMPeriod )
      {
        pHandle->ADC_TriggerEdge = LL_ADC_REG_TRIG_EXT_FALLING;

        hCntSmp = ( 2u * pHandle->Half_PWMPeriod ) - hCntSmp - 1u;
      }
    }

    pHandle->wADC_CHSELR = pHandle->wADC_CHSELR_AB;
  }
  /* Set TIMx_CH4 value */
  pHandle -> tmp_CCR4 = hCntSmp;

  return R3F0XX_WriteTIMRegisters( &pHandle->_Super );
}

/**
* @brief  Configure the ADC for the current sampling related to sector 5.
*         It means set the sampling point via TIM1_Ch4 value and polarity
*         ADC sequence length and channels.
*         And call the WriteTIMRegisters method.
* @param  pHdl: handler of the current instance of the PWM component
* @retval none
*/
uint16_t R3F0XX_SetADCSampPointSect5( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp;
  uint16_t hDeltaDuty;

  PWMC_R3_F0_Handle_t * pHandle = ( PWMC_R3_F0_Handle_t * )pHdl;

  /* Check if sampling AB in the middle of PWM is possible */
  if ( ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhC ) > pHandle->pParams_str->hTafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4 (could be also sector 5) */
    pHandle->_Super.hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = ( uint32_t )( pHandle->Half_PWMPeriod ) - 1u;

    pHandle->wADC_CHSELR = pHandle->wADC_CHSELR_AB;

  }
  else
  {
    /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/

    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
        duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
        one with minimum complementary duty and one with variable complementary duty. In this case, phases
        with variable complementary duty and with maximum duty are converted and the first will be always
        the phase with variable complementary duty cycle */

    /* Crossing Point Searching */
    hDeltaDuty = ( uint16_t )( pHandle->_Super.hCntPhC - pHandle->_Super.hCntPhA );

    /* Definition of crossing point */
    if ( hDeltaDuty > ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhC ) * 2u )
    {
      hCntSmp = pHandle->_Super.hCntPhC - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHandle->_Super.hCntPhC + pHandle->pParams_str->hTafter;

      if ( hCntSmp >= pHandle->Half_PWMPeriod )
      {
        pHandle->ADC_TriggerEdge = LL_ADC_REG_TRIG_EXT_FALLING;

        hCntSmp = ( 2u * pHandle->Half_PWMPeriod ) - hCntSmp - 1u;
      }
    }

    pHandle->wADC_CHSELR = pHandle->wADC_CHSELR_AB;
  }
  /* Set TIMx_CH4 value */
  pHandle -> tmp_CCR4 = hCntSmp;

  return R3F0XX_WriteTIMRegisters( &pHandle->_Super );
}

/**
* @brief  Configure the ADC for the current sampling related to sector 6.
*         It means set the sampling point via TIM1_Ch4 value and polarity
*         ADC sequence length and channels.
*         And call the WriteTIMRegisters method.
* @param  pHdl: handler of the current instance of the PWM component
* @retval none
*/
uint16_t R3F0XX_SetADCSampPointSect6( PWMC_Handle_t * pHdl )
{
  uint16_t hCntSmp;
  uint16_t hDeltaDuty;

  PWMC_R3_F0_Handle_t * pHandle = ( PWMC_R3_F0_Handle_t * )pHdl;

  /* Check if sampling AB in the middle of PWM is possible */
  if ( ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhA ) > pHandle->pParams_str->hTafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
     * to sector 4 (could be also sector 5) */
    pHandle->_Super.hSector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = ( uint32_t )( pHandle->Half_PWMPeriod ) - 1u;

    pHandle->wADC_CHSELR = pHandle->wADC_CHSELR_AB;

  }
  else
  {
    /* In this case it is necessary to convert phases with Maximum and variable complementary duty cycle.*/

    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
        duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
        one with minimum complementary duty and one with variable complementary duty. In this case, phases
        with variable complementary duty and with maximum duty are converted and the first will be always
        the phase with variable complementary duty cycle */

    /* Crossing Point Searching */
    hDeltaDuty = ( uint16_t )( pHandle->_Super.hCntPhA - pHandle->_Super.hCntPhC );

    /* Definition of crossing point */
    if ( hDeltaDuty > ( uint16_t )( pHandle->Half_PWMPeriod - pHandle->_Super.hCntPhA ) * 2u )
    {
      hCntSmp = pHandle->_Super.hCntPhA - pHandle->pParams_str->hTbefore;
    }
    else
    {
      hCntSmp = pHandle->_Super.hCntPhA + pHandle->pParams_str->hTafter;

      if ( hCntSmp >= pHandle->Half_PWMPeriod )
      {

        pHandle->ADC_TriggerEdge = LL_ADC_REG_TRIG_EXT_FALLING;

        hCntSmp = ( 2u * pHandle->Half_PWMPeriod ) - hCntSmp - 1u;
      }
    }

    pHandle->wADC_CHSELR = pHandle->wADC_CHSELR_BC;
  }
  /* Set TIMx_CH4 value */
  pHandle -> tmp_CCR4 = hCntSmp;

  return R3F0XX_WriteTIMRegisters( &pHandle->_Super );
}

/**
* @brief  Write the dutycycle into timer regiters and check for FOC duration.
* @param  pHandle Pointer on the target component instance.
* @retval none
*/
static uint16_t R3F0XX_WriteTIMRegisters( PWMC_Handle_t * pHdl )
{
  uint16_t hAux;

  PWMC_R3_F0_Handle_t * pHandle = ( PWMC_R3_F0_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = TIM1;

  TIM1->CCR1 = pHandle->_Super.hCntPhA;
  TIM1->CCR2 = pHandle->_Super.hCntPhB;
  TIM1->CCR3 = pHandle->_Super.hCntPhC;

  LL_TIM_OC_EnablePreload( TIMx, LL_TIM_CHANNEL_CH4 );
  TIMx->CCR4 = pHandle->tmp_CCR4;
 /* Re-configuration of CCR4 must be done before the timer update to be taken
    into account at the next PWM cycle. Otherwise we are too late, we flag a
    FOC_DURATION error */
  if ( LL_TIM_IsActiveFlag_UPDATE( TIM1 ) )
  {
    hAux = MC_FOC_DURATION;
  }
  else
  {
    hAux = MC_NO_ERROR;
  }

  if ( pHandle->_Super.SWerror == 1u )
  {
    hAux = MC_FOC_DURATION;
    pHandle->_Super.SWerror = 0u;
  }
  /* Set the trigger polarity as computed inside SetADCSampPointSectX*/
  LL_ADC_REG_SetTriggerEdge (ADC1, pHandle->ADC_TriggerEdge);
  /* Configure the ADC scheduler as selected inside SetADCSampPointSectX*/
  ADC1->CHSELR = pHandle->wADC_CHSELR;
  /* ReConfigure sampling time, as deconfigured by reg_conv_manager */
  LL_ADC_SetSamplingTimeCommonChannels ( ADC1, pHandle->pParams_str->b_ISamplingTime );
  /* ADC needs to be restarted because DMA is configured as limited */
  LL_ADC_REG_StartConversion( ADC1 );

  /* Reset the ADC trigger edge for next conversion */
  pHandle->ADC_TriggerEdge = LL_ADC_REG_TRIG_EXT_RISING;
  
  return hAux;
}

/**
* @brief  Implementaion of PWMC_GetPhaseCurrents to be performed during
*         calibration. It sum up ADC conversion data into wPhaseAOffset and
*         wPhaseBOffset to compute the offset introduced in the current feedback
*         network. It is required to proper configure ADC inputs before to enable
*         the offset computation.
* @param  pHandle Pointer on the target component instance.
* @retval It always returns {0,0} in Curr_Components format
*/
static void R3F0XX_HFCurrentsCalibration( PWMC_Handle_t * pHdl, Curr_Components * pStator_Currents )
{
  PWMC_R3_F0_Handle_t * pHandle = ( PWMC_R3_F0_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = TIM1;

  /* Clear the flag to indicate the start of FOC algorithm*/
  LL_TIM_ClearFlag_UPDATE( TIMx );
  

  if ( pHandle->bIndex < NB_CONVERSIONS )
  {
    pHandle->wPhaseAOffset += pHandle->ADC1_DMA_converted[pHandle->bCalib_A_index];
    pHandle->wPhaseBOffset += pHandle->ADC1_DMA_converted[pHandle->bCalib_B_index];
    pHandle->wPhaseCOffset += pHandle->ADC1_DMA_converted[pHandle->bCalib_C_index];
    pHandle->bIndex++;
  }
}


/**
  * @brief  It turns on low sides switches. This function is intended to be
  *         used for charging boot capacitors of driving section. It has to be
  *         called each motor start-up when using high voltage drivers
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R3F0XX_TurnOnLowSides( PWMC_Handle_t * pHdl )
{
  PWMC_R3_F0_Handle_t * pHandle = ( PWMC_R3_F0_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = TIM1;

  pHandle->_Super.bTurnOnLowSidesAction = true;

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  /*Turn on the three low side switches */
  TIMx->CCR1 = 0u;
  TIMx->CCR2 = 0u;
  TIMx->CCR3 = 0u;

  /* Wait until next update */
  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == RESET )
  {}

  /* Main PWM Output Enable */
  TIMx->BDTR |= TIM_BDTR_MOE;

  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
    LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
    LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );

  }
  return;
}

/**
  * @brief  This function enables the PWM outputs
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R3F0XX_SwitchOnPWM( PWMC_Handle_t * pHdl )
{
  PWMC_R3_F0_Handle_t * pHandle = ( PWMC_R3_F0_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = TIM1;

  pHandle->_Super.bTurnOnLowSidesAction = false;

  /* We forbid ADC usage for regular conversion on Systick*/
  pHandle->ADCRegularLocked=true; 
  
  /* Wait for a new PWM period */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == RESET )
  {}

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  /* Set all duty to 50% */
  TIMx->CCR1 = ( uint32_t )( pHandle->Half_PWMPeriod ) >> 1;
  TIMx->CCR2 = ( uint32_t )( pHandle->Half_PWMPeriod ) >> 1;
  TIMx->CCR3 = ( uint32_t )( pHandle->Half_PWMPeriod ) >> 1;
  TIMx->CCR4 = ( uint32_t )( pHandle->Half_PWMPeriod ) - 5u;

  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == RESET )
  {}

  /* Main PWM Output Enable */
  TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
  TIMx->BDTR |= TIM_BDTR_MOE;

  if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
  {
    if ( ( TIMx->CCER & TIMxCCER_MASK_CH123 ) != 0u )
    {
      LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
      LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
      LL_GPIO_SetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
    }
    else
    {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );

    }
  }

  /* If Calibration is true */
  if ( ( pHandle->hFlags & CALIB ) != 0u )
  {
    /* Calibration */

    /* Configuration of DMA and ADC to next conversions */
    /* It's possible write the CHSELR resgister because the ADC conversion
       is stopped by the R3F0XX_SwitchOffPWM function */
    LL_ADC_REG_SetSequencerChannels ( ADC1,
                                      __LL_ADC_DECIMAL_NB_TO_CHANNEL ( pHandle->pParams_str->bIaChannel ) |
                                      __LL_ADC_DECIMAL_NB_TO_CHANNEL ( pHandle->pParams_str->bIbChannel ) |
                                      __LL_ADC_DECIMAL_NB_TO_CHANNEL ( pHandle->pParams_str->bIcChannel ) );
    LL_ADC_SetSamplingTimeCommonChannels ( ADC1, pHandle->pParams_str->b_ISamplingTime );

    /* Setting of the DMA Buffer Size.*/
    /* NOTE. This register (CNDTRx) must not be written when the DMAy Channel x is ENABLED */
    LL_DMA_DisableChannel( DMA1, LL_DMA_CHANNEL_1 );
    /* Write the Buffer size on CNDTR register */
    LL_DMA_SetDataLength( DMA1, LL_DMA_CHANNEL_1, 3u );

    /* DMA Enabling */
    LL_DMA_EnableChannel( DMA1, LL_DMA_CHANNEL_1 );

    /* Clear EOC */
    LL_ADC_ClearFlag_EOC( ADC1 );

    /* Enable ADC DMA request*/
    LL_ADC_REG_SetDMATransfer( ADC1, LL_ADC_REG_DMA_TRANSFER_LIMITED );

    /* External Trigger of ADC Enabling */
    ADC1->CFGR1 &= ADC_CLEAR_TRIG_EDGE_Mask;
    ADC1->CFGR1 |= LL_ADC_REG_TRIG_EXT_RISING;

    /* Start of ADC */
    LL_ADC_REG_StartConversion( ADC1 );
  }
  else
  {
    /* Configuration of DMA and ADC to next conversions */
    /* It's possible write the CHSELR resgister because the ADC conversion
       is stopped by the R3F0XX_SwitchOffPWM function */

    /* Selection of ADC channels to convert */
    LL_ADC_REG_SetSequencerChannels ( ADC1,
                                      __LL_ADC_DECIMAL_NB_TO_CHANNEL ( pHandle->pParams_str->bIaChannel ) |
                                      __LL_ADC_DECIMAL_NB_TO_CHANNEL ( pHandle->pParams_str->bIbChannel ) );
    LL_ADC_SetSamplingTimeCommonChannels ( ADC1, pHandle->pParams_str->b_ISamplingTime );

    /* Setting of the DMA Buffer Size.*/
    /* NOTE. This register (CNDTRx) must not be written
       when the DMAy Channel x is ENABLED*/

    /* DMA Disabling*/
    LL_DMA_DisableChannel( DMA1, LL_DMA_CHANNEL_1 );

    /* Write the Buffer size on CNDTR register - Two currents to convert */
    LL_DMA_SetDataLength( DMA1, LL_DMA_CHANNEL_1, 2u );

    /* DMA Enabling*/
    LL_DMA_EnableChannel( DMA1, LL_DMA_CHANNEL_1 );

    /* Clear EOC */
    LL_ADC_ClearFlag_EOC( ADC1 );

    /* Enable ADC DMA request*/
    LL_ADC_REG_SetDMATransfer( ADC1, LL_ADC_REG_DMA_TRANSFER_LIMITED );

    /* External Trigger of ADC Enabling */
    ADC1->CFGR1 &= ADC_CLEAR_TRIG_EDGE_Mask;
    ADC1->CFGR1 |= LL_ADC_REG_TRIG_EXT_RISING;

    /* Start of ADC */
    LL_ADC_REG_StartConversion( ADC1 );

  }
  /* Clear Pending Interrupt Bits */
  LL_DMA_ClearFlag_HT1( DMA1 ); // TBC: for TC1, GL1 (not cleared ...)

  /* DMA Interrupt Event configuration */
  LL_DMA_EnableIT_TC( DMA1, LL_DMA_CHANNEL_1 );

  /* Set the RUN flag to indicate the NOT IDLE condition */
  pHandle->hFlags |= CURRENT_READING_ON;

  return;
}


/**
  * @brief  It disables PWM generation on the proper Timer peripheral acting on
  *         MOE bit and reset the TIM status
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval none
  */
void R3F0XX_SwitchOffPWM( PWMC_Handle_t * pHdl )
{
  PWMC_R3_F0_Handle_t * pHandle = ( PWMC_R3_F0_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = TIM1;

  pHandle->_Super.bTurnOnLowSidesAction = false;

  /* Main PWM Output Disable */
  if ( pHandle->BrakeActionLock == true )
  {
  }
  else
  {
    TIMx->BDTR &= ~( ( uint32_t )( LL_TIM_OSSI_ENABLE ) );

    if ( ( pHandle->pParams_str->LowSideOutputs ) == ES_GPIO )
    {
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_u_port, pHandle->pParams_str->pwm_en_u_pin );
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_v_port, pHandle->pParams_str->pwm_en_v_pin );
      LL_GPIO_ResetOutputPin( pHandle->pParams_str->pwm_en_w_port, pHandle->pParams_str->pwm_en_w_pin );
    }
  }
  TIMx->BDTR &= ( uint32_t )~TIM_BDTR_MOE;

  /* Disabling of DMA Interrupt Event configured */
  LL_DMA_DisableIT_TC( DMA1, LL_DMA_CHANNEL_1 );

  LL_ADC_REG_StopConversion( ADC1 );

  /* Disable ADC DMA request*/
  ADC1->CFGR1 &= ~ADC_CFGR1_DMAEN;

  /* Clear Transmission Complete Flag  of DMA1 Channel1 */
  LL_DMA_ClearFlag_TC1( DMA1 );

  /* Clear EOC */
  LL_ADC_ClearFlag_EOC( ADC1 );

  /* The ADC is not triggered anymore by the PWM timer */
  LL_ADC_REG_SetTriggerSource (ADC1, LL_ADC_REG_TRIG_SOFTWARE);
  
 /* We allow ADC usage for regular conversion on Systick*/
  pHandle->ADCRegularLocked=false; 

  return;
}

/**
 * @brief  It contains the Break event interrupt
 * @param  pHandle: handler of the current instance of the PWM component
 * @retval none
 */
void * F0XX_BRK_IRQHandler( PWMC_R3_F0_Handle_t * pHandle )
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
  * @brief  It is used to check if an overcurrent occurred since last call.
  * @param  pHdl: handler of the current instance of the PWM component
  * @retval uint16_t It returns MC_BREAK_IN whether an overcurrent has been
  *                  detected since last method call, MC_NO_FAULTS otherwise.
  */
uint16_t R3F0XX_IsOverCurrentOccurred( PWMC_Handle_t * pHdl )
{
  uint16_t retVal = MC_NO_FAULTS;

  PWMC_R3_F0_Handle_t * pHandle = ( PWMC_R3_F0_Handle_t * )pHdl;

  if ( pHandle->OverVoltageFlag == true )
  {
    retVal = MC_OVER_VOLT;
    pHandle->OverVoltageFlag = false;
  }

  if ( pHandle->OverCurrentFlag == true )
  {
    retVal |= MC_BREAK_IN;
    pHandle->OverCurrentFlag = false;
  }
  return retVal;
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

/************************ (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/

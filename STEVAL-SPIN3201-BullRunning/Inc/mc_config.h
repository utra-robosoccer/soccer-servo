/**
  ******************************************************************************
  * @file    mc_config.h 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Motor Control Subsystem components configuration and handler 
  *          structures declarations.
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
  
#ifndef __MC_CONFIG_H
#define __MC_CONFIG_H

#include "pid_regulator.h"
#include "revup_ctrl.h"
#include "speed_torq_ctrl.h"
#include "virtual_speed_sensor.h"
#include "ntc_temperature_sensor.h"
#include "pwm_curr_fdbk.h"
#include "r_divider_bus_voltage_sensor.h"
#include "virtual_bus_voltage_sensor.h"
#include "pqd_motor_power_measurement.h"
#include "user_interface.h"
#include "motor_control_protocol.h"
#include "r3_f0xx_pwm_curr_fdbk.h"
 
extern RevUpCtrl_Handle_t RevUpControlM1;
#include "ramp_ext_mngr.h"
#include "circle_limitation.h"
#include "sto_speed_pos_fdbk.h"
#include "sto_pll_speed_pos_fdbk.h"
#include "usart_frame_communication_protocol.h"

extern PID_Handle_t PIDSpeedHandle_M1;
extern PID_Handle_t PIDIqHandle_M1;
extern PID_Handle_t PIDIdHandle_M1;
extern NTC_Handle_t TempSensorParamsM1;

extern PWMC_R3_F0_Handle_t PWM_Handle_M1;

extern SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1;
extern PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM1;
extern PQD_MotorPowMeas_Handle_t *pPQD_MotorPowMeasM1; 
extern VirtualSpeedSensor_Handle_t VirtualSpeedSensorM1;
extern STO_Handle_t STO_M1;
extern STO_PLL_Handle_t STO_PLL_M1;
extern RDivider_Handle_t RealBusVoltageSensorParamsM1;
extern CircleLimitation_Handle_t CircleLimitationM1;

extern UI_Handle_t UI_Params;

extern RampExtMngr_Handle_t RampExtMngrHFParamsM1;
extern UFCP_Handle_t pUSART;
#define NBR_OF_MOTORS 1
#endif /* __MC_CONFIG_H */
/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/

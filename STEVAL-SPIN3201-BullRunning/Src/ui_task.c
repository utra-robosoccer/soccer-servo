/**
  ******************************************************************************
  * @file    ui_task.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implementes user interface tasks definition
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
/* Pre-compiler coherency check */

#include "ui_task.h"
#include "mc_config.h"
#include "ui_exported_functions.h"
#include "parameters_conversion.h"

#define OPT_DACX  0x20 /*!<Bit field indicating that the UI uses SPI AD7303 DAC.*/

DAC_UI_Handle_t * pDAC = MC_NULL;

MCP_Handle_t * pMCP = MC_NULL;
MCP_Handle_t MCP_UI_Params; 

static volatile uint16_t  bUITaskCounter;
static volatile uint16_t  bCOMTimeoutCounter;
static volatile uint16_t  bCOMATRTimeCounter = SERIALCOM_ATR_TIME_TICKS;

#define VECT_TABLE_BASE 0x08007E00

/* Setup the exported functions see UIExportedFunctions.h enum. */
void* const exportedFunctions[EF_UI_NUMBERS] =
{
  (void*)(&UI_GetReg),
  (void*)(&UI_ExecSpeedRamp),
  (void*)(&UI_SetReg),
  (void*)(&UI_ExecCmd),
  (void*)(&UI_GetSelectedMCConfig),
  (void*)(&UI_SetRevupData),
  (void*)(&UI_GetRevupData),
  (void*)MC_NULL,
  (void*)(&UI_SetCurrentReferences)
};

void UI_TaskInit( uint32_t* pUICfg, uint8_t bMCNum, MCI_Handle_t* pMCIList[],
                  MCT_Handle_t* pMCTList[],const char* s_fwVer )
{

    pMCP = &MCP_UI_Params;
    pMCP->_Super = UI_Params;

    UFCP_Init( & pUSART );
    MCP_Init(pMCP, (FCP_Handle_t *) & pUSART, & UFCP_Send, & UFCP_Receive, & UFCP_AbortReceive, pDAC, s_fwVer);
    UI_Init(&pMCP->_Super, bMCNum, pMCIList, pMCTList, pUICfg); /* Initialize UI and link MC components */

}

void UI_Scheduler(void)
{
  if(bUITaskCounter > 0u)
  {
    bUITaskCounter--;
  }

  if(bCOMTimeoutCounter > 1u)
  {
    bCOMTimeoutCounter--;
  }

  if(bCOMATRTimeCounter > 1u)
  {
    bCOMATRTimeCounter--;
  }
}

void UI_DACUpdate(uint8_t bMotorNbr)
{
}

void MC_SetDAC(DAC_Channel_t bChannel, MC_Protocol_REG_t bVariable)
{
}

void MC_SetUserDAC(DAC_UserChannel_t bUserChNumber, int16_t hValue)
{
}

MCP_Handle_t * GetMCP(void)
{
  return pMCP;
}

UI_Handle_t * GetDAC(void)
{
  return &pDAC->_Super;
}

bool UI_IdleTimeHasElapsed(void)
{
  bool retVal = false;
  if (bUITaskCounter == 0u)
  {
    retVal = true;
  }
  return (retVal);
}

void UI_SetIdleTime(uint16_t SysTickCount)
{
  bUITaskCounter = SysTickCount;
}

bool UI_SerialCommunicationTimeOutHasElapsed(void)
{
  bool retVal = false;
  if (bCOMTimeoutCounter == 1u)
  {
    bCOMTimeoutCounter = 0u;
    retVal = true;
  }
  return (retVal);
}

bool UI_SerialCommunicationATRTimeHasElapsed(void)
{
  bool retVal = false;
  if (bCOMATRTimeCounter == 1u)
  {
    bCOMATRTimeCounter = 0u;
    retVal = true;
  }
  return (retVal);
}

void UI_SerialCommunicationTimeOutStop(void)
{
  bCOMTimeoutCounter = 0u;
}

void UI_SerialCommunicationTimeOutStart(void)
{
  bCOMTimeoutCounter = SERIALCOM_TIMEOUT_OCCURENCE_TICKS;
}

/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/

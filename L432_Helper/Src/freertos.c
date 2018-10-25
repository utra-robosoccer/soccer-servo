/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
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
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "App/app_rx.h"
#include "App/app_tx.h"
#include "App/app_control.h"
#include "App/app_sensing.h"
#include "data_table.h"
#include "helpers.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId RXHandle;
uint32_t RXBuffer[ 512 ];
osStaticThreadDef_t RXControlBlock;
osThreadId TXHandle;
uint32_t TXBuffer[ 512 ];
osStaticThreadDef_t TXControlBlock;
osThreadId ControlHandle;
uint32_t ControlBuffer[ 128 ];
osStaticThreadDef_t ControlControlBlock;
osThreadId SensorHandle;
uint32_t SensorBuffer[ 128 ];
osStaticThreadDef_t SensorControlBlock;
osMessageQId commandQHandle;
uint8_t commandQBuffer[ 1 * sizeof( uint8_t ) ];
osStaticMessageQDef_t commandQControlBlock;
osMutexId dataTableLockHandle;
osStaticMutexDef_t dataTableLockControlBlock;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
extern void StartRX(void const * argument);
extern void StartTX(void const * argument);
extern void StartControlTask(void const * argument);
extern void StartSensorTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* definition and creation of dataTableLock */
  osMutexStaticDef(dataTableLock, &dataTableLockControlBlock);
  dataTableLockHandle = osMutexCreate(osMutex(dataTableLock));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of RX */
  osThreadStaticDef(RX, StartRX, osPriorityRealtime, 0, 512, RXBuffer, &RXControlBlock);
  RXHandle = osThreadCreate(osThread(RX), NULL);

  /* definition and creation of TX */
  osThreadStaticDef(TX, StartTX, osPriorityHigh, 0, 512, TXBuffer, &TXControlBlock);
  TXHandle = osThreadCreate(osThread(TX), NULL);

  /* definition and creation of Control */
  osThreadStaticDef(Control, StartControlTask, osPriorityAboveNormal, 0, 128, ControlBuffer, &ControlControlBlock);
  ControlHandle = osThreadCreate(osThread(Control), NULL);

  /* definition and creation of Sensor */
  osThreadStaticDef(Sensor, StartSensorTask, osPriorityNormal, 0, 128, SensorBuffer, &SensorControlBlock);
  SensorHandle = osThreadCreate(osThread(Sensor), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of commandQ */
  osMessageQStaticDef(commandQ, 1, uint8_t, commandQBuffer, &commandQControlBlock);
  commandQHandle = osMessageCreate(osMessageQ(commandQ), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */
void StartRX(void const * argument){
    // ONE-TIME APPLICATION INIT CODE SINCE THIS IS THE HIGHEST-PRIORITY TASK
    initDataTable();
    setOsStartFlag();

    bool statusIsOkay;
    for(;;){
        statusIsOkay = receive();

        if(statusIsOkay){
            processData();
        }
    }
}


void StartTX(void const * argument){
    uint8_t addressToRead;
    for(;;){
        while(
            xQueueReceive(
                commandQHandle,
                &addressToRead,
                pdMS_TO_TICKS(1)
            ) != pdTRUE
        );

        updateBufferContents(addressToRead);

        transmitBufferContents();
    }
}

void StartControlTask(void const * argument){
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    for(;;){
        // Service this task once every ms
        vTaskDelayUntil(
            &xLastWakeTime,
            pdMS_TO_TICKS(1)
        );

        controlUpdateStateVariables();

        controlUpdateSignals();
    }
}

void StartSensorTask(void const * argument){
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    for(;;){
        // Service this task once every ms
        vTaskDelayUntil(
            &xLastWakeTime,
            pdMS_TO_TICKS(1)
        );

        sensorUpdate();
    }
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

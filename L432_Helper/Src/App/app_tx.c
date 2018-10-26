/**
  *****************************************************************************
  * @file    app_tx.c
  * @author  Tyler
  *
  * @ingroup TX
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "App/app_tx.h"
#include "helpers.h"
#include "data_table.h"
#include "usart.h"
#include "tim.h"
#include "cmsis_os.h"




/******************************** Constants **********************************/
static const uint8_t NOTIFIED_FROM_TX_ISR = 0x80;
static const uint8_t NOTIFIED_FROM_TIMER_ISR = 0x20;




/****************************** Public Variables *****************************/
extern osThreadId TXHandle;




/***************************** Private Variables *****************************/
/** @brief Transmit buffer */
static volatile uint8_t buf[8] = {0xFF, 0xFF, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00};




/******************************** Functions **********************************/
/**
 * @brief Updates the transmit buffer to send back the contents requested by
 *        the master device
 * @param data Pointer to the container specifying what needs to be sent back
 */
void updateBufferContents(uint8_t addressToRead){
    uint16_t id;
    uint16_t pos;

    readDataTable(REG_ID, &id);
    readDataTable(REG_CURRENT_POSITION, &pos);

    buf[2] = (uint8_t)id;
    buf[5] = (uint8_t)(pos & 0xFF); // low byte
    buf[6] = (uint8_t)((pos >> 8) & 0xFF); // high byte
    buf[7] = Dynamixel_ComputeChecksum((uint8_t*)buf, sizeof(buf));
}

/**
 * @brief Sends the transmit buffer to the master device
 */
void transmitBufferContents(void){
    // Return delay time (about 100 us)
    HAL_TIM_Base_Start_IT(&htim16);
    waitUntilNotifiedOrTimeout(NOTIFIED_FROM_TIMER_ISR, 1);

    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)buf, 8);
    bool status = waitUntilNotifiedOrTimeout(NOTIFIED_FROM_TX_ISR, 1);
    if(!status){
        HAL_UART_AbortTransmit(&huart1);
    }
}

void txTimerEventHandler(void){
    if(osHasStarted()){
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        xTaskNotifyFromISR(
            TXHandle,
            NOTIFIED_FROM_TIMER_ISR,
            eSetBits,
            &xHigherPriorityTaskWoken
        );

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
 * @brief Callback function invoked upon finishing asynchronous UART TX
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if(osHasStarted()){
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        if(huart == &huart1){
            xTaskNotifyFromISR(
                TXHandle,
                NOTIFIED_FROM_TX_ISR,
                eSetBits,
                &xHigherPriorityTaskWoken
            );
        }

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/**
  *****************************************************************************
  * @file    tx_helpers.c
  * @author  Tyler
  *
  * @defgroup TX_Helpers TX Helpers
  * @brief Helpers for imitating motor transmissions
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "tx_helpers.h"
#include "helpers.h"
#include "data_table.h"
#include "usart.h"
#include "cmsis_os.h"




/******************************** Constants **********************************/
static const uint8_t NOTIFIED_FROM_TX_ISR = 0x80;




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
void update_buffer_contents(Data_t* data){
    uint16_t id;
    uint16_t pos;

    readDataTable(ID_IDX, &id);
    readDataTable(CURRENT_POSITION_IDX, &pos);

    buf[2] = (uint8_t)id;
    buf[5] = (pos & 0xFF); // low byte
    buf[6] = (pos >> 8) & 0xFF; // high byte
    buf[7] = Dynamixel_ComputeChecksum((uint8_t*)buf, sizeof(buf));
}

/**
 * @brief Sends the transmit buffer to the master device
 */
void transmit_buffer_contents(void){
    // TODO(tyler) replace this with a hardware timer of 100 us
    osDelay(pdMS_TO_TICKS(1));

    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)buf, 8);
    bool status = waitUntilNotifiedOrTimeout(NOTIFIED_FROM_TX_ISR, 1);
    if(!status){
        HAL_UART_AbortTransmit(&huart1);
    }
}

/**
 * @brief Callback function invoked upon finishing asynchronous UART TX
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
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

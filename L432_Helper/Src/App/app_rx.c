/**
  *****************************************************************************
  * @file    app_rx.c
  * @author  Tyler
  *
  * @ingroup RX
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "App/app_rx.h"

#include <string.h>

#include "data_table.h"
#include "helpers.h"
#include "usart.h"
#include "cmsis_os.h"




/******************************** Constants **********************************/
#define INST_WRITE_DATA      0x03
#define INST_READ_DATA       0x02
#define NOTIFIED_FROM_RX_ISR 0x40




/****************************** Public Variables *****************************/
extern osThreadId RXHandle;
extern osMessageQId commandQHandle;




/***************************** Private Variables *****************************/
/** @brief Receive buffer */
static volatile uint8_t buff[9] = {0};




/************************ Private Function Prototypes ************************/
/**
 * @brief Handles write requests
 */
static void processWriteDataInst(){
    HAL_UART_Receive_IT(&huart1, (uint8_t*)&buff[8], 1); // CHKSM
    bool statusIsOkay = waitUntilNotifiedOrTimeout(NOTIFIED_FROM_RX_ISR, 1);

    if(!statusIsOkay){
        return;
    }

    uint8_t computedChecksum = Dynamixel_ComputeChecksum(
        (uint8_t*)buff,
        sizeof(buff)
    );

    bool checksumIsValid = (computedChecksum == buff[8]);

    if(checksumIsValid){
        // Check the ID
        uint8_t id = buff[2];
        uint16_t myId;
        readDataTable(REG_ID, &myId);
        if((uint8_t)myId != id){
            // Do not proceed if the ID in the packet is not the ID of this
            // motor
            return;
        }

        // Write data into the data table based on the address in the packet
        uint8_t address = buff[5];
        writeDataTable(
                address,
            buff[6] | (buff[7] << 8)
        );
    }
}

/**
 * @brief Handles read requests
 */
static void processReadDataInst(){
    uint8_t computedChecksum = Dynamixel_ComputeChecksum((uint8_t*)buff, 8);
    bool checksumIsValid = (computedChecksum == buff[7]);

    uint8_t id = buff[2];
    if(checksumIsValid){
        uint16_t myId;
        readDataTable(REG_ID, &myId);
        if((uint8_t)myId != id){
            // Do not proceed if the ID in the packet is not the ID of this
            // motor
            return;
        }

        uint8_t address = buff[5];
        xQueueSend(commandQHandle, &address, pdMS_TO_TICKS(1));
    }
}




/******************************** Functions **********************************/
/**
 * @brief Initiates a DMA-based reception from the master device
 */
bool receive(){
    HAL_UART_Receive_DMA(&huart1, (uint8_t*)buff, 8);
    bool status = waitUntilNotifiedOrTimeout(NOTIFIED_FROM_RX_ISR, 2);

    if(!status){
       HAL_UART_AbortReceive(&huart1);
       memset((uint8_t*)buff, 0, sizeof(buff));
    }

    return status;
}

/**
 * @brief Process data received from master device. Invokes handlers
 *        with linkage internal
 */
void processData(){
    uint8_t instruction = buff[4];

    switch(instruction){
        case INST_WRITE_DATA:
            processWriteDataInst();
            break;
        case INST_READ_DATA:
            processReadDataInst();
            break;
        default:
            break;
    }
}
/**
 * @brief Callback function invoked upon finishing asynchronous UART RX
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if(osHasStarted()){
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        if(huart == &huart1){
            xTaskNotifyFromISR(
                RXHandle,
                NOTIFIED_FROM_RX_ISR,
                eSetBits,
                &xHigherPriorityTaskWoken
            );
        }

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

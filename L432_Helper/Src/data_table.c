/**
  *****************************************************************************
  * @file    data_table.c
  * @author  Tyler
  *
  * @defgroup Table Table
  * @brief Motor data table, essentially a big global memory for state
  *        variables which is accessed via APIs
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "data_table.h"
#include "cmsis_os.h"




/********************************** Types ************************************/
typedef enum{
    ID_IDX,
    GOAL_POSITION_IDX,
    CURRENT_POSITION_IDX,
    MAX_TABLE_IDX
}TableIdx_e;




/****************************** Public Variables *****************************/
extern osMutexId dataTableLockHandle;




/***************************** Private Variables *****************************/
/** @brief ID of the motor that is programmed in */
static const uint16_t MY_ID = 0x01;

/** @brief Master data table, analogous to a register map */
static uint16_t table[MAX_TABLE_IDX];




/***************************** Private Functions *****************************/
/**
 * @brief Maps Dynamixel register addresses to internal register addresses
 * @param dynamixelReg the Dynamixel register address
 * @return the internal register address
 */
static TableIdx_e map(uint8_t dynamixelReg){
    TableIdx_e retval = MAX_TABLE_IDX;
    switch(dynamixelReg){
        case REG_ID:
            retval = ID_IDX;
            break;
        case REG_GOAL_POSITION:
            retval = GOAL_POSITION_IDX;
            break;
        case REG_CURRENT_POSITION:
            retval = CURRENT_POSITION_IDX;
            break;
    }
    return retval;
}




/******************************** Functions **********************************/
void initDataTable(){
    table[ID_IDX] = MY_ID; // The ID for this motor;
}

bool writeDataTable(uint8_t reg, uint16_t data){
    TableIdx_e idx = map(reg);
    if(idx == MAX_TABLE_IDX){
        return false;
    }

    xSemaphoreTake(dataTableLockHandle, pdMS_TO_TICKS(1));
    table[idx] = data;
    xSemaphoreGive(dataTableLockHandle);

    return true;
}

bool readDataTable(uint8_t reg, uint16_t* data){
    TableIdx_e idx = map(reg);
    if(idx == MAX_TABLE_IDX){
        return false;
    }

    xSemaphoreTake(dataTableLockHandle, pdMS_TO_TICKS(1));
    *data = table[idx];
    xSemaphoreGive(dataTableLockHandle);

    return true;
}

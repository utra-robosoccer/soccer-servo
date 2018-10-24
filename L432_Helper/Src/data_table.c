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




/***************************** Private Variables *****************************/
static uint16_t table[MAX_TABLE_IDX];




/******************************** Functions **********************************/
/**
 * @brief wrapper for safely writing to the motor data table
 * @param idx The index of the table entry to be written to
 * @param data The goal data received from master, or the data acquired from
 *        sensors, to be written into the data table
 * @return true if successful, otherwise false
 */
bool writeDataTable(TableIdx_e idx, uint16_t data){
    if(idx >= MAX_TABLE_IDX){
        return false;
    }

    // TODO: protect with a mutex
    table[idx] = data;

    return true;
}

/**
 * @brief wrapper for safely reading from the motor data table
 * @param idx The index of the table entry to be read
 * @param[out] data Address where the requested table data should be copied
 * @return true if successful, otherwise false
 */
bool readDataTable(TableIdx_e idx, uint16_t* data){
    if(idx >= MAX_TABLE_IDX){
        return false;
    }

    // TODO: protect with a mutex
    *data = table[idx];

    return true;
}

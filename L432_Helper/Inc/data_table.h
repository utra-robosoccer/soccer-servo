/**
  *****************************************************************************
  * @file    data_table.h
  * @author  Tyler
  *
  * @defgroup Header
  * @ingroup  Table
  * @{
  *****************************************************************************
  */




#ifndef DATA_TABLE_H
#define DATA_TABLE_H




/********************************* Includes **********************************/
#include <stdint.h>
#include <stdbool.h>




/********************************** Types ************************************/
typedef enum{
    ID_IDX,
    GOAL_POSITION_IDX,
    CURRENT_POSITION_IDX,
    MAX_TABLE_IDX
}TableIdx_e;



/***************************** Function prototypes ***************************/
/**
 * @brief wrapper for safely writing to the motor data table
 * @param idx The index of the table entry to be written to
 * @param data The goal data received from master, or the data acquired from
 *        sensors, to be written into the data table
 * @return true if successful, otherwise false
 */
bool writeDataTable(TableIdx_e idx, uint16_t data);

/**
 * @brief wrapper for safely reading from the motor data table
 * @param idx The index of the table entry to be read
 * @param[out] data Address where the requested table data should be copied
 * @return true if successful, otherwise false
 */
bool readDataTable(TableIdx_e idx, uint16_t* data);




/**
 * @}
 */
/* end Header */

#endif /* DATA_TABLE_H */

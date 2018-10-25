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




/******************************** Constants **********************************/
#define REG_ID               0x03
#define REG_GOAL_POSITION    0x1E
#define REG_CURRENT_POSITION 0x24




/***************************** Function prototypes ***************************/
/**
 * @brief One-time initialization code which set predetermined values for
 *        some addresses in the table
 * @note In the future, can be extended to read config value from non-volatile
 *       memory
 */
void initDataTable();

/**
 * @brief wrapper for safely writing to the motor data table
 * @param idx The index of the table entry to be written to
 * @param data The goal data received from master, or the data acquired from
 *        sensors, to be written into the data table
 * @return true if successful, otherwise false
 */
bool writeDataTable(uint8_t reg, uint16_t data);

/**
 * @brief wrapper for safely reading from the motor data table
 * @param idx The index of the table entry to be read
 * @param[out] data Address where the requested table data should be copied
 * @return true if successful, otherwise false
 */
bool readDataTable(uint8_t reg, uint16_t* data);




/**
 * @}
 */
/* end Header */

#endif /* DATA_TABLE_H */

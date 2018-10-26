/**
  *****************************************************************************
  * @file    app_tx.h
  * @author  Tyler
  *
  * @defgroup TX
  * @brief Transmission helper functions
  * @{
  *****************************************************************************
  */




#ifndef APP_TX_H
#define APP_TX_H




/********************************* Includes **********************************/
#include <stdint.h>




/***************************** Function prototypes ***************************/
void updateBufferContents(uint8_t addressToRead);
void transmitBufferContents(void);
void txTimerEventHandler(void);




/**
 * @}
 */
/* end TX */

#endif /* APP_TX_H */

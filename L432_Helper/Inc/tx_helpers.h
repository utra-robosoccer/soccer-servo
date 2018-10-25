/**
  *****************************************************************************
  * @file    tx_helpers.h
  * @author  Tyler
  *
  * @defgroup Header
  * @ingroup  TX_Helpers
  * @{
  *****************************************************************************
  */




#ifndef TX_HELPERS_H
#define TX_HELPERS_H




/********************************* Includes **********************************/
#include "types.h"




/***************************** Function prototypes ***************************/
void updateBufferContents(void);
void transmitBufferContents(void);
void txTimerEventHandler(void);




/**
 * @}
 */
/* end Header */

#endif /* TX_HELPERS_H */

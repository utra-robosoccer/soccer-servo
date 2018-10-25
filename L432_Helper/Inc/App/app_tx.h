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




/***************************** Function prototypes ***************************/
void updateBufferContents(void);
void transmitBufferContents(void);
void txTimerEventHandler(void);




/**
 * @}
 */
/* end TX */

#endif /* APP_TX_H */

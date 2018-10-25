/**
  *****************************************************************************
  * @file    app_rx.h
  * @author  Tyler
  *
  * @defgroup RX
  * @brief Helpers for receiving data from the master device
  * @{
  *****************************************************************************
  */




#ifndef APP_RX_H
#define APP_RX_H




/********************************* Includes **********************************/
#include <stdbool.h>




/***************************** Function prototypes ***************************/
bool receive();
void processData();




/**
 * @}
 */
/* end - RX */

#endif /* APP_RX_H */

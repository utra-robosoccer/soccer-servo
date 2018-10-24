/**
  *****************************************************************************
  * @file    control.c
  * @author  Tyler
  *
  * @defgroup Control Control
  * @brief Control routines
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "control.h"
#include "types.h"
#include "data_table.h"




/***************************** Private Variables *****************************/
static uint16_t goalPosition;
static uint16_t currentPosition;




/******************************** Functions **********************************/
void controlUpdateStateVariables(){
    readDataTable(GOAL_POSITION_IDX, &goalPosition);
    readDataTable(CURRENT_POSITION_IDX, &currentPosition);
    // TODO: etc...
}

void controlUpdateSignals(){
    // TODO: Implement control algorithm here
}

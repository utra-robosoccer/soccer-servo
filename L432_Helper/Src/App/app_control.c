/**
  *****************************************************************************
  * @file    app_control.c
  * @author  Tyler
  *
  * @defgroup Control Control
  * @brief Control routines
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "App/app_control.h"
#include "data_table.h"




/***************************** Private Variables *****************************/
static uint16_t goalPosition;
static uint16_t currentPosition;




/******************************** Functions **********************************/
void controlUpdateStateVariables(){
    readDataTable(REG_GOAL_POSITION, &goalPosition);
    readDataTable(REG_CURRENT_POSITION, &currentPosition);
    // TODO: etc...
}

void controlUpdateSignals(){
    // TODO: Implement control algorithm here
}

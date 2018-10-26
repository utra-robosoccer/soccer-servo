/**
  *****************************************************************************
  * @file    app_control.c
  * @author  Tyler
  *
  * @ingroup Control
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "App/app_control.h"
#include "data_table.h"
#include "tim.h"




/***************************** Private Variables *****************************/
static uint16_t goalPosition;
static uint16_t currentPosition;




/******************************** Constants **********************************/
/**
 * @brief AX12As have 10 bits to represent values between 0 and 300 degrees.
 *        This servo operates between 0 and 180 degrees.
 *
 *        (1023 / 300 degrees) * 180 degrees => 613
 *
 *        Thus, the max position of 180 is encoded as 613
 */
const uint16_t POS_MAX = 613;
const uint16_t POS_MAX_DUTY_CYCLE = 47500; // 180 degrees
const uint16_t POS_MIN_DUTY_CYCLE = 12500; // 0 degrees




/***************************** Private Functions *****************************/
/**
 * @brief Low-level function that talks with the hardware to set the position
 */
void ll_setPosition(uint16_t position){
    float fpos = position;
    float FPOS_MAX = POS_MAX;
    float FPOS_MAX_DUTY_CYCLE = POS_MAX_DUTY_CYCLE;
    float FPOS_MIN_DUTY_CYCLE = POS_MIN_DUTY_CYCLE;

    uint16_t tim = (fpos / FPOS_MAX) * (FPOS_MAX_DUTY_CYCLE - FPOS_MIN_DUTY_CYCLE) + FPOS_MIN_DUTY_CYCLE;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, tim);
}




/******************************** Functions **********************************/
void controlInit(){
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, POS_MIN_DUTY_CYCLE);
}

void controlUpdateStateVariables(){
    readDataTable(REG_GOAL_POSITION, &goalPosition);
    readDataTable(REG_CURRENT_POSITION, &currentPosition);
    // TODO: etc...
}

void controlUpdateSignals(){
    // TODO: Implement control algorithm here
    ll_setPosition(goalPosition);
}

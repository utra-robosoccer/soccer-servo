/**
  *****************************************************************************
  * @file    app_sensing.c
  * @author  Tyler
  *
  * @ingroup Sensing
  *****************************************************************************
  */




/********************************* Includes **********************************/
#include "App/app_sensing.h"
#include "data_table.h"
#include "lfsr.h"




/***************************** Public Functions ******************************/
void sensorUpdate(void){
    // TODO: Replace with real sensor acquisition implementation
    uint16_t data;
    readDataTable(REG_GOAL_POSITION, &data);

    static uint32_t lfsr = 0x2F; // Seed value
    static uint32_t polynomial = POLY_MASK_PERIOD_63;

    // Add statistical noise to the data
    lfsr_update(&lfsr, polynomial);
    int8_t noise = (lfsr >> 2) - 8;
    if(data + noise > 0){
        data = data + noise;
    }
    writeDataTable(REG_CURRENT_POSITION, data);
}

/****************************************
 * Name: consts.h
 * Purpose: regroup various constants used throughout the code
 *
 * Created by Andrea Silvestroni and Pietro Argnani - 2023
 ***************************************/

#ifndef SOURCE_CONSTS_H_
#define SOURCE_CONSTS_H_

// Motor step pins
#define STEP_PIN_1 CYBSP_D8 // D8
#define STEP_PIN_2 CYBSP_D9 // D9

// Motor direction pins
#define DIR_PIN_1 CYBSP_A6 // A6
#define DIR_PIN_2 CYBSP_A7 // A7

// Limit switch controls
#define HOLE_LIMIT_PIN CYBSP_D10 // D10
#define END_LIMIT_PIN CYBSP_D11  // D11

// Radar motion signal (TD)
#define RADAR_MOTION_PIN CYBSP_D12 // D12

#define MOTORS_COUNT 2

// Number of ms for each shake motion
#define SHAKE_DELAY_QUICK 150
#define SHAKE_DELAY_LONG 5000

// Number of ms to wait after movement is detected before
// starting the cleaning process
#define CLEANING_TIMER_DELAY_MS 120000 // 2 minutes
// Clock frequency for the cleaning timer
#define CLEANING_TIMER_FREQ_HZ 10000

#endif
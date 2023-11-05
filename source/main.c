/****************************************
 * Name: main.c
 * Purpose: main file of the firmware
 *
 * Created by Andrea Silvestroni and Pietro Argnani - 2023
 ***************************************/

/*******************************************************************************
 * Header Files
 *******************************************************************************/
#include "stdlib.h"
#include "cybsp.h"
#include "cyhal.h"
#include "cy_retarget_io.h"
#include <inttypes.h>

#include "consts.h"
#include "limit_switch.h"
#include "motor.h"

/*******************************************************************************
 * Types
 *******************************************************************************/
typedef enum
{
    LITTERBOX_IDLE,          // Base state, nothing is happening
    LITTERBOX_AWAKE,         // Movement has been detected, become alert
    LITTERBOX_TIMER_RUNNING, // No movement for a while, the cleaning timer is running
    LITTERBOX_FILTERING,     // The filtering process is running
    LITTERBOX_DUMPING,       // Contents are being dumped from the litterbox
    LITTERBOX_RETURN,        // The barrel is returning to the home position
    LITTERBOX_LEVELING,      // Litter is being leveled by shaking it

    LITTERBOX_EMERGENCY,
} litter_box_state_t;

/*******************************************************************************
 * Global Variables
 *******************************************************************************/
motor_data_t *motors;             // Array of motor objects to be controlled
litter_box_state_t current_state; // Current state of the system
cyhal_timer_t cleaning_timer;     // Timer to start the cleaning process

/*******************************************************************************
 * Utility functions
 *******************************************************************************/

/*******************************************************************************
 * Function Name: handle_error
 ********************************************************************************
 * Summary:
 *  User defined error handling function.
 *
 * Parameters:
 *  status - status for evaluation.
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void handle_error(cy_rslt_t status)
{
    if (CY_RSLT_SUCCESS != status)
    {
        /* Halt the CPU while debugging */
        CY_ASSERT(0);
    }
}

/*******************************************************************************
 * Function Name: check_status
 ********************************************************************************
 * Summary:
 *  Prints the message and waits forever when an error occurs.
 *
 * Parameters:
 *  message - message to print if status is non-zero.
 *  status - status for evaluation.
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void check_status(char *message, cy_rslt_t status)
{
    if (CY_RSLT_SUCCESS != status)
    {
        printf("\r\n=====================================================\r\n");
        printf("\nFAIL: %s\r\n", message);
        printf("Error Code: 0x%08" PRIX32 "\n", status);
        printf("\r\n=====================================================\r\n");

        while (true)
            ;
    }
}

/*******************************************************************************
 * Function Name: set_state
 ********************************************************************************
 * Summary:
 *  Updates the current state of the system and prints it
 *
 * Parameters:
 *  new_state - the new state for the system
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void set_state(litter_box_state_t new_state)
{
    printf("New state: %d\r\n", new_state);
    current_state = new_state;
}

/*******************************************************************************
 * Function Name: shake
 ********************************************************************************
 * Summary:
 *  Makes the barrel shake by alternating the rotation direction of the motors
 *
 * Parameters:
 *  speed - the speed of the motors
 *  delay - the duration of each motor sway
 *  shakes_count - the number of sways
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void shake(uint32_t speed, uint16_t delay, uint16_t shakes_count)
{
    // Start all motors
    for (size_t i = 0; i < MOTORS_COUNT; i++)
    {
        motor_start(&motors[i], speed);
    }

    // Shake shakes_count times
    for (uint16_t i = 0; i < shakes_count; i++)
    {
        // Set each motor to rotate clockwise;
        for (size_t i = 0; i < MOTORS_COUNT; i++)
        {
            motor_set_rotation_direction(&motors[i], MOTOR_ROT_CLOCK);
        }

        cyhal_system_delay_ms(delay);

        // Invert rotation direction
        for (size_t i = 0; i < MOTORS_COUNT; i++)
        {
            motor_set_rotation_direction(&motors[i], MOTOR_ROT_COUNTERCLOCK);
        }

        cyhal_system_delay_ms(delay);
    }

    // Stop all motors
    for (size_t i = 0; i < MOTORS_COUNT; i++)
    {
        motor_stop(&motors[i]);
    }
}

/*******************************************************************************
 * State functions
 *******************************************************************************/

/*******************************************************************************
 * Function Name: filter_litter
 ********************************************************************************
 * Summary:
 *  Beginning of the litter filtering process:
 *  - starts the motors with counter-clockwise rotation and minimum speed
 *  - creates and starts two timers for periodic speed variation, easing the litter into the filtering grate
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void filter_litter()
{
    if (current_state != LITTERBOX_IDLE && current_state != LITTERBOX_TIMER_RUNNING)
    {
        return;
    }

    set_state(LITTERBOX_FILTERING);

    printf("Starting litter filtering...\r\n");

    // Start all motors
    for (size_t i = 0; i < MOTORS_COUNT; i++)
    {
        motor_set_rotation_direction(&motors[i], MOTOR_ROT_COUNTERCLOCK);
        motor_start(&motors[i], MOTOR_SPEED_MIN);
    }
}

/*******************************************************************************
 * Function Name: dump_contents
 ********************************************************************************
 * Summary:
 *  Forces the droppings out of the dumping hole:
 *  - fast speed variations on the motors allow the barrel to be "shaken", forcing contents close to/into the hole
 *  - slow speed variations on the motors allow ease the contents into the hole
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void dump_contents()
{
    if (current_state != LITTERBOX_FILTERING)
    {
        return;
    }

    set_state(LITTERBOX_DUMPING);

    printf("Dumping contents...\r\n");

    // Slightly rotate back to allow shaking
    for (size_t i = 0; i < MOTORS_COUNT; i++)
    {
        motor_set_rotation_direction(&motors[i], MOTOR_ROT_CLOCK);
        motor_start(&motors[i], MOTOR_SPEED_MIN);
    }

    cyhal_system_delay_ms(1000);

    shake(MOTOR_SPEED_MAX, SHAKE_DELAY_QUICK, 30);

    reset_position();
}

/*******************************************************************************
 * Function Name: reset_position
 ********************************************************************************
 * Summary:
 * Resets the barrel position to the original location
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void reset_position()
{
    if (current_state != LITTERBOX_DUMPING)
    {
        return;
    }

    set_state(LITTERBOX_RETURN);

    printf("Returning to home position...\r\n");

    for (size_t i = 0; i < MOTORS_COUNT; i++)
    {
        motor_set_rotation_direction(&motors[i], MOTOR_ROT_CLOCK);
        motor_start(&motors[i], MOTOR_SPEED_MIN + 50u);
    }
}

/*******************************************************************************
 * Function Name: level_litter
 ********************************************************************************
 * Summary:
 * Levels the cat litter by shaking the barrel
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void level_litter()
{
    if (current_state != LITTERBOX_RETURN)
    {
        return;
    }

    set_state(LITTERBOX_LEVELING);
    printf("Leveling litter...\r\n");

    // Slightly rotate back to allow shaking
    for (size_t i = 0; i < MOTORS_COUNT; i++)
    {
        motor_set_rotation_direction(&motors[i], MOTOR_ROT_COUNTERCLOCK);
        motor_start(&motors[i], MOTOR_SPEED_MIN);
    }

    cyhal_system_delay_ms(2000);

    shake(MOTOR_SPEED_MAX, SHAKE_DELAY_QUICK, 30);

    printf("Cleaning completed!\r\n");
    set_state(LITTERBOX_IDLE);
}

/*******************************************************************************
 * Interrupt callbacks
 *******************************************************************************/

/*******************************************************************************
 * Function Name: cb_motion_detected
 ********************************************************************************
 * Summary:
 *  Callback for radar shield interrupts when movement is detected.
 *  Resets the cleaning timer if it's currently running
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void cb_motion_detected()
{
    switch (current_state)
    {
    case LITTERBOX_IDLE:
    case LITTERBOX_TIMER_RUNNING:
        break;
    default:
        return;
    }

    if (current_state != LITTERBOX_IDLE && current_state != LITTERBOX_TIMER_RUNNING)
    {
        return;
    }

    printf("Motion detected, resetting cleaning timer...\r\n");

    set_state(LITTERBOX_AWAKE);

    cy_rslt_t result = cyhal_timer_stop(&cleaning_timer);
    check_status("cyhal_timer_stop failed with error code", result);

    result = cyhal_timer_reset(&cleaning_timer);
    check_status("cyhal_timer_reset failed with error code", result);

    printf("Cleaning timer reset\r\n");
}

/*******************************************************************************
 * Function Name: cb_no_motion
 ********************************************************************************
 * Summary:
 *  Callback for radar shield interrupts when no movement is detected.
 *  Starts a timer to trigger the beginning of the cleaning process.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void cb_no_motion()
{
    if (current_state != LITTERBOX_AWAKE)
    {
        return;
    }

    printf("No motion detected, starting cleaning timer...\r\n");

    cy_rslt_t result = cyhal_timer_start(&cleaning_timer);
    check_status("cyhal_timer_start failed with error code", result);

    set_state(LITTERBOX_TIMER_RUNNING);

    printf("Cleaning timer started\r\n");
}

/*******************************************************************************
 * Function Name: cb_timer_ended
 ********************************************************************************
 * Summary:
 *  Callback for the end of the cleaning timer.
 *  Starts the cleaning process.
 *
 * Parameters:
 *  callback_arg - argument for the callback
 *  event - timer event for the callback
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void cb_timer_ended(void *callback_arg, cyhal_timer_event_t event)
{
    if (current_state != LITTERBOX_TIMER_RUNNING)
    {
        printf("cb_timer_ended called in the wrong state (???)\r\n");
        return;
    }

    printf("Cleaning timer completed, starting cleaning process...\r\n");

    filter_litter();

    printf("Cleaning process started\r\n");
}

/*******************************************************************************
 * Function Name: cb_radar_motion
 ********************************************************************************
 * Summary:
 *  Callback for radar shield interrupts, determines which callback to call depending
 *  on the event (IRQ rise or fall)
 *
 * Parameters:
 *  callback_arg - argument for the callback
 *  event - event for the callback, allows identifying the kind of IRQ
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void cb_radar_motion(void *callback_arg, cyhal_gpio_event_t event)
{
    // Radar TD drives low, 0 = motion
    if (event == CYHAL_GPIO_IRQ_FALL)
    {
        cb_motion_detected();
    }

    // 1 = no motion
    if (event == CYHAL_GPIO_IRQ_RISE)
    {
        cb_no_motion();
    }
}

// Callback data for the radar GPIO, needs to reside outside of the stack
cyhal_gpio_callback_data_t radar_motion_callback_data;

/*******************************************************************************
 * Setup functions
 *******************************************************************************/

/*******************************************************************************
 * Function Name: gpio_init_radar
 ********************************************************************************
 * Summary:
 *  Initializes the GPIO pin for the radar shield
 *
 * Parameters:
 *  void
 *
 * Return:
 *  cy_rslt_t - result identifier
 *
 *******************************************************************************/
cy_rslt_t gpio_init_radar()
{
    cy_rslt_t result;

    result = cyhal_gpio_init(RADAR_MOTION_PIN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, true);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    radar_motion_callback_data.callback = &cb_radar_motion;

    cyhal_gpio_register_callback(RADAR_MOTION_PIN, &radar_motion_callback_data);
    cyhal_gpio_enable_event(RADAR_MOTION_PIN, CYHAL_GPIO_IRQ_BOTH, 0, true);

    return 0;
}

/*******************************************************************************
 * Function Name: timer_init
 ********************************************************************************
 * Summary:
 *  Initializes the timer object for the cleaning timer
 *
 * Parameters:
 *  void
 *
 * Return:
 *  cy_rslt_t - result identifier
 *
 *******************************************************************************/
cy_rslt_t timer_init()
{
    cy_rslt_t result;

    cyhal_timer_cfg_t cleaning_timer_cfg = {
        .compare_value = 0,
        .period = CLEANING_TIMER_DELAY_MS - 1,
        .value = 0,
        .direction = CYHAL_TIMER_DIR_UP,
        .is_compare = false,
        .is_continuous = false,
    };

    result = cyhal_timer_init(&cleaning_timer, NC, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    result = cyhal_timer_configure(&cleaning_timer, &cleaning_timer_cfg);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    result = cyhal_timer_set_frequency(&cleaning_timer, CLEANING_TIMER_FREQ_HZ);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    /* Assign the ISR to execute on timer interrupt */
    cyhal_timer_register_callback(&cleaning_timer, cb_timer_ended, NULL);

    /* Set the event on which timer interrupt occurs and enable it */
    cyhal_timer_enable_event(&cleaning_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT, CYHAL_ISR_PRIORITY_DEFAULT, true);

    return result;
}

int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    handle_error(result);

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize the retarget-io to use the debug UART port */
    result = cy_retarget_io_init(
        CYBSP_DEBUG_UART_TX,
        CYBSP_DEBUG_UART_RX,
        CY_RETARGET_IO_BAUDRATE);

    handle_error(result);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("******************\r\n"
           "Self-Cleaning Cat Litter Box\r\n"
           "\tby Andrea Silvestroni & Pietro Argnani\r\n"
           "****************** \r\n\n");

    // Initialize the motors objects
    motors = malloc(sizeof(motor_data_t) * MOTORS_COUNT);

    result = motor_init(&motors[0], STEP_PIN_1, DIR_PIN_1);
    check_status("motor_init failed for motor 0 with error code", result);

    result = motor_init(&motors[1], STEP_PIN_2, DIR_PIN_2);
    check_status("motor_init failed for motor 1 with error code", result);

    // Initialize the limit switch objects
    limit_switch_data_t hole_limit_switch;
    limit_switch_data_t end_limit_switch;

    result = limit_switch_init(&hole_limit_switch, HOLE_LIMIT_PIN, &dump_contents);
    check_status("limit_switch_init failed for hole switch with error code", result);

    result = limit_switch_init(&end_limit_switch, END_LIMIT_PIN, &level_litter);
    check_status("limit_switch_init failed for end switch with error code", result);

    // Initialize the radar shield GPIO
    result = gpio_init_radar();
    check_status("gpio_init_radar failed with error code", result);

    // Initialize the cleaning timer
    result = timer_init();
    check_status("timer_init failed with error code", result);

    set_state(LITTERBOX_IDLE);

    for (;;)
    {
        /* Put the CPU into sleep mode to save power */
        cyhal_syspm_sleep();
    }
}

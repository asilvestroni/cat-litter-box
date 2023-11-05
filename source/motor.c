/****************************************
 * Name: motor.c
 * Purpose: provide a simple API for interacting with stepper motors
 * using the DRV8825 driver
 *
 * Created by Andrea Silvestroni and Pietro Argnani - 2023
 ***************************************/

#include "motor.h"

/*******************************************************************************
 * Function Name: motor_init
 ********************************************************************************
 * Summary:
 *  Initialize a motor object, specifying the step and direction pins
 *
 * Parameters:
 *  motor - the motor object to be initialized
 *  step_pin - the pin to be used for motor stepping
 *  dir_pin - the pin to be used for motor rotation direction changes
 *
 * Return:
 *  cy_rslt_t - result identifier
 *
 *******************************************************************************/
cy_rslt_t motor_init(motor_data_t *motor, cyhal_gpio_t step_pin, cyhal_gpio_t dir_pin)
{
    cy_rslt_t result;
    cyhal_pwm_t pwm_step_pin;

    result = cyhal_pwm_init(&pwm_step_pin, step_pin, NULL);

    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    result = cyhal_gpio_init(dir_pin, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);

    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    motor->pwm_step_pin = pwm_step_pin;
    motor->gpio_dir_pin = dir_pin;
    motor->current_rotation_direction = MOTOR_ROT_CLOCK;
    motor->current_motor_state = MOTOR_STATE_OFF;

    return 0;
}

/*******************************************************************************
 * Function Name: motor_stop
 ********************************************************************************
 * Summary:
 *  Stop the rotation of a motor object
 *
 * Parameters:
 *  motor - the motor object to be stopped
 *
 * Return:
 *  cy_rslt_t - result identifier
 *
 *******************************************************************************/
cy_rslt_t motor_stop(motor_data_t *motor)
{
    return cyhal_pwm_stop(&motor->pwm_step_pin);
}

/*******************************************************************************
 * Function Name: motor_start
 ********************************************************************************
 * Summary:
 *  Start the rotation of a motor object specifying its speed
 *
 * Parameters:
 *  motor - the motor object to be started
 *  speed - the new speed of the motor
 *
 * Return:
 *  cy_rslt_t - result identifier
 *
 *******************************************************************************/
cy_rslt_t motor_start(motor_data_t *motor, uint16_t speed)
{
    cy_rslt_t result;

    /* Set the PWM pin frequency and duty cycle */
    result = cyhal_pwm_set_duty_cycle(&motor->pwm_step_pin, PWM_MOTOR_DUTY_CYCLE, speed);
    printf("Starting motor on pin %d with speed %d\r\n", motor->pwm_step_pin.pin, speed);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    /* Set the rotation direction */
    cyhal_gpio_write(motor->gpio_dir_pin, motor->current_rotation_direction == MOTOR_ROT_CLOCK ? false : true);

    /* Start the PWM */
    result = cyhal_pwm_start(&motor->pwm_step_pin);

    return result;
}

/*******************************************************************************
 * Function Name: motor_set_rotation_direction
 ********************************************************************************
 * Summary:
 *  Update the direction of rotation for a motor object
 *
 * Parameters:
 *  motor - the motor object to be updated
 *  direction - the new rotation direction of the motor
 *
 * Return:
 *  cy_rslt_t - result identifier
 *
 *******************************************************************************/
cy_rslt_t motor_set_rotation_direction(motor_data_t *motor, motor_rot_dir_t direction)
{
    motor->current_rotation_direction = direction;
    cyhal_gpio_write(motor->gpio_dir_pin, motor->current_rotation_direction == MOTOR_ROT_CLOCK ? false : true);
    return 0;
}

/****************************************
 * Name: motor.h
 * Purpose: provide a simple API for interacting with stepper motors
 * using the DRV8825 driver
 *
 * Created by Andrea Silvestroni and Pietro Argnani - 2023
 ***************************************/

#ifndef SOURCE_MOTOR_H_
#define SOURCE_MOTOR_H_

#include "stdio.h"
#include "cyhal.h"

#include "consts.h"

// Motor PWM duty cycle = 100%
#define PWM_MOTOR_DUTY_CYCLE 100.0f

// Allowed frequency for maximum speed
#define MOTOR_SPEED_MAX 512u

// Allowed frequency for minimum speed
#define MOTOR_SPEED_MIN 85u

// Type for the possible rotation directions of the stepper motor
typedef enum
{
    MOTOR_ROT_CLOCK,
    MOTOR_ROT_COUNTERCLOCK,
} motor_rot_dir_t;

// Type for the possible states of the stepper motor
typedef enum
{
    MOTOR_STATE_ON,
    MOTOR_STATE_OFF,
} motor_state_t;

typedef struct {
    cyhal_pwm_t pwm_step_pin;
    cyhal_gpio_t gpio_dir_pin;
    motor_state_t current_motor_state;
    motor_rot_dir_t current_rotation_direction;
} motor_data_t;

cy_rslt_t motor_init(motor_data_t *motor, cyhal_gpio_t step_pin, cyhal_gpio_t dir_pin);
cy_rslt_t motor_stop(motor_data_t *motor);
cy_rslt_t motor_start(motor_data_t *motor, uint16_t speed);
cy_rslt_t motor_set_rotation_direction(motor_data_t *motor, motor_rot_dir_t direction);

#endif
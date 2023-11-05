/****************************************
 * Name: limit_switch.h
 * Purpose: provide a simple API for limit switch components
 *
 * Created by Andrea Silvestroni and Pietro Argnani - 2023
 ***************************************/

#ifndef SOURCE_LIMIT_SWITCH_H_
#define SOURCE_LIMIT_SWITCH_H_

#include "cyhal.h"

typedef struct
{
    uint32_t gpio_control_pin;
    cyhal_gpio_callback_data_t callback;
} limit_switch_data_t;

cy_rslt_t limit_switch_init(limit_switch_data_t *limit_switch, cyhal_gpio_t control_pin, void *callback);
cy_rslt_t limit_switch_update_pin(limit_switch_data_t *limit_switch, cyhal_gpio_t control_pin);
cy_rslt_t limit_switch_update_callback(limit_switch_data_t *limit_switch, void *callback);

#endif
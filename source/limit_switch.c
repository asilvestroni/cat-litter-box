/****************************************
 * Name: limit_switch.c
 * Purpose: provide a simple API for limit switch components
 *
 * Created by Andrea Silvestroni and Pietro Argnani - 2023
 ***************************************/

#include "limit_switch.h"

/*******************************************************************************
 * Function Name: limit_switch_init
 ********************************************************************************
 * Summary:
 *  Initializes a limit switch object by registering the callback to be called on the pin interrupts
 *
 * Parameters:
 *  limit_switch - the limit switch object to initialize
 *  control_pin - the pin on which GPIO interrupts will be received
 *  callback - a pointer to the callback function to invoke in GPIO interrupts
 *
 * Return:
 *  cy_rslt_t - result identifier
 *
 *******************************************************************************/
cy_rslt_t limit_switch_init(limit_switch_data_t *limit_switch, cyhal_gpio_t control_pin, void *callback)
{
    cy_rslt_t result;
    result = cyhal_gpio_init(control_pin, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);

    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    limit_switch->callback.callback = callback;

    cyhal_gpio_register_callback(control_pin, &limit_switch->callback);
    cyhal_gpio_enable_event(control_pin, CYHAL_GPIO_IRQ_FALL, CYHAL_ISR_PRIORITY_DEFAULT, true);

    limit_switch->gpio_control_pin = control_pin;

    return 0;
}

/*******************************************************************************
 * Function Name: limit_switch_update_pin
 ********************************************************************************
 * Summary:
 *  Changes the pin associated with a limit switch object
 *
 * Parameters:
 *  limit_switch - the limit switch object to update
 *  control_pin - the new pin to associate
 *
 * Return:
 *  cy_rslt_t - result identifier
 *
 *******************************************************************************/
cy_rslt_t limit_switch_update_pin(limit_switch_data_t *limit_switch, cyhal_gpio_t control_pin)
{
    if (limit_switch->gpio_control_pin == control_pin)
    {
        return 0;
    }

    // Remove the callback from the old pin
    cyhal_gpio_enable_event(limit_switch->gpio_control_pin, CYHAL_GPIO_IRQ_FALL, CYHAL_ISR_PRIORITY_DEFAULT, false);
    cyhal_gpio_register_callback(limit_switch->gpio_control_pin, NULL);
    cyhal_gpio_free(limit_switch->gpio_control_pin);

    // Sets the callback for the new pin
    cy_rslt_t result;
    result = cyhal_gpio_init(control_pin, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);

    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    cyhal_gpio_register_callback(control_pin, &limit_switch->callback);
    cyhal_gpio_enable_event(control_pin, CYHAL_GPIO_IRQ_FALL, CYHAL_ISR_PRIORITY_DEFAULT, true);

    limit_switch->gpio_control_pin = control_pin;

    return 0;
}

/*******************************************************************************
 * Function Name: limit_switch_update_callback
 ********************************************************************************
 * Summary:
 *  Changes the callback associated with the pin on a limit switch object
 *
 * Parameters:
 *  limit_switch - the limit switch object to update
 *  callback - the new callback to be invoked
 *
 * Return:
 *  cy_rslt_t - result identifier
 *
 *******************************************************************************/
cy_rslt_t limit_switch_update_callback(limit_switch_data_t *limit_switch, void *callback)
{
    if (limit_switch->callback.callback == callback)
    {
        return 0;
    }

    // Removes the old callback
    cyhal_gpio_enable_event(limit_switch->gpio_control_pin, CYHAL_GPIO_IRQ_FALL, CYHAL_ISR_PRIORITY_DEFAULT, false);
    cyhal_gpio_register_callback(limit_switch->gpio_control_pin, NULL);

    // Adds the new callback
    limit_switch->callback.callback = callback;

    cyhal_gpio_register_callback(limit_switch->gpio_control_pin, &limit_switch->callback);
    cyhal_gpio_enable_event(limit_switch->gpio_control_pin, CYHAL_GPIO_IRQ_FALL, CYHAL_ISR_PRIORITY_DEFAULT, true);

    return 0;
}
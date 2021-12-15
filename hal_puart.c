/*
 * Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */


 /** @file hal_puart.c
 */

#include "wiced_bt_dev.h"
#include "sparcommon.h"
#include "wiced_hal_puart.h"
#include "wiced_platform.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_gpio.h"
#include "wiced_bt_stack.h"
#include "wiced_timer.h"

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/
 static wiced_result_t puart_app_management_cback(wiced_bt_management_evt_t event,
                                  wiced_bt_management_evt_data_t *p_event_data);

 static void test_puart_driver(void);

#if PUART_RTS_CTS_FLOW
#if !defined(CYW20706A2)
#error "Puart with RTS/CTS flow control demonstrated for CYW20706A2 device only."
#endif
static wiced_timer_t hal_puart_flow_timer;
#endif

/******************************************************************************
 *                                Function Definitions
 ******************************************************************************/
/**
 Function name:
 application_start

 Function Description:
 @brief    Starting point of your application. Entry point to the application.
           Set device configuration and start BT stack initialization.
           The actual application initialization will happen when stack reports
           that BT device is ready.

 @param void

 @return void
 */
APPLICATION_START ( )
{
    wiced_result_t result = WICED_BT_SUCCESS;

    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);
#ifdef CYW20706A2
    wiced_hal_puart_init();
  #if PUART_RTS_CTS_FLOW
    // special setup to enable flow control pins
    platform_puart_flow_control_pin_init();
  #endif
    // Please see the User Documentation to reference the valid pins.
    // CTS and RTS are defined non-zero #if PUART_RTS_CTS_FLOW, see wiced_platform.h
    if(!wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, WICED_PUART_CTS, WICED_PUART_RTS))
    {
        WICED_BT_TRACE("wiced_hal_puart_select_uart_pads failed!!\n");
    }
#else
    wiced_hal_puart_configuration(115200, PARITY_NONE, STOP_BIT_1);
#endif
    WICED_BT_TRACE("*************Starting PUART Application**********\n\r");

    /* Register BT stack callback*/
    result = wiced_bt_stack_init(puart_app_management_cback, NULL, NULL);
    if(WICED_BT_SUCCESS != result)
    {
        WICED_BT_TRACE("Stack Initialization Failed!!\n\r");
    }
}

/**
 Function Name:
 puart_app_management_cback

 Function Description:
 @brief  Callback function that will be invoked by application_start()

 @param  event           Bluetooth management event type
 @param  p_event_data    Pointer to the the bluetooth management event data

 @return        status of the callback function
 */
wiced_result_t
puart_app_management_cback(wiced_bt_management_evt_t event,
                                  wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_SUCCESS;

    switch(event)
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            test_puart_driver();
            break;
        default:
            break;
    }
    return result;
}

/**
 Function Name:
 puart_rx_interrupt_callback

 Function Description:
 @brief  Interrupt routine called when PUART has data to be read

 @param  unused    Not used

 @return        void
 */
void puart_rx_interrupt_callback(void* unused)
{
    /* There can be at most 16 bytes in the HW FIFO.*/
    uint8_t  readbyte=0;

    /* Drain rx and send to tx. We don't want to wait on tx and assume it won't be overrun (because RTS/CTS) */
    while (wiced_hal_puart_rx_fifo_not_empty() && wiced_hal_puart_read(&readbyte))
    {
        readbyte += 1;
        // this will wait for tx fifo empty before queueing byte
        wiced_hal_puart_synchronous_write(&readbyte,1);
    }
    #if !PUART_RTS_CTS_FLOW
    wiced_hal_puart_reset_puart_interrupt( );
    #endif
}

#if PUART_RTS_CTS_FLOW
void puart_allow_interrupt()
{
    wiced_hal_puart_reset_puart_interrupt( );
}
#endif

/**
 Function Name:
 test_puart_driver

 Function Description:
 @brief  This function turns off flow control, and enables Tx
         and Rx. Echoes the input byte with increment by 1.

 @param  void

 @return  void
 */
void test_puart_driver( void )
{
    /* Set flow control */
    #if PUART_RTS_CTS_FLOW
    wiced_hal_puart_flow_on();
    #else
    wiced_hal_puart_flow_off();
    #endif

    /* BEGIN - puart interrupt */
    wiced_hal_puart_reset_puart_interrupt();
    wiced_hal_puart_register_interrupt(puart_rx_interrupt_callback);
#ifndef CYW20706A2
    wiced_hal_puart_set_watermark_level(1);
#endif

    /* Turn on Tx */
    wiced_hal_puart_enable_tx();
    wiced_hal_puart_print( "Type something! "
            "Keystrokes are echoed to the terminal ...\r\n");

    #if PUART_RTS_CTS_FLOW
    wiced_hal_puart_print( "Using hardware flow control to limit data transfer buffer\r\n");

    /* Start a periodic timer to so data transfer is delayed */
    wiced_init_timer( &hal_puart_flow_timer, puart_allow_interrupt, 0, WICED_SECONDS_PERIODIC_TIMER);
    wiced_start_timer( &hal_puart_flow_timer, 1);
    #endif
}

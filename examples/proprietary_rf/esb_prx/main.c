/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "nrf_esb.h"

#include <stdbool.h>
#include <stdint.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_esb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_error.h"
#include "boards.h"
#include "nrf_drv_timer.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "SEGGER_RTT.h"
/* HOST:selective_auto_ack:true noack:true
selective_auto_ack:true noack:true
未应答
selective_auto_ack:false noack:true
未应答
selective_auto_ack:true noack:false
未应答
selective_auto_ack:false noack:false
未应答
*/
/* HOST:selective_auto_ack:true noack:false
selective_auto_ack:true noack:true
未应答
selective_auto_ack:false noack:true
未应答
selective_auto_ack:true noack:false
未应答
selective_auto_ack:false noack:false
未应答
*/
#define CH2             6
//#define U_LED_4             7
#define CH3             27 
const nrf_drv_timer_t log_timer = NRF_DRV_TIMER_INSTANCE(0);
const nrf_drv_timer_t count_timer = NRF_DRV_TIMER_INSTANCE(1);
uint8_t led_nr;

nrf_esb_payload_t tx_payload = \
{
  .pipe = 6,
  .length = 6,
  .noack = true,
};
nrf_esb_payload_t rx_payload;

uint8_t mac1[6] = {0x01,0x02,0x03,0x04,0x05,0x06};
uint8_t mac2[6] = {0x61,0x62,0x63,0x64,0x65,0x66};

volatile uint32_t recCount = 0;
volatile uint32_t txSuccess = 0;
volatile uint32_t txfailed = 0;
volatile uint32_t channel2 = 0;
volatile uint32_t channel4 = 0;
volatile uint8_t chIndex = 0;
uint8_t channelMap[] = {4,77};
/*lint -save -esym(40, BUTTON_1) -esym(40, BUTTON_2) -esym(40, BUTTON_3) -esym(40, BUTTON_4) -esym(40, LED_1) -esym(40, LED_2) -esym(40, LED_3) -esym(40, LED_4) */
uint32_t channel;
uint32_t time_ticks;
uint8_t isAdd = 0;
uint8_t num = 0;
uint8_t packet = 0;
volatile uint32_t frame = 0;
void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
    int ret = 0;
    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
            nrf_gpio_pin_toggle(CH3);
            txSuccess++;
            //SEGGER_RTT_printf(0,"TX SUCCESS EVENT\n");
            break;
        case NRF_ESB_EVENT_TX_FAILED:
            txfailed++;
            //SEGGER_RTT_printf(0,"TX FAILED EVENT\n");
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
            nrf_gpio_pin_toggle(CH2);
            //SEGGER_RTT_printf(0,"RX RECEIVED EVENT\n");
            nrf_drv_timer_disable(&count_timer);
            nrfx_timer_clear(&count_timer);
            nrf_drv_timer_extended_compare(&count_timer, NRF_TIMER_CC_CHANNEL0, \
            time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
            nrf_drv_timer_enable(&count_timer);
            if (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
            {
                //SEGGER_RTT_printf(0,"noack:%d\n",rx_payload.noack);
                recCount++;
                if(num != rx_payload.data[30])
                {
                  num = rx_payload.data[30];
                  packet = 0;
                  isAdd = 0;
                }
                ++packet;
                if(packet>=2 && isAdd == 0)
                {
                  ++frame;
                  isAdd = 1;
                }
                //nrf_esb_get_rf_channel(&channel);
                //if(channel == 0x02)
                //  channel2++;
                //else
                //  channel4++;
            }
            break;
    }
}


void clocks_start( void )
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}


void gpio_init( void )
{
    bsp_board_init(BSP_INIT_LEDS);
}


uint32_t esb_init( void )
{
    uint32_t err_code;
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };
    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.payload_length           = 50;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PRX;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.selective_auto_ack       = true;//false true

    err_code = nrf_esb_init(&nrf_esb_config);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, 8);
    VERIFY_SUCCESS(err_code);

    return err_code;
}

void count_timer_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            nrf_esb_stop_rx();
            ++chIndex;
            if(chIndex >= sizeof(channelMap))
              chIndex  = 0;
            nrf_esb_set_rf_channel(channelMap[chIndex]);
            nrf_esb_start_rx();
            //nrf_drv_timer_disable(&count_timer);
            break;

        default:
            //Do nothing.
            break;
    }
}
void log_timer_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            SEGGER_RTT_printf(0,"recCount:%d,frame = %d\n" \
            ,recCount,frame);
            recCount = 0;
            frame = 0;
            break;

        default:
            //Do nothing.
            break;
    }
}

void log_timer_init()
{
    uint32_t time_ms = 1000;
    //uint32_t time_us = 200;
    uint32_t time_ticks;
    uint32_t err_code = NRF_SUCCESS;
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&log_timer, &timer_cfg, log_timer_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrf_drv_timer_ms_to_ticks(&log_timer, time_ms);
    //time_ticks = nrf_drv_timer_us_to_ticks(&log_timer, time_us);
    nrf_drv_timer_extended_compare(
         &log_timer, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    nrf_drv_timer_enable(&log_timer);
}
void count_timer_init()
{
    //uint32_t time_ms = 1000;
    uint32_t time_us = 100;
    uint32_t err_code = NRF_SUCCESS;
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.interrupt_priority = 4;
    err_code = nrf_drv_timer_init(&count_timer, &timer_cfg, count_timer_handler);
    APP_ERROR_CHECK(err_code);

    //time_ticks = nrf_drv_timer_ms_to_ticks(&count_timer, time_ms);
    time_ticks = nrf_drv_timer_us_to_ticks(&count_timer, time_us);
}
void debug_init( void )
{
    nrf_gpio_cfg_output(CH2);
    nrf_gpio_cfg_output(CH3);
    nrf_gpio_pin_set(CH2);
    nrf_gpio_pin_set(CH3);
}
int main(void)
{
    uint32_t err_code;

    gpio_init();

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
    debug_init();
    clocks_start();

    err_code = esb_init();
    APP_ERROR_CHECK(err_code);

    SEGGER_RTT_printf(0,"Enhanced ShockBurst Receiver Example started.\n");
    log_timer_init();
    count_timer_init();
    err_code = nrf_esb_start_rx();
    APP_ERROR_CHECK(err_code);

    while (true)
    {
        //if(nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
        //  SEGGER_RTT_printf(0,"write ok\n");
        // else
        //  SEGGER_RTT_printf(0,"write err\n");
        //nrf_delay_ms(1000);
        if (NRF_LOG_PROCESS() == false)
        {
            __WFE();
        }
    }
}
/*lint -restore */

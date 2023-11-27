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
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_esb.h"
#include "nrf_error.h"
#include "nrf_esb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_delay.h"
#include "app_util.h"
#include "nrf_drv_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_atomic.h"
#include "SEGGER_RTT.h"
//#include "nrf_mtx.h"

#define CH0             6
//#define CH2             7
#define CH1             27 
const nrf_drv_timer_t log_timer = NRF_DRV_TIMER_INSTANCE(1);
const nrf_drv_timer_t count_timer = NRF_DRV_TIMER_INSTANCE(0);
//static nrf_esb_payload_t        tx_payload = NRF_ESB_CREATE_PAYLOAD(2, 0x01, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00);
static nrf_esb_payload_t        tx_payload;
static nrf_esb_payload_t        rx_payload;

//nrfx_atomic_u32_t writeCount = 0;
//nrfx_atomic_u32_t writeSuccess = 0;
//nrfx_atomic_u32_t writeFailed = 0;
//static nrf_mtx_t mLock;
volatile uint32_t writeCount = 0;
volatile uint32_t writeSuccess = 0;
volatile uint32_t writeFailed = 0;

volatile uint32_t txSuccess = 0;
volatile uint32_t txFailed = 0;
volatile uint32_t recCount = 0;

uint8_t rf_state = 0;

uint32_t channel = 0x02;

void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
    uint32_t ret = 0;
    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
            nrf_gpio_pin_toggle(CH0);
            txSuccess++;
            //SEGGER_RTT_printf(0,"TX SUCCESS EVENT\n");
            break;
        case NRF_ESB_EVENT_TX_FAILED:
            txFailed++;
            //SEGGER_RTT_printf(0,"TX FAILED EVENT\n");
            //(void) nrf_esb_flush_tx();
            //(void) nrf_esb_start_tx();
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
            nrf_gpio_pin_toggle(CH1);
            //SEGGER_RTT_printf(0,"RX RECEIVED EVENT\n");
            //nrf_esb_get_rf_channel(&channel);
            //if(channel == 0x02)
            //  ret = nrf_esb_set_rf_channel(0x04);
            //else
            //  ret = nrf_esb_set_rf_channel(0x02);
            //SEGGER_RTT_printf(0,"ret = %d\n",ret);
            while (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
            {
                if (rx_payload.length == 6)
                {
                    recCount++;
                    //SEGGER_RTT_printf(0,"RX RECEIVED PAYLOAD\n");
                }
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

/*
selective_auto_ack:true选择性等待应答，false时必须应答
noack:true不需要等待应答，false时必须应答
*/
uint32_t esb_init( void )
{
    uint32_t err_code;
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };

    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
    //nrf_esb_config.tx_mode                  = NRF_ESB_TXMODE_MANUAL_START;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.retransmit_delay         = 350;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PTX;
    nrf_esb_config.selective_auto_ack       = true;//false true

    err_code = nrf_esb_init(&nrf_esb_config);

    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, NRF_ESB_PIPE_COUNT);
    VERIFY_SUCCESS(err_code);

    return err_code;
}
//uint8_t timer_count = 0;
//void count_timer_handler(nrf_timer_event_t event_type, void* p_context)
//{
//    switch (event_type)
//    {
//        case NRF_TIMER_EVENT_COMPARE0:
//            timer_count++;
//            rf_state = timer_count%2;
//            break;
//        default:
//            //Do nothing.
//            break;
//    }
//}

void log_timer_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            SEGGER_RTT_printf(0,"send:%d,success:%d,loss:%d,txS:%d,txF:%d,rec:%d\n" \
            ,writeCount,writeSuccess,writeFailed,txSuccess,txFailed,recCount);
            if(writeFailed>=writeCount)
            {
                nrf_esb_flush_tx(); 
            }
            writeCount = 0;
            writeSuccess = 0;
            writeFailed = 0;
            txSuccess = 0;
            txFailed = 0;
            recCount = 0;
            break;
        default:
            //Do nothing.
            break;
    }
}
//void count_timer_init()
//{
//    //uint32_t time_ms = 1000; //Time(in miliseconds) between consecutive compare events.
//    uint32_t time_us = 500;
//    uint32_t time_ticks;
//    uint32_t err_code = NRF_SUCCESS;
//    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
//    timer_cfg.interrupt_priority = 4;
//    err_code = nrf_drv_timer_init(&count_timer, &timer_cfg, count_timer_handler);
//    APP_ERROR_CHECK(err_code);

//    //time_ticks = nrf_drv_timer_ms_to_ticks(&count_timer, time_ms);
//    time_ticks = nrf_drv_timer_us_to_ticks(&count_timer,time_us);
//    nrf_drv_timer_extended_compare(
//         &count_timer, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);    
//}
void log_timer_init()
{
    uint32_t time_ms = 1000; //Time(in miliseconds) between consecutive compare events.
    //uint32_t time_us = 500;
    uint32_t time_ticks;
    uint32_t err_code = NRF_SUCCESS;
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.interrupt_priority = 7;
    err_code = nrf_drv_timer_init(&log_timer, &timer_cfg, log_timer_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrf_drv_timer_ms_to_ticks(&log_timer, time_ms);
    //time_ticks = nrf_drv_timer_us_to_ticks(&count_timer,time_us);
    nrf_drv_timer_extended_compare(
         &log_timer, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
   nrf_drv_timer_enable(&log_timer);
}
void debug_init( void )
{
    nrf_gpio_cfg_output(CH0);
    nrf_gpio_cfg_output(CH1);
    nrf_gpio_pin_set(CH0);
    nrf_gpio_pin_set(CH1);
}
int main(void)
{
    ret_code_t err_code;

    //gpio_init();
    uint8_t i = 0;
    int bytes = 1;
    int delay = 0;
    int ret = 0;
     err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
    debug_init();
    clocks_start();
    //count_timer_init();
    log_timer_init();
    err_code = esb_init();
    APP_ERROR_CHECK(err_code);

    SEGGER_RTT_printf(0,"Enhanced ShockBurst Transmitter Example started.\n");
    tx_payload.pipe = 3;
    tx_payload.length = 32;
    tx_payload.noack = true;//false true
    for(i=1;i<252;i++)
    {
      tx_payload.data[i] = i;
    }
    //nrf_mtx_init(&mLock);
    //nrf_drv_timer_enable(&count_timer);
    while (true)
    {
        __set_BASEPRI(0x50);
        writeCount++;
        //if(rf_state<2)
        //{
          //nrf_esb_flush_tx();
          //ret = nrf_esb_set_rf_channel((rf_state+1)*2);
          if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
          {
              writeSuccess++;
          }
          else
          {
              writeFailed++;
          }
        //  rf_state = 2;
        //}
        //nrf_delay_us(350);
        //nrf_delay_ms(1000);
        __set_BASEPRI(0x00);

    }
}
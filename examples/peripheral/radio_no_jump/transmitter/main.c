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
/** @file
*
* @defgroup nrf_dev_button_radio_tx_example_main main.c
* @{
* @ingroup nrf_dev_button_radio_tx_example
*
* @brief Radio Transceiver Example Application main file.
*
* This file contains the source code for a sample application using the NRF_RADIO peripheral.
*
*/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "radio_config.h"
#include "nrf_gpio.h"
#include "app_timer.h"
#include "boards.h"
#include "bsp.h"
#include "nordic_common.h"
#include "nrf_error.h"
#include <nrfx.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "SEGGER_RTT.h"
#include "nrf_drv_timer.h"


static uint8_t packet[32] = {0};
uint8_t currChIndex = 0;
uint8_t chanelMap[] = {4,77};

volatile uint32_t advSend = 0;
volatile uint32_t timer2count = 0;


const nrf_drv_timer_t log_timer = NRF_DRV_TIMER_INSTANCE(1);

static void log_timer_handler(nrf_timer_event_t event_type, void* p_context)
{
  switch (event_type)
  {
      case NRF_TIMER_EVENT_COMPARE0:
        SEGGER_RTT_printf(0,"advSend:%d,t2:%d\n",advSend,timer2count);
        advSend = 0;
        timer2count = 0;
      break;
      default:
      break;
  }
}
void log_timer_init(void)
{
    uint32_t err_code = NRF_SUCCESS;
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.interrupt_priority = 1;
    err_code = nrf_drv_timer_init(&log_timer, &timer_cfg, log_timer_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_timer_extended_compare(
         &log_timer, NRF_TIMER_CC_CHANNEL0, nrfx_timer_ms_to_ticks(&log_timer,1000), NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);  //DIS_JUDGE
    nrf_drv_timer_enable(&log_timer);
}
void sync_timer_init(void)
{
  NRF_TIMER2->PRESCALER = 4;
  NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;
  NRF_TIMER2->BITMODE   = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER2->SHORTS    = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
  NRF_TIMER2->CC[0]    = 500;
  NRF_TIMER2->TASKS_CLEAR = 1;
  NRF_TIMER2->EVENTS_COMPARE[0] = 0;
  NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
  NVIC_SetPriority(TIMER2_IRQn, 2);
  //NVIC_ClearPendingIRQ(TIMER2_IRQn);
  NVIC_EnableIRQ(TIMER2_IRQn);
  NRF_TIMER2->TASKS_START = 1;
}
void sync_ppi_init(void)
{
  //NRF_PPI->CH[10].EEP = (uint32_t)&NRF_TIMER2->EVENTS_COMPARE[0];
  //NRF_PPI->CH[10].TEP = (uint32_t)&NRF_RADIO->TASKS_START;
  //NRF_PPI->CHENSET = 1<<10;
}
void radio_init()
{
  // Radio config
  NRF_RADIO->TXPOWER   = (RADIO_TXPOWER_TXPOWER_Pos8dBm << RADIO_TXPOWER_TXPOWER_Pos);
  NRF_RADIO->FREQUENCY = 4UL;  // Frequency bin 4, 2404MHz
  NRF_RADIO->MODE      = (RADIO_MODE_MODE_Nrf_1Mbit << RADIO_MODE_MODE_Pos);

  // Radio address config
  NRF_RADIO->PREFIX0 = 0XC1C2C3C4UL;

  NRF_RADIO->PREFIX1 = 0xC5C6C7C8UL;

  NRF_RADIO->BASE0 = 0x47534358UL;  // Base address for prefix 0 converted to nRF24L series format
  NRF_RADIO->BASE1 = 0x67736378UL;  // Base address for prefix 1-7 converted to nRF24L series format

  NRF_RADIO->TXADDRESS   = 0x00UL;  // Set device address 0 to use when transmitting
  NRF_RADIO->RXADDRESSES = 0x01UL;  // Enable device address 0 to use to select which addresses to receive

  // Packet configuration
  NRF_RADIO->PCNF0 = 0UL;

  // Packet configuration
  NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos) |
                     (RADIO_PCNF1_ENDIAN_Big       << RADIO_PCNF1_ENDIAN_Pos)  |
                     (4UL                          << RADIO_PCNF1_BALEN_Pos)   |
                     (32UL                         << RADIO_PCNF1_STATLEN_Pos) |
                     (32UL                         << RADIO_PCNF1_MAXLEN_Pos); //lint !e845 "The right argument to operator '|' is certain to be 0"

  // CRC Config
  NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos); // Number of checksum bits
  if ((NRF_RADIO->CRCCNF & RADIO_CRCCNF_LEN_Msk) == (RADIO_CRCCNF_LEN_Two << RADIO_CRCCNF_LEN_Pos))
  {
      NRF_RADIO->CRCINIT = 0xFFFFUL;   // Initial value
      NRF_RADIO->CRCPOLY = 0x11021UL;  // CRC poly: x^16 + x^12^x^5 + 1
  }
  else if ((NRF_RADIO->CRCCNF & RADIO_CRCCNF_LEN_Msk) == (RADIO_CRCCNF_LEN_One << RADIO_CRCCNF_LEN_Pos))
  {
      NRF_RADIO->CRCINIT = 0xFFUL;   // Initial value
      NRF_RADIO->CRCPOLY = 0x107UL;  // CRC poly: x^8 + x^2^x^1 + 1
  }
  NRF_RADIO->SHORTS = RADIO_SHORTS_END_DISABLE_Msk;
  NRF_RADIO->INTENSET = RADIO_INTENSET_READY_Msk|RADIO_INTENSET_DISABLED_Msk;
  NVIC_SetPriority(RADIO_IRQn, 1);
  NVIC_EnableIRQ(RADIO_IRQn);
  NRF_RADIO->PACKETPTR = (uint32_t)packet;
  NRF_RADIO->EVENTS_ADDRESS = 0UL;
  NRF_RADIO->EVENTS_PAYLOAD = 0UL;
  NRF_RADIO->EVENTS_DISABLED = 0UL;
  NRF_RADIO->TASKS_TXEN = 1UL;
}
void TIMER2_IRQHandler(void)
{
  NRF_TIMER2->EVENTS_COMPARE[0] = 0UL;
  NRF_RADIO->TASKS_START = 1UL;
  NRF_P1->OUTSET = (~NRF_P1->OUT & (1UL<<3));
  NRF_P1->OUTCLR = (NRF_P1->OUT & (1UL<<3));
  //timer2count++;
}
void RADIO_IRQHandler(void)
{
  if (NRF_RADIO->EVENTS_DISABLED)
  {
    //if((++packet[0])%2)
    //  NRF_P1->OUTCLR |= 1<<8;
    //else
    //  NRF_P1->OUTSET |= 1<<8;
    NRF_P1->OUTSET = (~NRF_P1->OUT & (1UL<<4));
    NRF_P1->OUTCLR = (NRF_P1->OUT & (1UL<<4));
    ++packet[1];
    if(packet[1]==10)
    {
      ++packet[0];
      packet[1] = 0;
    }
    //SEGGER_RTT_printf(0,"%d-%d\n",packet[0],packet[1]);
    NRF_RADIO->EVENTS_DISABLED = 0UL;
    currChIndex = !currChIndex;
    currChIndex = 0;
    NRF_RADIO->FREQUENCY = chanelMap[currChIndex];

    NRF_RADIO->PACKETPTR    = (uint32_t)packet;

    //NVIC_ClearPendingIRQ(RADIO_IRQn);
    NRF_RADIO->TASKS_TXEN = 1UL;
    NRF_P1->OUTSET = (~NRF_P1->OUT & (1UL<<1));
    NRF_P1->OUTCLR = (NRF_P1->OUT & (1UL<<1));
    ++advSend;
  }
  else if(NRF_RADIO->EVENTS_READY)
  {
    NRF_RADIO->EVENTS_READY = 0UL;
    NRF_P1->OUTSET = (~NRF_P1->OUT & (1UL<<2));
    NRF_P1->OUTCLR = (NRF_P1->OUT & (1UL<<2));
  }
}
/**@brief Function for initialization oscillators.
 */
void clock_initialization()
{
    /* Start 16 MHz crystal oscillator */
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;

    /* Wait for the external oscillator to start up */
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        // Do nothing.
    }

    /* Start low frequency crystal oscillator for app_timer(used by bsp)*/
    NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART    = 1;

    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
        // Do nothing.
    }
}


/**
 * @brief Function for application main entry.
 * @return 0. int return type required by ANSI/ISO standard.
 */
int main(void)
{
  uint32_t err_code = NRF_SUCCESS;
  
  clock_initialization();
  nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(1,1));
  nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(1,2));
  nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(1,3));
  nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(1,4));
  radio_init();
  sync_ppi_init();
  log_timer_init();
  sync_timer_init();
  
  SEGGER_RTT_printf(0,"Radio transmitter start no jump channel\n");
  while (true)
  {
  __WFI();
  }
}


/**
 *@}
 **/

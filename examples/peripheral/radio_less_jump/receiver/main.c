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
* @defgroup nrf_dev_radio_rx_example_main main.c
* @{
* @ingroup nrf_dev_radio_rx_example
* @brief Radio Receiver example Application main file.
*
* This file contains the source code for a sample application using the NRF_RADIO peripheral.
*
*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <nrfx.h>
#include "radio_config.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "bsp.h"
#include "app_timer.h"
#include "nordic_common.h"
#include "nrf_error.h"
#include "app_error.h"
#include "nrf_drv_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "SEGGER_RTT.h"
#include "nrf_atomic.h"


#define JUMPCHANNEL

nrf_atomic_u32_t flag = 0;

volatile uint32_t rec = 0;
volatile uint32_t crcerr = 0;
volatile uint32_t lost = 0;
volatile uint32_t flag2 = 0;
volatile uint32_t flag3 = 0;
volatile uint32_t tc1 = 0;
volatile uint32_t tc2 = 0;

volatile uint8_t num = 0;
volatile static uint8_t packet[32] = {0};              /**< Packet to transmit. */

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
const nrf_drv_timer_t log_timer = NRF_DRV_TIMER_INSTANCE(0);

static void log_timer_handler(nrf_timer_event_t event_type, void* p_context)
{
  switch (event_type)
  {
      case NRF_TIMER_EVENT_COMPARE0:
        SEGGER_RTT_printf(0,"r:%ld,cr:%d,f2:%d,f3:%d,tc1:%d,tc2:%d,ls:%d\n",\
        rec,crcerr,flag2,flag3,tc1,tc2,lost);
        rec = 0;
        crcerr = 0;
        flag2 = 0;
        flag3 = 0;
        tc1 = 0;
        tc2 = 0;
        lost = 0;
      break;
      default:
      break;
  }
}
void log_timer_init(void)
{
    uint32_t err_code = NRF_SUCCESS;
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.interrupt_priority = 7;
    err_code = nrf_drv_timer_init(&log_timer, &timer_cfg, log_timer_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_timer_extended_compare(
         &log_timer, NRF_TIMER_CC_CHANNEL0, nrfx_timer_ms_to_ticks(&log_timer,1000), NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);  //DIS_JUDGE
    nrf_drv_timer_enable(&log_timer);
}
uint8_t currChIndex = 0;
uint8_t chanelMap[] = {4,77};

void RADIO_IRQHandler(void)
{
  uint8_t tmp = 0;

/***************************************************************/
  #ifdef JUMPCHANNEL
  if (NRF_RADIO->EVENTS_DISABLED)
  {
    //if(NRF_P0->OUTSET & (1<<18))
    //  NRF_P0->OUTCLR |= 1<<18;
    //else
    //  NRF_P0->OUTSET |= 1<<18;  
    //SEGGER_RTT_printf(0,"DIS\n");
    NRF_RADIO->EVENTS_DISABLED = 0UL;
    switch(flag)
    {
      case 0://未同步
        //num = packet[0];
        //SEGGER_RTT_printf(0,"R0:%d\n",packet[0]);
        NRF_P0->OUTSET = (~NRF_P0->OUT & (1UL<<15));
        NRF_P0->OUTCLR = (NRF_P0->OUT & (1UL<<15));
        NRF_TIMER2->CC[0]    = 500;
        NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Msk|TIMER_INTENSET_COMPARE1_Msk;
        NRF_TIMER2->SHORTS    = TIMER_SHORTS_COMPARE1_CLEAR_Msk|TIMER_SHORTS_COMPARE1_STOP_Msk;
        currChIndex = !currChIndex;
        NRF_RADIO->FREQUENCY = chanelMap[currChIndex];
        NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk|RADIO_SHORTS_END_DISABLE_Msk;
        NRF_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Msk;
        NRF_PPI->CHENSET = 1<<10;
        NRF_RADIO->TXADDRESS    = 0UL;
        NRF_RADIO->RXADDRESSES  = 1UL;
        flag = 1;
        NRF_RADIO->TASKS_RXEN = 1UL; 
        if(NRF_RADIO->CRCSTATUS)
          ++rec;
        else
          ++crcerr;
      break;
      case 1://已同步
        //SEGGER_RTT_printf(0,"R1\n");
        if(packet[0]%2)
          NRF_P0->OUTCLR |= 1UL<<16;
        else
          NRF_P0->OUTSET |= 1UL<<16;
        //NRF_P0->OUTSET = (~NRF_P0->OUT & (1UL<<16));
        //NRF_P0->OUTCLR = (NRF_P0->OUT & (1UL<<16));
        NRF_TIMER2->TASKS_STOP = 1UL;
        NRF_TIMER2->TASKS_CLEAR = 1UL; 
        currChIndex = !currChIndex;
        NRF_RADIO->FREQUENCY = chanelMap[currChIndex];
        NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk|RADIO_SHORTS_END_DISABLE_Msk;
        NRF_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Msk;
        NRF_PPI->CHENSET = 1<<10;
        NRF_RADIO->TXADDRESS    = 0UL;
        NRF_RADIO->RXADDRESSES  = 1UL;
        NRF_RADIO->TASKS_RXEN = 1UL;  
        if(NRF_RADIO->CRCSTATUS)
          ++rec;
        else
          ++crcerr;
      break;
      case 2://初步丢失同步
        //SEGGER_RTT_printf(0,"R2\n");
        //NVIC_ClearPendingIRQ(TIMER2_IRQn);
        //NRF_TIMER2->EVENTS_COMPARE[0] = 0UL; 
        //NRF_TIMER2->EVENTS_COMPARE[1] = 0UL;
        NRF_P0->OUTSET = (~NRF_P0->OUT & (1UL<<18));
        NRF_P0->OUTCLR = (NRF_P0->OUT & (1UL<<18));
        //NRF_RADIO->MODECNF0 = RADIO_MODECNF0_RU_Msk;
        currChIndex = !currChIndex;
        NRF_RADIO->FREQUENCY = chanelMap[currChIndex];
        NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk|RADIO_SHORTS_END_DISABLE_Msk;
        NRF_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Msk;
        NRF_PPI->CHENCLR = 1<<10;
        NRF_RADIO->TXADDRESS    = 0UL;
        NRF_RADIO->RXADDRESSES  = 1UL;
        NRF_RADIO->TASKS_RXEN = 1UL;   
        flag = 1;
        ++flag2;
      break;
      case 3://进一步丢失同步
        //SEGGER_RTT_printf(0,"R3\n");
        //NVIC_ClearPendingIRQ(TIMER2_IRQn);
        //NRF_TIMER2->EVENTS_COMPARE[1] = 0UL;
        //NRF_TIMER2->EVENTS_COMPARE[0] = 0UL;
        NRF_P0->OUTSET = (~NRF_P0->OUT & (1UL<<20));
        NRF_P0->OUTCLR = (NRF_P0->OUT & (1UL<<20));
        NRF_TIMER2->CC[0] = 0;
        NRF_TIMER2->INTENCLR = TIMER_INTENSET_COMPARE0_Msk;
        NRF_TIMER2->SHORTS = TIMER_SHORTS_COMPARE1_CLEAR_Msk;
        NRF_RADIO->MODECNF0 = RADIO_MODECNF0_RU_Msk;
        currChIndex = !currChIndex;
        NRF_RADIO->FREQUENCY = chanelMap[currChIndex];
        NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk|RADIO_SHORTS_END_DISABLE_Msk;
        NRF_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Msk;
        NRF_PPI->CHENSET = 1<<10;
        //NVIC_ClearPendingIRQ(TIMER2_IRQn);
        NRF_RADIO->TXADDRESS    = 0UL;
        NRF_RADIO->RXADDRESSES  = 1UL;
        //NRF_TIMER2->EVENTS_COMPARE[0] = 0UL;
        //NRF_TIMER2->EVENTS_COMPARE[1] = 0UL;
        NRF_RADIO->TASKS_RXEN = 1UL;  
        flag = 1;
        ++flag3;
      break;
      default://未知
      break;
    }
  }
  #else
  if (NRF_RADIO->EVENTS_DISABLED)
  {
    NRF_RADIO->EVENTS_DISABLED = 0UL;
    //SEGGER_RTT_printf(0,"RADIO DISABLED\n");
    if(NRF_RADIO->CRCSTATUS)
      ++rec;
    else
      ++crcerr;
    NRF_RADIO->TASKS_RXEN = 1UL;
  }
  else if(NRF_RADIO->EVENTS_READY)
  {
    NRF_RADIO->EVENTS_READY = 0UL;
    SEGGER_RTT_printf(0,"RADIO READY\n");
  }
  #endif
}
#ifdef JUMPCHANNEL
void TIMER2_IRQHandler(void)
{
  static uint8_t count = 0;
  if(NRF_TIMER2->EVENTS_COMPARE[0])
  {
    NRF_TIMER2->EVENTS_COMPARE[0] = 0UL;
    ++tc1;
    //SEGGER_RTT_printf(0,"T1\n");
    nrfx_atomic_u32_store(&flag,2);
    //SEGGER_RTT_printf(0,"TIMER COMPARE0\n");
    NRF_P0->OUTSET = (~NRF_P0->OUT & (1UL<<17));
    NRF_P0->OUTCLR = (NRF_P0->OUT & (1UL<<17));
    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;
    //触发任务前需要清除事件
    NRF_RADIO->TASKS_DISABLE = 1;
  }
  else
  {
    ++tc2;
    NRF_TIMER2->EVENTS_COMPARE[1] = 0UL;
    //NRF_TIMER2->TASKS_STOP = 1;
    //NRF_TIMER2->TASKS_CLEAR = 1;     
    //SEGGER_RTT_printf(0,"TIMER COMPARE1\n");
    NRF_P0->OUTSET = (~NRF_P0->OUT & (1UL<<19));
    NRF_P0->OUTCLR = (NRF_P0->OUT & (1UL<<19));
    if((++count)>2)
    {
      //SEGGER_RTT_printf(0,"T3\n");
      nrfx_atomic_u32_store(&flag,0);
      count = 0;

      NRF_PPI->CHENCLR = 1UL;
      NRF_TIMER2->TASKS_STOP = 1UL;
      ++lost;
    }
    else
    {
      //SEGGER_RTT_printf(0,"T2\n");
      nrfx_atomic_u32_store(&flag,3);
      NRF_RADIO->EVENTS_READY = 0;
      NRF_RADIO->EVENTS_DISABLED = 0;
      NRF_RADIO->TASKS_DISABLE = 1;
    }
  }
  //NRF_RADIO->TASKS_DISABLE = 1;
}

void sync_timer_init(void)
{
  NRF_TIMER2->PRESCALER = 4;
  NRF_TIMER2->MODE = TIMER_MODE_MODE_Timer;
  NRF_TIMER2->BITMODE   = TIMER_BITMODE_BITMODE_16Bit;
  NRF_TIMER2->SHORTS    = TIMER_SHORTS_COMPARE1_CLEAR_Msk|TIMER_SHORTS_COMPARE1_STOP_Msk;
  NRF_TIMER2->CC[0]    = 500;
  NRF_TIMER2->CC[1]    = 1000;
  NRF_TIMER2->TASKS_CLEAR = 1;
  NRF_TIMER2->EVENTS_COMPARE[0] = 0;
  NRF_TIMER2->EVENTS_COMPARE[1] = 0;
  NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE0_Msk|TIMER_INTENSET_COMPARE1_Msk;
  //NRF_TIMER2->INTENSET = TIMER_INTENSET_COMPARE1_Msk;
  NVIC_SetPriority(TIMER2_IRQn, 2);
  //NVIC_ClearPendingIRQ(TIMER2_IRQn);
  NVIC_EnableIRQ(TIMER2_IRQn);
  //NRF_TIMER2->TASKS_START = 1;
}
void ppi_init(void)
{
  NRF_PPI->CH[10].EEP = (uint32_t)&NRF_RADIO->EVENTS_READY;
  NRF_PPI->CH[10].TEP = (uint32_t)&NRF_TIMER2->TASKS_START;  
  //NRF_PPI->CH[11].EEP = (uint32_t)&NRF_TIMER2->EVENTS_COMPARE[0];
  //NRF_PPI->CH[11].TEP = (uint32_t)&NRF_RADIO->TASKS_DISABLE;
  NRF_PPI->CHENCLR = 1<<10;
  //NRF_PPI->CHENSET = 1<<10|1<<11;  
}
#endif
/**
 * @brief Function for application main entry.
 * @return 0. int return type required by ANSI/ISO standard.
 */
int main(void)
{
    uint32_t err_code = NRF_SUCCESS;

    clock_initialization();
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0,20));
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0,19));
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0,18));
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0,17));
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0,16));
    nrf_gpio_cfg_output(NRF_GPIO_PIN_MAP(0,15));
    #ifdef JUMPCHANNEL
    sync_timer_init();
    ppi_init();
    #endif
    radio_configure();
    NRF_RADIO->PACKETPTR = (uint32_t)&packet;
    log_timer_init();
    SEGGER_RTT_printf(0,"Radio receiver example started.\n");
    SEGGER_RTT_printf(0,"Wait for first packet\n");
    APP_ERROR_CHECK(err_code);
    while (true)
    {
        //uint32_t received = read_packet();
        //++rec;
        //SEGGER_RTT_printf(0,"ttttt %d\n", (unsigned int)received);

    }
}

/**
 *@}
 **/

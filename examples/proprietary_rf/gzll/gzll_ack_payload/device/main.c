/**
 * Copyright (c) 2012 - 2019, Nordic Semiconductor ASA
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
/**
 * This project requires that a host that runs the
 * @ref gzll_host_m_ack_payload_example example is used as a counterpart for
 * receiving the data. This can be on either an nRF5x device or an nRF24Lxx device
 * running the \b gzll_host_m_ack_payload example in the nRFgo SDK.
 *
 * This example sends a packet and adds a new packet to the TX queue every time
 * it receives an ACK. Before adding a packet to the TX queue, the contents of
 * the GPIO Port BUTTONS is copied to the first payload byte (byte 0).
 * When an ACK is received, the contents of the first payload byte of
 * the ACK are output on GPIO Port LEDS.
 */

#include "nrf_gzll.h"
#include "bsp.h"
#include "app_timer.h"
#include "app_error.h"
#include "nrf_gzll_error.h"
#include "nrf_drv_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "SEGGER_RTT.h"



const nrf_drv_timer_t log_timer = NRF_DRV_TIMER_INSTANCE(3);
const nrf_drv_timer_t count_timer = NRF_DRV_TIMER_INSTANCE(4);

volatile uint32_t sends = 0;
volatile uint32_t sendf = 0;
volatile uint32_t resend = 0;
/*****************************************************************************/
/** @name Configuration */
/*****************************************************************************/
#define PIPE_NUMBER             0   /**< Pipe 0 is used in this example. */
#define TX_PAYLOAD_LENGTH       32   /**< 1-byte payload length is used when transmitting. */
#define MAX_TX_ATTEMPTS         1 /**< Maximum number of transmission attempts */

static uint8_t                  m_data_payload[TX_PAYLOAD_LENGTH];                /**< Payload to send to Host. */
static uint8_t                  m_ack_payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH]; /**< Placeholder for received ACK payloads from Host. */

#if GZLL_TX_STATISTICS
static nrf_gzll_tx_statistics_t m_statistics;   /**< Struct containing transmission statistics. */
#endif

#if GZLL_PA_LNA_CONTROL

#define GZLL_PA_LNA_TIMER       CONCAT_2(NRF_TIMER, GZLL_PA_LNA_TIMER_NUM) /**< Convert timer number into timer struct. */

/**< PA/LNA structure configuration. */
static nrf_gzll_pa_lna_cfg_t m_pa_lna_cfg = {
    .lna_enabled        = GZLL_LNA_ENABLED,
    .pa_enabled         = GZLL_PA_ENABLED,
    .lna_active_high    = GZLL_LNA_ACTIVE_HIGH,
    .pa_active_high     = GZLL_PA_ACTIVE_HIGH,
    .lna_gpio_pin       = GZLL_PA_LNA_CRX_PIN,
    .pa_gpio_pin        = GZLL_PA_LNA_CTX_PIN,
    .pa_gpiote_channel  = GZLL_PA_LNA_TX_GPIOTE_CHAN,
    .lna_gpiote_channel = GZLL_PA_LNA_RX_GPIOTE_CHAN,
    .timer              = GZLL_PA_LNA_TIMER,
    .ppi_channels[0]    = GZLL_PA_LNA_PPI_CHAN_1,
    .ppi_channels[1]    = GZLL_PA_LNA_PPI_CHAN_2,
    .ppi_channels[2]    = GZLL_PA_LNA_PPI_CHAN_3,
    .ppi_channels[3]    = GZLL_PA_LNA_PPI_CHAN_4,
    .ramp_up_time       = GZLL_PA_LNA_RAMP_UP_TIME
};

static int32_t m_rssi_sum    = 0; /**< Variable used to calculate average RSSI. */
static int32_t m_packets_cnt = 0; /**< Transmitted packets counter. */
#endif

/**
 * @brief Function to read the button states.
 *
 * @return Returns states of buttons.
 */
static uint8_t input_get(void)
{
    uint8_t result = 0;
    for (uint32_t i = 0; i < BUTTONS_NUMBER; i++)
    {
        if (bsp_button_is_pressed(i))
        {
            result |= (1 << i);
        }
    }

    return ~(result);
}


/**
 * @brief Function to control the LED outputs.
 *
 * @param[in] val Desirable state of the LEDs.
 */
static void output_present(uint8_t val)
{
    uint32_t i;

    for (i = 0; i < LEDS_NUMBER; i++)
    {
        if (val & (1 << i))
        {
            bsp_board_led_on(i);
        }
        else
        {
            bsp_board_led_off(i);
        }
    }
}


/**
 * @brief Initialize the BSP modules.
 */
static void ui_init(void)
{
    uint32_t err_code;

    // Initialize application timer.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // BSP initialization.
    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, NULL);
    APP_ERROR_CHECK(err_code);

    // Set up logger
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("Gazell ACK payload example. Device mode.");
    NRF_LOG_FLUSH();

    bsp_board_init(BSP_INIT_LEDS);
}


/*****************************************************************************/
/** @name Gazell callback function definitions  */
/*****************************************************************************/
/**
 * @brief TX success callback.
 *
 * @details If an ACK was received, another packet is sent.
 */
void  nrf_gzll_device_tx_success(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
    bool     result_value         = false;
    uint32_t m_ack_payload_length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;
    ++sends;
    if (tx_info.payload_received_in_ack)
    {
        // Pop packet and write first byte of the payload to the GPIO port.
        result_value =
            nrf_gzll_fetch_packet_from_rx_fifo(pipe, m_ack_payload, &m_ack_payload_length);
        if (!result_value)
        {
          SEGGER_RTT_printf(0,"rec err\n");
        }
    }
}


/**
 * @brief TX failed callback.
 *
 * @details If the transmission failed, send a new packet.
 *
 * @warning This callback does not occur by default since NRF_GZLL_DEFAULT_MAX_TX_ATTEMPTS
 * is 0 (inifinite retransmits).
 */
void nrf_gzll_device_tx_failed(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
    ++sendf;
    resend += tx_info.num_tx_attempts;
    // Load data into TX queue.
    //bool result_value = nrf_gzll_add_packet_to_tx_fifo(pipe, m_data_payload, TX_PAYLOAD_LENGTH);
    //if (!result_value)
    //{

    //}
}


/**
 * @brief Gazelle callback.
 * @warning Required for successful Gazell initialization.
 */
void nrf_gzll_host_rx_data_ready(uint32_t pipe, nrf_gzll_host_rx_info_t rx_info)
{

}


/**
 * @brief Gazelle callback.
 * @warning Required for successful Gazell initialization.
 */
void nrf_gzll_disabled()
{
}

#if GZLL_PA_LNA_CONTROL
/**
 * @brief Function for configuring front end control in Gazell.
 */
static bool front_end_control_setup(void)
{
    bool result_value = true;

    // Configure pins controlling SKY66112 module.
    nrf_gpio_cfg_output(GZLL_PA_LNA_CHL_PIN);
    nrf_gpio_cfg_output(GZLL_PA_LNA_CPS_PIN);
    nrf_gpio_cfg_output(GZLL_PA_LNA_ANT_SEL_PIN);
    nrf_gpio_cfg_output(GZLL_PA_LNA_CSD_PIN);

    // Turn on front end module.
    nrf_gpio_pin_clear(GZLL_PA_LNA_CHL_PIN);
    nrf_gpio_pin_clear(GZLL_PA_LNA_CPS_PIN);
    nrf_gpio_pin_clear(GZLL_PA_LNA_ANT_SEL_PIN);
    nrf_gpio_pin_set(GZLL_PA_LNA_CSD_PIN);

    // PA/LNA configuration must be called after @ref nrf_gzll_init() and before @ref nrf_gzll_enable()
    result_value = nrf_gzll_set_pa_lna_cfg(&m_pa_lna_cfg);

    return result_value;
}
#endif
volatile uint32_t write = 0;
volatile uint32_t writes = 0;
volatile uint32_t writef = 0;
void log_timer_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            SEGGER_RTT_printf(0,"w:%d,ws:%d,wf:%d,sf:%d,ss:%d,rs:%d\n" \
            ,write,writes,writef,sendf,sends,resend);
            write = 0;
            writes = 0;
            writef = 0;
            sendf = 0;
            sends = 0;
            resend = 0;
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
/*****************************************************************************/
/**
 * @brief Main function.
 *
 * @return ANSI required int return type.
 */
/*****************************************************************************/
uint8_t table[NRF_GZLL_CONST_MAX_CHANNEL_TABLE_SIZE] = {4, 15, 25, 42, 63, 77, 80};
int main()
{

    // Initialize Gazell.
    bool result_value = nrf_gzll_init(NRF_GZLL_MODE_DEVICE);
    GAZELLE_ERROR_CODE_CHECK(result_value);

    // Attempt sending every packet up to MAX_TX_ATTEMPTS times.
    nrf_gzll_set_max_tx_attempts(2);
    
    nrf_gzll_set_channel_table(table,1);
    nrf_gzll_tx_statistics_disable();
//#if GZLL_TX_STATISTICS
//    // Turn on transmission statistics gathering.
//    result_value = nrf_gzll_tx_statistics_enable(&m_statistics);
//    GAZELLE_ERROR_CODE_CHECK(result_value);
//#endif
    for(int i=0;i<TX_PAYLOAD_LENGTH;++i)
    {
      m_data_payload[i] = i;
    }
    result_value = nrf_gzll_add_packet_to_tx_fifo(PIPE_NUMBER, m_data_payload, TX_PAYLOAD_LENGTH);
    if (!result_value)
    {
        SEGGER_RTT_printf(0,"TX fifo error \n");
    }
    log_timer_init();
    // Enable Gazell to start sending over the air.
    result_value = nrf_gzll_enable();
    GAZELLE_ERROR_CODE_CHECK(result_value);
    uint8_t tmp_table[NRF_GZLL_CONST_MAX_CHANNEL_TABLE_SIZE] = {0};
    uint32_t table_size = 0;
    SEGGER_RTT_printf(0,"Gzll ack payload device example started.\n");
    SEGGER_RTT_printf(0,"timeslot_period:%d\n",nrf_gzll_get_timeslot_period());
    SEGGER_RTT_printf(0,"max attempts:%d\n",nrf_gzll_get_max_tx_attempts());
    table_size = nrf_gzll_get_channel_table_size();
    SEGGER_RTT_printf(0,"table size:%d\n",table_size);
    nrf_gzll_get_channel_table(&tmp_table[0],&table_size);
    SEGGER_RTT_printf(0,"table: ");
    for(int i=0;i<table_size;++i)
    {
      SEGGER_RTT_printf(0,"[%d] ",tmp_table[i]);
    }
    SEGGER_RTT_printf(0,"\n");
    SEGGER_RTT_printf(0,"datarate:%d\n",nrf_gzll_get_datarate());
    while (true)
    {
      __disable_irq();
      ++write;
      if(nrf_gzll_ok_to_add_packet_to_tx_fifo(PIPE_NUMBER))
      {
        ++m_data_payload[0];
        nrf_gzll_add_packet_to_tx_fifo(PIPE_NUMBER, m_data_payload, TX_PAYLOAD_LENGTH);
        ++writes;
      }
      else
      {
        ++writef;
      }
      __enable_irq();
    }
}

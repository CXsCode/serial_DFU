/**
 * Copyright (c) 2016 - 2019, Nordic Semiconductor ASA
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

#include "nrf_error.h"
#include "nrf_esb.h"
#include "nrf_esb_error_codes.h"
#include "nrf_gpio.h"
#include <string.h>
#include <stddef.h>
#include "sdk_common.h"
#include "sdk_macros.h"
#include "app_util.h"
#include "nrf_delay.h"

#define BIT_MASK_UINT_8(x) (0xFF >> (8 - (x)))

// Constant parameters
#define RX_WAIT_FOR_ACK_TIMEOUT_US_2MBPS        (48)        /**< 2 Mb RX wait for acknowledgment time-out value. Smallest reliable value - 43. */
#define RX_WAIT_FOR_ACK_TIMEOUT_US_1MBPS        (73)        /**< 1 Mb RX wait for acknowledgment time-out value. Smallest reliable value - 68. */
#define RX_WAIT_FOR_ACK_TIMEOUT_US_250KBPS      (250)       /**< 250 Kb RX wait for acknowledgment time-out value. */
#define RX_WAIT_FOR_ACK_TIMEOUT_US_1MBPS_BLE    (73)        /**< 1 Mb RX wait for acknowledgment time-out (combined with BLE). Smallest reliable value - 68.*/

// Interrupt flags
#define     NRF_ESB_INT_TX_SUCCESS_MSK          0x01        /**< Interrupt mask value for TX success. */
#define     NRF_ESB_INT_TX_FAILED_MSK           0x02        /**< Interrupt mask value for TX failure. */
#define     NRF_ESB_INT_RX_DATA_RECEIVED_MSK    0x04        /**< Interrupt mask value for RX_DR. */

#define     NRF_ESB_PID_RESET_VALUE             0xFF        /**< Invalid PID value which is guaranteed to not collide with any valid PID value. */
#define     NRF_ESB_PID_MAX                     3           /**< Maximum value for PID. */
#define     NRF_ESB_CRC_RESET_VALUE             0xFFFF      /**< CRC reset value. */

// Internal Enhanced ShockBurst module state.
typedef enum {
    //空闲
    NRF_ESB_STATE_IDLE,                                     /**< Module idle. */
    //TX模式下正在传输不需要应答的数据包
    NRF_ESB_STATE_PTX_TX,                                   /**< Module transmitting without acknowledgment. */
    //TX模式下正在传输需要应答的数据包
    NRF_ESB_STATE_PTX_TX_ACK,                               /**< Module transmitting with acknowledgment. */
    //TX模式下正在等待有载荷的应答数据包
    NRF_ESB_STATE_PTX_RX_ACK,                               /**< Module transmitting with acknowledgment and reception of payload with the acknowledgment response. */
    //正在接收不需要应答的数据包
    NRF_ESB_STATE_PRX,                                      /**< Module receiving packets without acknowledgment. */
    //在RX模式下发送应答
    NRF_ESB_STATE_PRX_SEND_ACK,                             /**< Module transmitting acknowledgment in RX mode. */
} nrf_esb_mainstate_t;

//失能Radio中断
#define DISABLE_RF_IRQ()      NVIC_DisableIRQ(RADIO_IRQn)
//使能Radio中断
#define ENABLE_RF_IRQ()       NVIC_EnableIRQ(RADIO_IRQn)
//Radio通用快捷方式：READY-START/END-DISABLE/ADDRESS-RSSISTART/DISABLED-RSSISTOP
#define _RADIO_SHORTS_COMMON ( RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | \
            RADIO_SHORTS_ADDRESS_RSSISTART_Msk | RADIO_SHORTS_DISABLED_RSSISTOP_Msk )
//检测长度宏定义
//如果长度=0 或 长度>NRF_ESB_MAX_PAYLOAD_LENGTH 或 选择了定长模式 或 长度>初始化配置长度 则 长度错误
#define VERIFY_PAYLOAD_LENGTH(p)                            \
do                                                          \
{                                                           \
    if (p->length == 0 ||                                   \
       p->length > NRF_ESB_MAX_PAYLOAD_LENGTH ||            \
       (m_config_local.protocol == NRF_ESB_PROTOCOL_ESB &&  \
        p->length > m_config_local.payload_length))         \
    {                                                       \
        return NRF_ERROR_INVALID_LENGTH;                    \
    }                                                       \
}while (0)


/* @brief Structure holding pipe info PID and CRC and acknowledgment payload. */
typedef struct
{
    //该管道最后一包数据的CRC，判断是否是重复包
    uint16_t    crc;                                      /**< CRC value of the last received packet (Used to detect retransmits). */
    //该管道最后一包数据的包ID，判断是否是重复包
    uint8_t     pid;                                      /**< Packet ID of the last received packet (Used to detect retransmits). */
    //需要应答的数据包的传输状态
    bool        ack_payload;                              /**< Flag indicating the state of the transmission of acknowledgment payloads. */
} pipe_info_t;


/* @brief  First-in, first-out queue of payloads to be transmitted. */
typedef struct
{
    //发送负载指针数组，每个元素指向一个负载的首地址
    nrf_esb_payload_t * p_payload[NRF_ESB_TX_FIFO_SIZE];  /**< Pointer to the actual queue. */
    //写索引
    uint32_t            entry_point;                      /**< Current start of queue. */
    //读索引
    uint32_t            exit_point;                       /**< Current end of queue. */
    //队列可用元素个数
    uint32_t            count;                            /**< Current number of elements in the queue. */
} nrf_esb_payload_tx_fifo_t;


/* @brief First-in, first-out queue of received payloads. */
typedef struct
{
    //接收负载指针数组
    nrf_esb_payload_t * p_payload[NRF_ESB_RX_FIFO_SIZE];  /**< Pointer to the actual queue. */
    //写索引
    uint32_t            entry_point;                      /**< Current start of queue. */
    //读索引
    uint32_t            exit_point;                       /**< Current end of queue. */
    //队列可用元素个数
    uint32_t            count;                            /**< Current number of elements in the queue. */
} nrf_esb_payload_rx_fifo_t;


/**@brief Enhanced ShockBurst address.
 *
 * Enhanced ShockBurst addresses consist of a base address and a prefix
 *          that is unique for each pipe. See @ref esb_addressing in the ESB user
 *          guide for more information.
*/
typedef struct
{
    //管道0的基址，大端
    uint8_t base_addr_p0[4];        /**< Base address for pipe 0 encoded in big endian. */
    //管道1~7的基址，大端
    uint8_t base_addr_p1[4];        /**< Base address for pipe 1-7 encoded in big endian. */
    //0~7的前缀
    uint8_t pipe_prefixes[8];       /**< Address prefix for pipe 0 to 7. */
    //可用管道数
    uint8_t num_pipes;              /**< Number of pipes available. */
    //地址长度(基址+前缀)
    uint8_t addr_length;            /**< Length of the address including the prefix. */
    //接收通管道能设置(每一位表示一个管道)
    uint8_t rx_pipes_enabled;       /**< Bitfield for enabled pipes. */
    //射频通道(0~100)
    uint8_t rf_channel;             /**< Channel to use (must be between 0 and 100). */
} nrf_esb_address_t;


// Module state
//是否已经初始化
static bool                         m_esb_initialized           = false;
//ESB当前状态
static volatile nrf_esb_mainstate_t m_nrf_esb_mainstate         = NRF_ESB_STATE_IDLE;
static nrf_esb_payload_t          * mp_current_payload;
//发送/接收事件的处理回调
static nrf_esb_event_handler_t      m_event_handler;

// Address parameters
//地址设置
__ALIGN(4) static nrf_esb_address_t m_esb_addr = NRF_ESB_ADDR_DEFAULT;

// RF parameters
//本地射频参数
static nrf_esb_config_t             m_config_local;

// TX FIFO
//发送队列与负载内存池
static nrf_esb_payload_t            m_tx_fifo_payload[NRF_ESB_TX_FIFO_SIZE];
static nrf_esb_payload_tx_fifo_t    m_tx_fifo;

// RX FIFO
//接收队列与负载内存池
static nrf_esb_payload_t            m_rx_fifo_payload[NRF_ESB_RX_FIFO_SIZE];
static nrf_esb_payload_rx_fifo_t    m_rx_fifo;

// Payload buffers
//发送负载缓存
static  uint8_t                     m_tx_payload_buffer[NRF_ESB_MAX_PAYLOAD_LENGTH + 2];
//接收负载缓存
static  uint8_t                     m_rx_payload_buffer[NRF_ESB_MAX_PAYLOAD_LENGTH + 2];

// Run time variables
//中断状态
static volatile uint32_t            m_interrupt_flags = 0;
//每个管道的包序号
static uint8_t                      m_pids[NRF_ESB_PIPE_COUNT];
//每个接收管道的管道信息
static pipe_info_t                  m_rx_pipe_info[NRF_ESB_PIPE_COUNT];
//剩余重传次数
static volatile uint32_t            m_retransmits_remaining;
//尝试重传次数
static volatile uint32_t            m_last_tx_attempts;
//等待应答延时
static volatile uint32_t            m_wait_for_ack_timeout_us;
//通用快捷方式
static volatile uint32_t            m_radio_shorts_common = _RADIO_SHORTS_COMMON;

// These function pointers are changed dynamically, depending on protocol configuration and state.
static void (*on_radio_disabled)(void) = 0;
static void (*on_radio_end)(void) = 0;
static void (*update_rf_payload_format)(uint32_t payload_length) = 0;


// The following functions are assigned to the function pointers above.
//不需要应答时，DISABLED事件的回调函数
static void on_radio_disabled_tx_noack(void);
//需要应答时，DISABLED事件的回调函数
static void on_radio_disabled_tx(void);
//等待应答时，DISABLED事件的回调函数
static void on_radio_disabled_tx_wait_for_ack(void);
//
static void on_radio_disabled_rx(void);
//
static void on_radio_disabled_rx_ack(void);


#define NRF_ESB_ADDR_UPDATE_MASK_BASE0          (1 << 0)    /*< Mask value to signal updating BASE0 radio address. */
#define NRF_ESB_ADDR_UPDATE_MASK_BASE1          (1 << 1)    /*< Mask value to signal updating BASE1 radio address. */
#define NRF_ESB_ADDR_UPDATE_MASK_PREFIX         (1 << 2)    /*< Mask value to signal updating radio prefixes. */


// Function to do bytewise bit-swap on an unsigned 32-bit value
//按字节逆序
static uint32_t bytewise_bit_swap(uint8_t const * p_inp)
{
#if __CORTEX_M == (0x04U)
    uint32_t inp = (*(uint32_t*)p_inp);
    return __REV((uint32_t)__RBIT(inp)); //lint -esym(628, __rev) -esym(526, __rev) -esym(628, __rbit) -esym(526, __rbit) */
#else
    uint32_t inp = (p_inp[3] << 24) | (p_inp[2] << 16) | (p_inp[1] << 8) | (p_inp[0]);
    inp = (inp & 0xF0F0F0F0) >> 4 | (inp & 0x0F0F0F0F) << 4;
    inp = (inp & 0xCCCCCCCC) >> 2 | (inp & 0x33333333) << 2;
    inp = (inp & 0xAAAAAAAA) >> 1 | (inp & 0x55555555) << 1;
    return inp;
#endif
}


// Convert a base address from nRF24L format to nRF5 format
//地址转换，nRF24L和nRF5的地址高低位相反
static uint32_t addr_conv(uint8_t const* p_addr)
{
    return __REV(bytewise_bit_swap(p_addr)); //lint -esym(628, __rev) -esym(526, __rev) */
}

#ifdef NRF52832_XXAA
static ret_code_t apply_address_workarounds()
{
    if ((NRF_FICR->INFO.VARIANT & 0x0000FF00) == 0x00004200) //Check if the device is an nRF52832 Rev. 1.
    {
        // Workaround for nRF52832 Rev 1 erratas
        //  Set up radio parameters.
        NRF_RADIO->MODECNF0 = (NRF_RADIO->MODECNF0 & ~RADIO_MODECNF0_RU_Msk) | RADIO_MODECNF0_RU_Default << RADIO_MODECNF0_RU_Pos;

        // Workaround for nRF52832 Rev 1 Errata 102 and nRF52832 Rev 1 Errata 106. This will reduce sensitivity by 3dB.
        *((volatile uint32_t *)0x40001774) = (*((volatile uint32_t *)0x40001774) & 0xFFFFFFFE) | 0x01000000;
    }

    if ((NRF_FICR->INFO.VARIANT & 0x0000FF00) == 0x00004500)//Check if the device is an nRF52832 Rev. 2.
    {
        /*
        Workaround for nRF52832 Rev 2 Errata 143
        Check if the most significant bytes of address 0 (including prefix) match those of another address.
        It's recommended to use a unique address 0 since this will avoid the 3dBm penalty incurred from the workaround.
        */
        uint32_t base_address_mask = m_esb_addr.addr_length == 5 ? 0xFFFF0000 : 0xFF000000;

        // Load the two addresses before comparing them to ensure defined ordering of volatile accesses.
        uint32_t addr0 = NRF_RADIO->BASE0 & base_address_mask;
        uint32_t addr1 = NRF_RADIO->BASE1 & base_address_mask;
        if (addr0 == addr1)
        {
            uint32_t prefix0 = NRF_RADIO->PREFIX0 & 0x000000FF;
            uint32_t prefix1 = (NRF_RADIO->PREFIX0 & 0x0000FF00) >> 8;
            uint32_t prefix2 = (NRF_RADIO->PREFIX0 & 0x00FF0000) >> 16;
            uint32_t prefix3 = (NRF_RADIO->PREFIX0 & 0xFF000000) >> 24;
            uint32_t prefix4 = NRF_RADIO->PREFIX1 & 0x000000FF;
            uint32_t prefix5 = (NRF_RADIO->PREFIX1 & 0x0000FF00) >> 8;
            uint32_t prefix6 = (NRF_RADIO->PREFIX1 & 0x00FF0000) >> 16;
            uint32_t prefix7 = (NRF_RADIO->PREFIX1 & 0xFF000000) >> 24;
            
            if (prefix0 == prefix1 || prefix0 == prefix2 || prefix0 == prefix3 || prefix0 == prefix4 || 
                prefix0 == prefix5 || prefix0 == prefix6 || prefix0 == prefix7)
            {
                // This will cause a 3dBm sensitivity loss, avoid using such address combinations if possible.
                *(volatile uint32_t *) 0x40001774 = ((*(volatile uint32_t *) 0x40001774) & 0xfffffffe) | 0x01000000; 
            }
        }
    }
    return NRF_SUCCESS;
}
#endif

//dpl：dynamic payload length
//变长发送
static void update_rf_payload_format_esb_dpl(uint32_t payload_length)
{
#if (NRF_ESB_MAX_PAYLOAD_LENGTH <= 32)
    // Using 6 bits for length
    //最大长度小于等于32时空中包有：
    //S0不占用空间，LENGTH占用6个bits，S1占用3个bits
    NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_S0LEN_Pos) |
                       (6 << RADIO_PCNF0_LFLEN_Pos) |
                       (3 << RADIO_PCNF0_S1LEN_Pos) ;
#else
    // Using 8 bits for length
    //最大长度大于32时空中包有：
    //S0不占用空间，LENGTH占用8个bits，S1占用3个bits
    NRF_RADIO->PCNF0 = (0 << RADIO_PCNF0_S0LEN_Pos) |
                       (8 << RADIO_PCNF0_LFLEN_Pos) |
                       (3 << RADIO_PCNF0_S1LEN_Pos) ;
#endif
    //NRF_RADIO->MODECNF0 = (NRF_RADIO->MODECNF0 & ~RADIO_MODECNF0_RU_Msk) | RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos;
    //失能数据白化
    //大端模式
    //基址长度=5-1=4
    //静态数据长度 = 0
    //最大数据长度 = NRF_ESB_MAX_PAYLOAD_LENGTH
    NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled    << RADIO_PCNF1_WHITEEN_Pos) |
                       (RADIO_PCNF1_ENDIAN_Big          << RADIO_PCNF1_ENDIAN_Pos)  |
                       ((m_esb_addr.addr_length - 1)    << RADIO_PCNF1_BALEN_Pos)   |
                       (0                               << RADIO_PCNF1_STATLEN_Pos) |
                       (NRF_ESB_MAX_PAYLOAD_LENGTH      << RADIO_PCNF1_MAXLEN_Pos);
}

//定长发送
static void update_rf_payload_format_esb(uint32_t payload_length)
{
    //设置空中包S0占用1byte，S1占用1bit，LENGTH不占用空间
    NRF_RADIO->PCNF0 = (1 << RADIO_PCNF0_S0LEN_Pos) |
                       (0 << RADIO_PCNF0_LFLEN_Pos) |
                       (1 << RADIO_PCNF0_S1LEN_Pos);
    //失能数据白化
    //大端模式
    //基址长度=5-1=4
    //静态数据长度 = payload_length
    //最大数据长度 = payload_length
    NRF_RADIO->PCNF1 = (RADIO_PCNF1_WHITEEN_Disabled    << RADIO_PCNF1_WHITEEN_Pos) |
                       (RADIO_PCNF1_ENDIAN_Big          << RADIO_PCNF1_ENDIAN_Pos)  |
                       ((m_esb_addr.addr_length - 1)    << RADIO_PCNF1_BALEN_Pos)   |
                       (payload_length                  << RADIO_PCNF1_STATLEN_Pos) |
                       (payload_length                  << RADIO_PCNF1_MAXLEN_Pos);
}

//更新Radio地址
static void update_radio_addresses(uint8_t update_mask)
{
    if ((update_mask & NRF_ESB_ADDR_UPDATE_MASK_BASE0) != 0)
    {
        NRF_RADIO->BASE0 = addr_conv(m_esb_addr.base_addr_p0);
    }

    if ((update_mask & NRF_ESB_ADDR_UPDATE_MASK_BASE1) != 0)
    {
        NRF_RADIO->BASE1 = addr_conv(m_esb_addr.base_addr_p1);
    }

    if ((update_mask & NRF_ESB_ADDR_UPDATE_MASK_PREFIX) != 0)
    {
        NRF_RADIO->PREFIX0 = bytewise_bit_swap(&m_esb_addr.pipe_prefixes[0]);
        NRF_RADIO->PREFIX1 = bytewise_bit_swap(&m_esb_addr.pipe_prefixes[4]);
    }
}

//更新发送功率
static void update_radio_tx_power()
{
    NRF_RADIO->TXPOWER = m_config_local.tx_output_power << RADIO_TXPOWER_TXPOWER_Pos;
}

//更新通信速度与等待应答时间
static bool update_radio_bitrate()
{
    NRF_RADIO->MODE = m_config_local.bitrate << RADIO_MODE_MODE_Pos;

    switch (m_config_local.bitrate)
    {
        case NRF_ESB_BITRATE_2MBPS:
#ifdef NRF52_SERIES
        case NRF_ESB_BITRATE_2MBPS_BLE:
#endif
            //2MBPS模式下，等待应答时间为48us
            m_wait_for_ack_timeout_us = RX_WAIT_FOR_ACK_TIMEOUT_US_2MBPS;
            break;

        case NRF_ESB_BITRATE_1MBPS:
            //1MBPS模式下，等待应答时间为73us
            m_wait_for_ack_timeout_us = RX_WAIT_FOR_ACK_TIMEOUT_US_1MBPS;
            break;

#ifdef NRF51
        case NRF_ESB_BITRATE_250KBPS:
            m_wait_for_ack_timeout_us = RX_WAIT_FOR_ACK_TIMEOUT_US_250KBPS;
            break;
#endif
        
        case NRF_ESB_BITRATE_1MBPS_BLE:
            m_wait_for_ack_timeout_us = RX_WAIT_FOR_ACK_TIMEOUT_US_1MBPS_BLE;
            break;

        default:
            // Should not be reached
            return false;
    }
    return true;
}

//根据发送是定长还是变长模式更新负载长度函数
static bool update_radio_protocol()
{
    switch (m_config_local.protocol)
    {
        case NRF_ESB_PROTOCOL_ESB_DPL:
            //变长发送
            update_rf_payload_format = update_rf_payload_format_esb_dpl;
            break;

        case NRF_ESB_PROTOCOL_ESB:
            //定长发送
            update_rf_payload_format = update_rf_payload_format_esb;
            break;

        default:
            // Should not be reached
            return false;
    }
    return true;
}

//更新CRC模块参数设置
static bool update_radio_crc()
{
    switch(m_config_local.crc)
    {
        case NRF_ESB_CRC_16BIT:
            NRF_RADIO->CRCINIT = 0xFFFFUL;      // Initial value
            NRF_RADIO->CRCPOLY = 0x11021UL;     // CRC poly: x^16+x^12^x^5+1
            break;
        
        case NRF_ESB_CRC_8BIT:
            NRF_RADIO->CRCINIT = 0xFFUL;        // Initial value
            NRF_RADIO->CRCPOLY = 0x107UL;       // CRC poly: x^8+x^2^x^1+1
            break;
        
        case NRF_ESB_CRC_OFF:
            break;
        
        default:
            return false;
    }
    NRF_RADIO->CRCCNF = m_config_local.crc << RADIO_CRCCNF_LEN_Pos;
    return true;
}

//更新Radio参数
static bool update_radio_parameters()
{
    bool params_valid = true;
    //更新发送功率
    update_radio_tx_power();
    //更新通信速度和等待应答的时间
    params_valid &= update_radio_bitrate();
    //更新空中包参数设置函数指针
    params_valid &= update_radio_protocol();
    //更新CRC模块参数设置
    params_valid &= update_radio_crc();
    //根据esb本地参数设置更新空中包参数设置
    update_rf_payload_format(m_config_local.payload_length);
    //判断设置重传延时是否>=最小重传延时
    params_valid &= (m_config_local.retransmit_delay >= NRF_ESB_RETRANSMIT_DELAY_MIN);
    return params_valid;
}

//重置接收和发送队列
static void reset_fifos()
{
    m_tx_fifo.entry_point = 0;
    m_tx_fifo.exit_point  = 0;
    m_tx_fifo.count       = 0;

    m_rx_fifo.entry_point = 0;
    m_rx_fifo.exit_point  = 0;
    m_rx_fifo.count       = 0;
}

//初始化接收和发送队列
static void initialize_fifos()
{
    //重置队列
    reset_fifos();
    //负载指针数组赋值
    for (int i = 0; i < NRF_ESB_TX_FIFO_SIZE; i++)
    {
        m_tx_fifo.p_payload[i] = &m_tx_fifo_payload[i];
    }

    for (int i = 0; i < NRF_ESB_RX_FIFO_SIZE; i++)
    {
        m_rx_fifo.p_payload[i] = &m_rx_fifo_payload[i];
    }
}

//更新发送队列信息
uint32_t nrf_esb_skip_tx()
{
    //判断是否初始化
    VERIFY_TRUE(m_esb_initialized, NRF_ERROR_INVALID_STATE);
    //判断发送队列是否为0
    VERIFY_TRUE(m_tx_fifo.count > 0, NRF_ERROR_BUFFER_EMPTY);
    //失能Radio中断
    DISABLE_RF_IRQ();
    //发送队列元素-1.读指针+1，如果超过最大值设置=0从头开始
    m_tx_fifo.count--;
    if (++m_tx_fifo.exit_point >= NRF_ESB_TX_FIFO_SIZE)
    {
        m_tx_fifo.exit_point = 0;
    }
    //使能Radio中断
    ENABLE_RF_IRQ();

    return NRF_SUCCESS;
}

/** @brief  Function to push the content of the rx_buffer to the RX FIFO.
 *
 *  The module will point the register NRF_RADIO->PACKETPTR to a buffer for receiving packets.
 *  After receiving a packet the module will call this function to copy the received data to
 *  the RX FIFO.
 *
 *  @param  pipe Pipe number to set for the packet.
 *  @param  pid  Packet ID.
 *
 *  @retval true   Operation successful.
 *  @retval false  Operation failed.
 */
//把 m_rx_payload_buffer 里的数据放进接收队列里，设置管道号和包号
static bool rx_fifo_push_rfbuf(uint8_t pipe, uint8_t pid)
{
    //如果接收队列还没满
    if (m_rx_fifo.count < NRF_ESB_RX_FIFO_SIZE)
    {   
        //如果是变长模式
        if (m_config_local.protocol == NRF_ESB_PROTOCOL_ESB_DPL)
        {   
            //判断首字节(实际负载数据长度)是否大于NRF_ESB_MAX_PAYLOAD_LENGTH
            if (m_rx_payload_buffer[0] > NRF_ESB_MAX_PAYLOAD_LENGTH)
            {
                return false;
            }
            //没有超过则设置待写入队列中的长度为首字节的值
            m_rx_fifo.p_payload[m_rx_fifo.entry_point]->length = m_rx_payload_buffer[0];
        }
        //如果定长模式，且处在发送模式
        else if (m_config_local.mode == NRF_ESB_MODE_PTX)
        {
            // Received packet is an acknowledgment
            //认为收到的是应答包
            m_rx_fifo.p_payload[m_rx_fifo.entry_point]->length = 0;
        }
        //如果是定长模式，且不在发送模式
        else
        {
            //认为接收到的数据包是固定长度
            m_rx_fifo.p_payload[m_rx_fifo.entry_point]->length = m_config_local.payload_length;
        }
        //把真正的负载数据复制到队列中(从第2个字节开始)
        memcpy(m_rx_fifo.p_payload[m_rx_fifo.entry_point]->data, &m_rx_payload_buffer[2],
               m_rx_fifo.p_payload[m_rx_fifo.entry_point]->length);
        //设置接收队列中该包的管道号/信号强度/包ID/是否需要应答
        m_rx_fifo.p_payload[m_rx_fifo.entry_point]->pipe  = pipe;
        m_rx_fifo.p_payload[m_rx_fifo.entry_point]->rssi  = NRF_RADIO->RSSISAMPLE;
        m_rx_fifo.p_payload[m_rx_fifo.entry_point]->pid   = pid;
        m_rx_fifo.p_payload[m_rx_fifo.entry_point]->noack = !(m_rx_payload_buffer[1] & 0x01);
        //移动接收队列写索引
        if (++m_rx_fifo.entry_point >= NRF_ESB_RX_FIFO_SIZE)
        {
            m_rx_fifo.entry_point = 0;
        }
        m_rx_fifo.count++;

        return true;
    }

    return false;
}

//ESB系统定时器初始化
static void sys_timer_init()
{
    // Configure the system timer with a 1 MHz base frequency
    //预分频16/2^4 = 1MHz
    NRF_ESB_SYS_TIMER->PRESCALER = 4;
    //16位模式
    NRF_ESB_SYS_TIMER->BITMODE   = TIMER_BITMODE_BITMODE_16Bit;
    //快捷方式：启用溢出即停止和清0
    NRF_ESB_SYS_TIMER->SHORTS    = TIMER_SHORTS_COMPARE1_CLEAR_Msk | TIMER_SHORTS_COMPARE1_STOP_Msk;
}

//可编程外设互联初始化
static void ppi_init()
{
    //绑定READY事件触发定时器计时开始任务
    NRF_PPI->CH[NRF_ESB_PPI_TIMER_START].EEP = (uint32_t)&NRF_RADIO->EVENTS_READY;
    NRF_PPI->CH[NRF_ESB_PPI_TIMER_START].TEP = (uint32_t)&NRF_ESB_SYS_TIMER->TASKS_START;
    //绑定ADDRESS事件触发定时器计失能任务
    NRF_PPI->CH[NRF_ESB_PPI_TIMER_STOP].EEP  = (uint32_t)&NRF_RADIO->EVENTS_ADDRESS;
    NRF_PPI->CH[NRF_ESB_PPI_TIMER_STOP].TEP  = (uint32_t)&NRF_ESB_SYS_TIMER->TASKS_SHUTDOWN;
    //绑定定时器比较匹配事件0触发Radio失能任务
    NRF_PPI->CH[NRF_ESB_PPI_RX_TIMEOUT].EEP  = (uint32_t)&NRF_ESB_SYS_TIMER->EVENTS_COMPARE[0];
    NRF_PPI->CH[NRF_ESB_PPI_RX_TIMEOUT].TEP  = (uint32_t)&NRF_RADIO->TASKS_DISABLE;
    //绑定定时器比较匹配事件1触发Radio发送任务使能任务
    NRF_PPI->CH[NRF_ESB_PPI_TX_START].EEP    = (uint32_t)&NRF_ESB_SYS_TIMER->EVENTS_COMPARE[1];
    NRF_PPI->CH[NRF_ESB_PPI_TX_START].TEP    = (uint32_t)&NRF_RADIO->TASKS_TXEN;
}

//开始发送
static void start_tx_transaction()
{
    bool ack;
    //最后一包发送次数=1
    m_last_tx_attempts = 1;
    // Prepare the payload
    //当前待发送数据包为发送队列读指针所指向数据包
    mp_current_payload = m_tx_fifo.p_payload[m_tx_fifo.exit_point];

    //判断是发送协议
    switch (m_config_local.protocol)
    {
        //定长发送
        case NRF_ESB_PROTOCOL_ESB:
            //更新空中包参数
            update_rf_payload_format(mp_current_payload->length);
            //负载前两个字节用作包ID
            m_tx_payload_buffer[0] = mp_current_payload->pid;
            m_tx_payload_buffer[1] = 0;
            //从第2个字节才是真正的负载数据
            memcpy(&m_tx_payload_buffer[2], mp_current_payload->data, mp_current_payload->length);
            //启用通用快捷方式与DISABLED-RXEN之间的快捷方式
            NRF_RADIO->SHORTS   = m_radio_shorts_common | RADIO_SHORTS_DISABLED_RXEN_Msk;
            //启用DISABLED/READY事件中断
            NRF_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Msk | RADIO_INTENSET_READY_Msk;

            // Configure the retransmit counter
            //设置当前重传次数
            m_retransmits_remaining = m_config_local.retransmit_count;
            //DISABLED事件回调为 on_radio_disabled_tx
            on_radio_disabled = on_radio_disabled_tx;
            //ESB状态为 NRF_ESB_STATE_PTX_TX_ACK
            m_nrf_esb_mainstate = NRF_ESB_STATE_PTX_TX_ACK;
            break;
        //变长发送
        case NRF_ESB_PROTOCOL_ESB_DPL:
            // ack为true时需要应答，ack为false时不需要应答
            //需要判断是否开启了自动请求应答和该包是否需要应答
            //如果开启了自动请求应答，则无论次包是否需要应答，都需要接收方应答
            //如果关闭了自动请求应答，则只要该包需要应答时，才会等待接收方应答
            ack = !mp_current_payload->noack || !m_config_local.selective_auto_ack;
            //负载[0]为包长度
            //负载[1]高7位为包ID，最低位0表示不需要接收方应答，1表示需要应答
            m_tx_payload_buffer[0] = mp_current_payload->length;
            m_tx_payload_buffer[1] = mp_current_payload->pid << 1;
            m_tx_payload_buffer[1] |= mp_current_payload->noack ? 0x00 : 0x01;
            //真正的负载数据从第2个字节开始
            memcpy(&m_tx_payload_buffer[2], mp_current_payload->data, mp_current_payload->length);

            // Handling ack if noack is set to false or if selective auto ack is turned off
            if (ack)//如果需要应答
            {   
                //启用通用快捷方式与DISABLED-RXEN之间的快捷方式
                NRF_RADIO->SHORTS   = m_radio_shorts_common | RADIO_SHORTS_DISABLED_RXEN_Msk;
                //启用DISABLED/READY事件中断
                NRF_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Msk | RADIO_INTENSET_READY_Msk;

                // Configure the retransmit counter
                //设置当前重传次数
                m_retransmits_remaining = m_config_local.retransmit_count;
                //DISABLED事件回调为 on_radio_disabled_tx
                on_radio_disabled = on_radio_disabled_tx;
                //ESB状态为 NRF_ESB_STATE_PTX_TX_ACK
                m_nrf_esb_mainstate = NRF_ESB_STATE_PTX_TX_ACK;
            }
            else//如果不需要应答
            {
                //启用通用快捷方式，而不使用DISABLED-RXEN之间的快捷方式
                NRF_RADIO->SHORTS   = m_radio_shorts_common;
                //启用DISABLED事件中断
                NRF_RADIO->INTENSET = RADIO_INTENSET_DISABLED_Msk;
                //DISABLED事件回调为 on_radio_disabled_tx_noack
                on_radio_disabled   = on_radio_disabled_tx_noack;
                //ESB状态为 NRF_ESB_STATE_PTX_TX
                m_nrf_esb_mainstate = NRF_ESB_STATE_PTX_TX;
            }
            break;

        default:
            // Should not be reached
            break;
    }
    //设置发送时的逻辑地址(0~7)
    NRF_RADIO->TXADDRESS    = mp_current_payload->pipe;
    //使能接收时的逻辑地址，bit0~7表示逻辑地址0~7，置1表示使能
    NRF_RADIO->RXADDRESSES  = 1 << mp_current_payload->pipe;
    //发送频率即channel
    NRF_RADIO->FREQUENCY    = m_esb_addr.rf_channel;
    //设置发送负载指针指向当前待发送包
    NRF_RADIO->PACKETPTR    = (uint32_t)m_tx_payload_buffer;
    //清除Radio中断
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    //使能Radio中断
    NVIC_EnableIRQ(RADIO_IRQn);
    //清除ADDRESS事件、PAYLOAD事件、DISABLED事件中断
    NRF_RADIO->EVENTS_ADDRESS = 0;
    NRF_RADIO->EVENTS_PAYLOAD = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;
    
    DEBUG_PIN_SET(DEBUGPIN4);
    //启动发送任务
    NRF_RADIO->TASKS_TXEN  = 1;
}

//不需要应答的DISABLED事件回调函数
static void on_radio_disabled_tx_noack()
{
    //中断状态为发送成功
    m_interrupt_flags |= NRF_ESB_INT_TX_SUCCESS_MSK;
    //更新发送队列信息
    (void) nrf_esb_skip_tx();
    //如果发送队列为空
    if (m_tx_fifo.count == 0)
    {
        //更改ESB状态为 NRF_ESB_STATE_IDLE
        m_nrf_esb_mainstate = NRF_ESB_STATE_IDLE;
        //调用 ESB_EVT_IRQ 中断
        NVIC_SetPendingIRQ(ESB_EVT_IRQ);
    }
    else//如果发送队列不为空
    {
        //调用 ESB_EVT_IRQ 中断
        NVIC_SetPendingIRQ(ESB_EVT_IRQ);
        //继续传输
        start_tx_transaction();
    }
}

//如果需要应答的DISABLED事件的回调函数
static void on_radio_disabled_tx()
{
    // Remove the DISABLED -> RXEN shortcut, to make sure the radio stays
    // disabled after the RX window
    //重新设置快捷方式，取消了 RADIO_SHORTS_DISABLED_RXEN_Msk ，确保DISABLED事件之后不会直接进入接收状态
    NRF_RADIO->SHORTS           = m_radio_shorts_common;

    // Make sure the timer is started the next time the radio is ready,
    // and that it will disable the radio automatically if no packet is
    // received by the time defined in m_wait_for_ack_timeout_us
    // 确保Radio下个READY事件前，定时器已经启动，如果在 m_wait_for_ack_timeout_us 
    // 时间内没有收到任何数据，则失能Radio
    //比较事件0定时时间为等待应答的时间
    NRF_ESB_SYS_TIMER->CC[0]    = m_wait_for_ack_timeout_us;
    //比较事件1定时时间为等待重传的时间-130
    NRF_ESB_SYS_TIMER->CC[1]    = m_config_local.retransmit_delay - 130;
    //清除定时器计数
    NRF_ESB_SYS_TIMER->TASKS_CLEAR = 1;
    //清除CC[0]比较事件
    NRF_ESB_SYS_TIMER->EVENTS_COMPARE[0] = 0;
    //清除CC[1]比较事件
    NRF_ESB_SYS_TIMER->EVENTS_COMPARE[1] = 0;
    //使能PPI[10,11,12]通道
    NRF_PPI->CHENSET            = (1 << NRF_ESB_PPI_TIMER_START) |
                                  (1 << NRF_ESB_PPI_RX_TIMEOUT) |
                                  (1 << NRF_ESB_PPI_TIMER_STOP);
    //失能PPI[13]通道
    NRF_PPI->CHENCLR            = (1 << NRF_ESB_PPI_TX_START);
    //清除Radio END事件
    NRF_RADIO->EVENTS_END       = 0;
    //如果是定长模式
    if (m_config_local.protocol == NRF_ESB_PROTOCOL_ESB)
    {
        //设置发送长度为0
        update_rf_payload_format(0);
    }
    //Radio包指针指向接收缓存
    NRF_RADIO->PACKETPTR        = (uint32_t)m_rx_payload_buffer;
    //DISABLED事件回调更改为 on_radio_disabled_tx_wait_for_ack
    on_radio_disabled           = on_radio_disabled_tx_wait_for_ack;
    //更改ESB当前状态为等待
    m_nrf_esb_mainstate         = NRF_ESB_STATE_PTX_RX_ACK;
}

//等待应答时DISABLED事件的回调函数
static void on_radio_disabled_tx_wait_for_ack()
{
    // This marks the completion of a TX_RX sequence (TX with ACK)
    
    // Make sure the timer will not deactivate the radio if a packet is received
    //这个函数是被调用的时候意味着带TX-RX整个序列完成
    //如果在RX模式下收到数据包，确保定时器不会关闭Radio

    //失能PPI[10,11,12]通道
    NRF_PPI->CHENCLR = (1 << NRF_ESB_PPI_TIMER_START) |
                       (1 << NRF_ESB_PPI_RX_TIMEOUT)  |
                       (1 << NRF_ESB_PPI_TIMER_STOP);

    // If the radio has received a packet and the CRC status is OK
    //如果END事件置位并且CRC校验通过
    if (NRF_RADIO->EVENTS_END && NRF_RADIO->CRCSTATUS != 0)
    {
        //关闭定时器
        NRF_ESB_SYS_TIMER->TASKS_SHUTDOWN = 1;
        //失能PPI[13]通道
        NRF_PPI->CHENCLR = (1 << NRF_ESB_PPI_TX_START);
        //中断状态设置 NRF_ESB_INT_TX_SUCCESS_MSK
        m_interrupt_flags |= NRF_ESB_INT_TX_SUCCESS_MSK;
        //尝试重传的次数 = 最大重传次数 - 剩余尝试重传的次数 + 1
        m_last_tx_attempts = m_config_local.retransmit_count - m_retransmits_remaining + 1;
        //更新发送队列信息
        (void) nrf_esb_skip_tx();
        //如果非定长发送模式并且接收到的数据包载荷首个字节>0(首字节表示真正载荷长度)
        if (m_config_local.protocol != NRF_ESB_PROTOCOL_ESB && m_rx_payload_buffer[0] > 0)
        {
            //把接收到的数据包放到接收队列并设置对应管道和包ID
            if (rx_fifo_push_rfbuf((uint8_t)NRF_RADIO->TXADDRESS, m_rx_payload_buffer[1] >> 1))
            {
                //中断状态追加 NRF_ESB_INT_RX_DATA_RECEIVED_MSK
                m_interrupt_flags |= NRF_ESB_INT_RX_DATA_RECEIVED_MSK;
            }
        }
        //如果发送队列为空且发送模式为手动发送模式
        if ((m_tx_fifo.count == 0) || (m_config_local.tx_mode == NRF_ESB_TXMODE_MANUAL))
        {
            //更改当前ESB状态为 NRF_ESB_STATE_IDLE
            m_nrf_esb_mainstate = NRF_ESB_STATE_IDLE;
            //调用ESB事件中断
            NVIC_SetPendingIRQ(ESB_EVT_IRQ);
        }
        else
        {
            //调用ESB事件中断
            NVIC_SetPendingIRQ(ESB_EVT_IRQ);
            //再次开始传输
            start_tx_transaction();
        }
    }
    else//如果END事件没有置位或CRC校验失败
    {
        //当前剩余重传次数--
        if (m_retransmits_remaining-- == 0)//如果=0
        {
            //关闭定时器
            NRF_ESB_SYS_TIMER->TASKS_SHUTDOWN = 1;
            //失能清除PPI[13]
            NRF_PPI->CHENCLR = (1 << NRF_ESB_PPI_TX_START);
            // All retransmits are expended, and the TX operation is suspended
            //尝试重传次数 = 重传次数+1
            m_last_tx_attempts = m_config_local.retransmit_count + 1;
            //中断状态追加 NRF_ESB_INT_TX_FAILED_MSK
            m_interrupt_flags |= NRF_ESB_INT_TX_FAILED_MSK;
            //更改ESB状态为 NRF_ESB_STATE_IDLE
            m_nrf_esb_mainstate = NRF_ESB_STATE_IDLE;
            //调用ESB事件中断
            NVIC_SetPendingIRQ(ESB_EVT_IRQ);
        }
        else//如果还有剩余重传次数
        {
            // There are still more retransmits left, TX mode should be
            // entered again as soon as the system timer reaches CC[1].
            //TX模式下在每次定时器CC[1]事件触发时重传
            //启动通用快捷方式与DISABLED-RXEN快捷方式
            NRF_RADIO->SHORTS = m_radio_shorts_common | RADIO_SHORTS_DISABLED_RXEN_Msk;
            //更新待发送空中包长度等参数
            update_rf_payload_format(mp_current_payload->length);
            //修改待发送包指针指向待发送负载
            NRF_RADIO->PACKETPTR = (uint32_t)m_tx_payload_buffer;
            //修改DISABLED事件回调为 on_radio_disabled_tx
            on_radio_disabled = on_radio_disabled_tx;
            //更改ESB状态为 NRF_ESB_STATE_PTX_TX_ACK
            m_nrf_esb_mainstate = NRF_ESB_STATE_PTX_TX_ACK;
            //启用定时器
            NRF_ESB_SYS_TIMER->TASKS_START = 1;
            //使能PPI[13]通道
            NRF_PPI->CHENSET = (1 << NRF_ESB_PPI_TX_START);
            //如果定时器比较事件1已经置位则直接使能发送
            if (NRF_ESB_SYS_TIMER->EVENTS_COMPARE[1])
            {
                NRF_RADIO->TASKS_TXEN = 1;
            }
        }
    }
}
//关闭接收并再次开始接收
static void clear_events_restart_rx(void)
{
    //相较于刚启用接收，取消了DISABLED-TXEN之间的快捷方式
    NRF_RADIO->SHORTS = m_radio_shorts_common;
    //更新空中负载长度相关设置
    update_rf_payload_format(m_config_local.payload_length);
    //把包指针指向当前接收缓存
    NRF_RADIO->PACKETPTR = (uint32_t)m_rx_payload_buffer;
    //清除 DISABLED 事件
    NRF_RADIO->EVENTS_DISABLED = 0;
    //启动 DISABLE 任务
    NRF_RADIO->TASKS_DISABLE = 1;
    //等待Radio失能
    while (NRF_RADIO->EVENTS_DISABLED == 0);
    //清除 DISABLED 事件
    NRF_RADIO->EVENTS_DISABLED = 0;
    //再次启用DISABLED-TXEN之间的快捷方式
    NRF_RADIO->SHORTS = m_radio_shorts_common | RADIO_SHORTS_DISABLED_TXEN_Msk;
    //再次开始接收
    NRF_RADIO->TASKS_RXEN = 1;
}
//接收时，DISABLED事件回调处理函数
static void on_radio_disabled_rx(void)
{
    bool            ack                = false;
    bool            retransmit_payload = false;
    bool            send_rx_event      = true;
    pipe_info_t *   p_pipe_info;
    //如果接收到数据CRC校验失败
    if (NRF_RADIO->CRCSTATUS == 0)
    {
        //关闭接收再重启
        clear_events_restart_rx();
        return;
    }
    //如果接收队列已满
    if (m_rx_fifo.count >= NRF_ESB_RX_FIFO_SIZE)
    {
        //关闭接收再重启？
        clear_events_restart_rx();
        return;
    }
    //接收队列未满
    //NRF_RADIO->RXMATCH为接收到数据包的逻辑地址，即管道号
    p_pipe_info = &m_rx_pipe_info[NRF_RADIO->RXMATCH];
    //如果接收到的数据包的CRC和包号和该管道最后一次接收到的相同，认为是重复包(有可能是上次接收成功但是应答失败)
    if (NRF_RADIO->RXCRC             == p_pipe_info->crc &&
        (m_rx_payload_buffer[1] >> 1) == p_pipe_info->pid
       )
    {
        //是重传包
        retransmit_payload = true;
        //不再调用接收回调
        send_rx_event = false;
    }
    //如果不相同，更新CRC和PID到该管道信息里
    p_pipe_info->pid = m_rx_payload_buffer[1] >> 1;
    p_pipe_info->crc = NRF_RADIO->RXCRC;
    //如果设置了自动应答或者该包数据要求应答，则应答
    if ((m_config_local.selective_auto_ack == false) || ((m_rx_payload_buffer[1] & 0x01) == 1))
    {
        ack = true;
    }

    if (ack)//如果需要应答此包数据
    {
        //启用DISABLED-RXEN之间的快捷方式，为了发送完继续接收
        NRF_RADIO->SHORTS = m_radio_shorts_common | RADIO_SHORTS_DISABLED_RXEN_Msk;
        //判断发送协议
        switch (m_config_local.protocol)
        {
            //变长发送
            case NRF_ESB_PROTOCOL_ESB_DPL:
                {
                    //如果发送队列不为空且当前待发送数据的管道号等于当前接收到的此包数据的管道号
                    if (m_tx_fifo.count > 0 &&
                        (m_tx_fifo.p_payload[m_tx_fifo.exit_point]->pipe == NRF_RADIO->RXMATCH)
                       )
                    {
                        // Pipe stays in ACK with payload until TX FIFO is empty
                        // Do not report TX success on first ack payload or retransmit
                        //如果应答携带负载且不是重传包
                        if (p_pipe_info->ack_payload == true && !retransmit_payload)
                        {
                            //更新发送队列读索引
                            if (++m_tx_fifo.exit_point >= NRF_ESB_TX_FIFO_SIZE)
                            {
                                m_tx_fifo.exit_point = 0;
                            }

                            m_tx_fifo.count--;

                            // ACK payloads also require TX_DS
                            // (page 40 of the 'nRF24LE1_Product_Specification_rev1_6.pdf').
                            //追加中断状态 NRF_ESB_INT_TX_SUCCESS_MSK
                            m_interrupt_flags |= NRF_ESB_INT_TX_SUCCESS_MSK;
                        }
                        //应答携带负载
                        p_pipe_info->ack_payload = true;
                        //更新待发送数据为发送队列读索引所指缓存
                        mp_current_payload = m_tx_fifo.p_payload[m_tx_fifo.exit_point];
                        //更新空中包负载长度等
                        update_rf_payload_format(mp_current_payload->length);
                        //首字节赋值为真正负载的长度
                        m_tx_payload_buffer[0] = mp_current_payload->length;
                        memcpy(&m_tx_payload_buffer[2],
                               mp_current_payload->data,
                               mp_current_payload->length);
                    }
                    else//发送队列为空或待发送数据不是当前管道的
                    {
                        //应答不携带负载
                        p_pipe_info->ack_payload = false;
                        //更新空中包长度为0
                        update_rf_payload_format(0);
                        //首字节为0
                        m_tx_payload_buffer[0] = 0;
                    }
                    //发送的第一个字节为接收到的第一个字节
                    m_tx_payload_buffer[1] = m_rx_payload_buffer[1];
                }
                break;
            //定长发送
            case NRF_ESB_PROTOCOL_ESB:
                {
                    //更新空中包长度为0
                    update_rf_payload_format(0);
                    //第0个字节原封返回，第一个字节为0
                    m_tx_payload_buffer[0] = m_rx_payload_buffer[0];
                    m_tx_payload_buffer[1] = 0;
                }
                break;
        }
        //更高ESB状态为NRF_ESB_STATE_PRX_SEND_ACK 接收时应答发送端
        m_nrf_esb_mainstate = NRF_ESB_STATE_PRX_SEND_ACK;
        //发送逻辑地址等于接收，即管道号
        NRF_RADIO->TXADDRESS = NRF_RADIO->RXMATCH;
        //包指针指向发送缓存
        NRF_RADIO->PACKETPTR = (uint32_t)m_tx_payload_buffer;
        //DISABLED 事件回调为 on_radio_disabled_rx_ack
        on_radio_disabled = on_radio_disabled_rx_ack;
    }
    else//如果不需要应答数据包
    {
        //重启接收
        clear_events_restart_rx();
    }
    //如果需要调用接收成功回调
    if (send_rx_event)
    {
        // Push the new packet to the RX buffer and trigger a received event if the operation was
        // successful.
        //把接收到的数据放进接收队列里
        if (rx_fifo_push_rfbuf(NRF_RADIO->RXMATCH, p_pipe_info->pid))
        {
            //追加中断状态 NRF_ESB_INT_RX_DATA_RECEIVED_MSK
            m_interrupt_flags |= NRF_ESB_INT_RX_DATA_RECEIVED_MSK;
            //调用ESB事件回调
            NVIC_SetPendingIRQ(ESB_EVT_IRQ);
        }
    }
}

//发送完应答后 DISABLED 事件的回调函数
static void on_radio_disabled_rx_ack(void)
{
    //启用 DISABLED-TXEN 快捷方式，保证接收完后立即触发发送
    NRF_RADIO->SHORTS = m_radio_shorts_common | RADIO_SHORTS_DISABLED_TXEN_Msk;
    //更新空中包长等
    update_rf_payload_format(m_config_local.payload_length);
    //包指针指向接收缓存
    NRF_RADIO->PACKETPTR = (uint32_t)m_rx_payload_buffer;
    //更新 DISABLED 事件回调为 on_radio_disabled_rx
    on_radio_disabled = on_radio_disabled_rx;
    //更改ESB状态为 NRF_ESB_STATE_PRX
    m_nrf_esb_mainstate = NRF_ESB_STATE_PRX;
}


/**@brief Function for clearing pending interrupts.
 *
 * @param[in,out]   p_interrupts        Pointer to the value that holds the current interrupts.
 *
 * @retval  NRF_SUCCESS                     If the interrupts were cleared successfully.
 * @retval  NRF_ERROR_NULL                  If the required parameter was NULL.
 * @retval  NRF_INVALID_STATE               If the module is not initialized.
 */
//复制当前中断状态给入参，然后清除当前中断状态
static uint32_t nrf_esb_get_clear_interrupts(uint32_t * p_interrupts)
{
    VERIFY_TRUE(m_esb_initialized, NRF_ERROR_INVALID_STATE);
    VERIFY_PARAM_NOT_NULL(p_interrupts);

    DISABLE_RF_IRQ();

    *p_interrupts = m_interrupt_flags;
    m_interrupt_flags = 0;

    ENABLE_RF_IRQ();

    return NRF_SUCCESS;
}

//Radio中断回调函数
void RADIO_IRQHandler()
{
    //READY 事件被置位且 READY 事件中断被使能
    if (NRF_RADIO->EVENTS_READY && (NRF_RADIO->INTENSET & RADIO_INTENSET_READY_Msk))
    {
        //清除 READY 事件
        NRF_RADIO->EVENTS_READY = 0;
        DEBUG_PIN_SET(DEBUGPIN1);
    }
    //END 事件被置位且 END 事件中断被使能
    if (NRF_RADIO->EVENTS_END && (NRF_RADIO->INTENSET & RADIO_INTENSET_END_Msk))
    {
        //清除 END 事件
        NRF_RADIO->EVENTS_END = 0;
        DEBUG_PIN_SET(DEBUGPIN2);

        // Call the correct on_radio_end function, depending on the current protocol state
        //如果有 END 事件回调则调用 on_radio_end
        if (on_radio_end)
        {
            on_radio_end();
        }
    }
    //DISABLED 事件被置位且 DISABLED 事件中断被使能
    if (NRF_RADIO->EVENTS_DISABLED && (NRF_RADIO->INTENSET & RADIO_INTENSET_DISABLED_Msk))
    {
        //清除 DISABLED 事件
        NRF_RADIO->EVENTS_DISABLED = 0;
        DEBUG_PIN_SET(DEBUGPIN3);

        // Call the correct on_radio_disable function, depending on the current protocol state
        //如果有 DISABLED 事件回调则调用 on_radio_disabled
        if (on_radio_disabled)
        {
            on_radio_disabled();
        }
    }

    DEBUG_PIN_CLR(DEBUGPIN1);
    DEBUG_PIN_CLR(DEBUGPIN2);
    DEBUG_PIN_CLR(DEBUGPIN3);
    DEBUG_PIN_CLR(DEBUGPIN4);
}

//esb初始化
uint32_t nrf_esb_init(nrf_esb_config_t const * p_config)
{
    uint32_t err_code;

    VERIFY_PARAM_NOT_NULL(p_config);
    //判断是否已经初始化过
    if (m_esb_initialized)
    {
        //已经初始化了，先失能esb，然后返回
        err_code = nrf_esb_disable();
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }
    nrf_gpio_cfg_output(DEBUGPIN1);
    nrf_gpio_cfg_output(DEBUGPIN2);
    nrf_gpio_cfg_output(DEBUGPIN3);
    nrf_gpio_cfg_output(DEBUGPIN4);
    //设置事件回调
    m_event_handler = p_config->event_handler;
    //复制所设参数到 m_config_local，后续操作 m_config_local
    memcpy(&m_config_local, p_config, sizeof(nrf_esb_config_t));
    //中断状态
    m_interrupt_flags    = 0;
    //清空所有接收管道的管道信息
    memset(m_rx_pipe_info, 0, sizeof(m_rx_pipe_info));
    //清空所有管道的包ID
    memset(m_pids, 0, sizeof(m_pids));
    //调用 update_radio_parameters 更新Radio参数，并检查返回值
    VERIFY_TRUE(update_radio_parameters(), NRF_ERROR_INVALID_PARAM);

    // Configure radio address registers according to ESB default values
    //重新设置了基址和前缀
    NRF_RADIO->BASE0   = 0xE7E7E7E7;
    NRF_RADIO->BASE1   = 0x43434343;
    NRF_RADIO->PREFIX0 = 0x23C343E7;
    NRF_RADIO->PREFIX1 = 0x13E363A3;
    //初始化发送接收队列
    initialize_fifos();
    //ESB系统定时器初始化
    sys_timer_init();
    //可编程外设互联初始化
    ppi_init();
    //设置Radio模块中断优先级
    NVIC_SetPriority(RADIO_IRQn, m_config_local.radio_irq_priority & ESB_IRQ_PRIORITY_MSK);
    //设置ESB事件中断优先级
    NVIC_SetPriority(ESB_EVT_IRQ, m_config_local.event_irq_priority & ESB_IRQ_PRIORITY_MSK);
    //使能ESB事件中断
    NVIC_EnableIRQ(ESB_EVT_IRQ);
    //esb状态：空闲
    m_nrf_esb_mainstate = NRF_ESB_STATE_IDLE;
    //已初始化
    m_esb_initialized = true;


#ifdef NRF52832_XXAA
if ((NRF_FICR->INFO.VARIANT & 0x0000FF00) == 0x00004500) //Check if the device is an nRF52832 Rev. 2.
    //Workaround for nRF52832 rev 2 errata 182
    *(volatile uint32_t *) 0x4000173C |= (1 << 10);
#endif

    return NRF_SUCCESS;
}

//ESB挂起
uint32_t nrf_esb_suspend(void)
{
    //ESB 处于 NRF_ESB_STATE_IDLE 时设置失败，返回 NRF_ERROR_BUSY 
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);

    // Clear PPI
    //失能所有PPI通道
    NRF_PPI->CHENCLR = (1 << NRF_ESB_PPI_TIMER_START) |
                       (1 << NRF_ESB_PPI_TIMER_STOP)  |
                       (1 << NRF_ESB_PPI_RX_TIMEOUT)  |
                       (1 << NRF_ESB_PPI_TX_START);
    //更改当前ESB状态为 NRF_ESB_STATE_IDLE
    m_nrf_esb_mainstate = NRF_ESB_STATE_IDLE;

    return NRF_SUCCESS;
}

//ESB失能
uint32_t nrf_esb_disable(void)
{
    // Clear PPI
    //失能所有PPI通道
    NRF_PPI->CHENCLR = (1 << NRF_ESB_PPI_TIMER_START) |
                       (1 << NRF_ESB_PPI_TIMER_STOP)  |
                       (1 << NRF_ESB_PPI_RX_TIMEOUT)  |
                       (1 << NRF_ESB_PPI_TX_START);
    //更改当前ESB状态为 NRF_ESB_STATE_IDLE
    m_nrf_esb_mainstate = NRF_ESB_STATE_IDLE;
    //初始化状态改为未初始化
    m_esb_initialized = false;
    //复位接收和发送队列
    reset_fifos();
    //复位所有接收管道和包ID信息
    memset(m_rx_pipe_info, 0, sizeof(m_rx_pipe_info));
    memset(m_pids, 0, sizeof(m_pids));

    // Disable the radio
    //失能ESB事件中断
    NVIC_DisableIRQ(ESB_EVT_IRQ);
    //保留了READY-START/END-DISABLE快捷方式
    NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos |
                        RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos;

    return NRF_SUCCESS;
}

//返回ESB是否处于空闲状态
bool nrf_esb_is_idle(void)
{
    return m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE;
}

//ESB事件中断
void ESB_EVT_IRQHandler(void)
{
    ret_code_t      err_code;
    uint32_t        interrupts;
    nrf_esb_evt_t   event;
    //尝试重传次数
    event.tx_attempts = m_last_tx_attempts;
    //
    err_code = nrf_esb_get_clear_interrupts(&interrupts);
    if (err_code == NRF_SUCCESS && m_event_handler != 0)
    {
        //根据当前中断状态依次调用 m_event_handler ，最多调用2次？
        if (interrupts & NRF_ESB_INT_TX_SUCCESS_MSK)
        {
            event.evt_id = NRF_ESB_EVENT_TX_SUCCESS;
            m_event_handler(&event);
        }
        if (interrupts & NRF_ESB_INT_TX_FAILED_MSK)
        {
            event.evt_id = NRF_ESB_EVENT_TX_FAILED;
            m_event_handler(&event);
        }
        if (interrupts & NRF_ESB_INT_RX_DATA_RECEIVED_MSK)
        {
            event.evt_id = NRF_ESB_EVENT_RX_RECEIVED;
            m_event_handler(&event);
        }
    }
}
//写负载到TX队列
uint32_t nrf_esb_write_payload(nrf_esb_payload_t const * p_payload)
{
    //检测是否初始化
    VERIFY_TRUE(m_esb_initialized, NRF_ERROR_INVALID_STATE);
    //检测负载是否为NULL
    VERIFY_PARAM_NOT_NULL(p_payload);
    //检测负载长度
    VERIFY_PAYLOAD_LENGTH(p_payload);
    //检测发送队列是否还有空间可写
    VERIFY_FALSE(m_tx_fifo.count >= NRF_ESB_TX_FIFO_SIZE, NRF_ERROR_NO_MEM);
    //管道号是否在0~7之间
    VERIFY_TRUE(p_payload->pipe < NRF_ESB_PIPE_COUNT, NRF_ERROR_INVALID_PARAM);
    //失能Radio中断
    DISABLE_RF_IRQ();
    //复制负载到发送队列
    memcpy(m_tx_fifo.p_payload[m_tx_fifo.entry_point], p_payload, sizeof(nrf_esb_payload_t));
    //将要发送的负载的管道的包ID+1，包ID在0~3之间循环
    m_pids[p_payload->pipe] = (m_pids[p_payload->pipe] + 1) % (NRF_ESB_PID_MAX + 1);
    //更新被复制到放发送队列中的这个负载的包ID
    m_tx_fifo.p_payload[m_tx_fifo.entry_point]->pid = m_pids[p_payload->pipe];
    //如果发送队列写指针超过最大，从头开始(环形缓冲)
    if (++m_tx_fifo.entry_point >= NRF_ESB_TX_FIFO_SIZE)
    {
        m_tx_fifo.entry_point = 0;
    }
    //发送队列已有元素个数+1
    m_tx_fifo.count++;
    //使能Radio中断
    ENABLE_RF_IRQ();
    //如果当前是发送模式，且是自动发送模式，且ESB处于空闲状态
    if (m_config_local.mode == NRF_ESB_MODE_PTX &&
        m_config_local.tx_mode == NRF_ESB_TXMODE_AUTO &&
        m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE)
    {
        //开始发送
        start_tx_transaction();
    }

    return NRF_SUCCESS;
}


uint32_t nrf_esb_read_rx_payload(nrf_esb_payload_t * p_payload)
{
    VERIFY_TRUE(m_esb_initialized, NRF_ERROR_INVALID_STATE);
    VERIFY_PARAM_NOT_NULL(p_payload);

    if (m_rx_fifo.count == 0)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    DISABLE_RF_IRQ();

    p_payload->length = m_rx_fifo.p_payload[m_rx_fifo.exit_point]->length;
    p_payload->pipe   = m_rx_fifo.p_payload[m_rx_fifo.exit_point]->pipe;
    p_payload->rssi   = m_rx_fifo.p_payload[m_rx_fifo.exit_point]->rssi;
    p_payload->pid    = m_rx_fifo.p_payload[m_rx_fifo.exit_point]->pid;
    p_payload->noack  = m_rx_fifo.p_payload[m_rx_fifo.exit_point]->noack; 
    memcpy(p_payload->data, m_rx_fifo.p_payload[m_rx_fifo.exit_point]->data, p_payload->length);

    if (++m_rx_fifo.exit_point >= NRF_ESB_RX_FIFO_SIZE)
    {
        m_rx_fifo.exit_point = 0;
    }

    m_rx_fifo.count--;

    ENABLE_RF_IRQ();

    return NRF_SUCCESS;
}


uint32_t nrf_esb_start_tx(void)
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);

    if (m_tx_fifo.count == 0)
    {
        return NRF_ERROR_BUFFER_EMPTY;
    }

    start_tx_transaction();

    return NRF_SUCCESS;
}

//ESB开始接收模式
uint32_t nrf_esb_start_rx(void)
{ 
    //ESB状态需要处在 NRF_ESB_STATE_IDLE 
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);
    //清除Radio的所有中断事件
    NRF_RADIO->INTENCLR = 0xFFFFFFFF;
    //清除DISABLED事件
    NRF_RADIO->EVENTS_DISABLED = 0;
    //DISABLED事件回调设置为 on_radio_disabled_rx
    on_radio_disabled = on_radio_disabled_rx;
    //使能通用快捷方式和DISABLED-TXEN之间的快捷方式
    NRF_RADIO->SHORTS      = m_radio_shorts_common | RADIO_SHORTS_DISABLED_TXEN_Msk;
    //使能DISABLED事件中断
    NRF_RADIO->INTENSET    = RADIO_INTENSET_DISABLED_Msk;
    //更改ESB当前状态为 NRF_ESB_STATE_PRX
    m_nrf_esb_mainstate    = NRF_ESB_STATE_PRX;
    //设置使能哪些逻辑地址的接收(按位)
    NRF_RADIO->RXADDRESSES  = m_esb_addr.rx_pipes_enabled;
    //设置接收频率即通道
    NRF_RADIO->FREQUENCY    = m_esb_addr.rf_channel;
    //把包指针执向接收缓存
    NRF_RADIO->PACKETPTR    = (uint32_t)m_rx_payload_buffer;
    //清除Radio中断
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    //使能Radio中断
    NVIC_EnableIRQ(RADIO_IRQn);
    //清除 ADDRESS/PAYLOAD/DISABLED 事件中断
    NRF_RADIO->EVENTS_ADDRESS = 0;
    NRF_RADIO->EVENTS_PAYLOAD = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;
    //启动接收
    NRF_RADIO->TASKS_RXEN  = 1;

    return NRF_SUCCESS;
}


uint32_t nrf_esb_stop_rx(void)
{
    if (m_nrf_esb_mainstate == NRF_ESB_STATE_PRX ||
        m_nrf_esb_mainstate == NRF_ESB_STATE_PRX_SEND_ACK)
    {
        NRF_RADIO->SHORTS = 0;
        NRF_RADIO->INTENCLR = 0xFFFFFFFF;
        on_radio_disabled = NULL;
        NRF_RADIO->EVENTS_DISABLED = 0;
        NRF_RADIO->TASKS_DISABLE = 1;
        while (NRF_RADIO->EVENTS_DISABLED == 0);
        m_nrf_esb_mainstate = NRF_ESB_STATE_IDLE;

        return NRF_SUCCESS;
    }

    return NRF_ESB_ERROR_NOT_IN_RX_MODE;
}


uint32_t nrf_esb_flush_tx(void)
{
    VERIFY_TRUE(m_esb_initialized, NRF_ERROR_INVALID_STATE);

    DISABLE_RF_IRQ();

    m_tx_fifo.count = 0;
    m_tx_fifo.entry_point = 0;
    m_tx_fifo.exit_point = 0;

    ENABLE_RF_IRQ();

    return NRF_SUCCESS;
}


uint32_t nrf_esb_pop_tx(void)
{
    VERIFY_TRUE(m_esb_initialized, NRF_ERROR_INVALID_STATE);
    VERIFY_TRUE(m_tx_fifo.count > 0, NRF_ERROR_BUFFER_EMPTY);

    DISABLE_RF_IRQ();

    if (--m_tx_fifo.entry_point >= NRF_ESB_TX_FIFO_SIZE)
    {
        m_tx_fifo.entry_point = 0;
    }
    m_tx_fifo.count--;

    ENABLE_RF_IRQ();

    return NRF_SUCCESS;
}


uint32_t nrf_esb_flush_rx(void)
{
    VERIFY_TRUE(m_esb_initialized, NRF_ERROR_INVALID_STATE);

    DISABLE_RF_IRQ();

    m_rx_fifo.count = 0;
    m_rx_fifo.entry_point = 0;
    m_rx_fifo.exit_point = 0;

    memset(m_rx_pipe_info, 0, sizeof(m_rx_pipe_info));

    ENABLE_RF_IRQ();

    return NRF_SUCCESS;
}


uint32_t nrf_esb_set_address_length(uint8_t length)
{
    //只能在 NRF_ESB_STATE_IDLE 状态下才能设置
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);
    //长度必须大于2小于6
    VERIFY_TRUE(length > 2 && length < 6, NRF_ERROR_INVALID_PARAM);
    
#ifdef NRF52832_XXAA
    uint32_t base_address_mask = length == 5 ? 0xFFFF0000 : 0xFF000000;
    if ((NRF_FICR->INFO.VARIANT & 0x0000FF00) == 0x00004200)  //Check if the device is an nRF52832 Rev. 1.
    {
        /* 
        Workaround for nRF52832 Rev 1 Errata 107
        Check if pipe 0 or pipe 1-7 has a 'zero address'.
        Avoid using access addresses in the following pattern (where X is don't care): 
        ADDRLEN=5 
        BASE0 = 0x0000XXXX, PREFIX0 = 0xXXXXXX00 
        BASE1 = 0x0000XXXX, PREFIX0 = 0xXXXX00XX 
        BASE1 = 0x0000XXXX, PREFIX0 = 0xXX00XXXX 
        BASE1 = 0x0000XXXX, PREFIX0 = 0x00XXXXXX 
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXXXX00 
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXX00XX 
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXX00XXXX 
        BASE1 = 0x0000XXXX, PREFIX1 = 0x00XXXXXX 

        ADDRLEN=4 
        BASE0 = 0x00XXXXXX, PREFIX0 = 0xXXXXXX00 
        BASE1 = 0x00XXXXXX, PREFIX0 = 0xXXXX00XX 
        BASE1 = 0x00XXXXXX, PREFIX0 = 0xXX00XXXX 
        BASE1 = 0x00XXXXXX, PREFIX0 = 0x00XXXXXX 
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXXXX00 
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXX00XX 
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXX00XXXX 
        BASE1 = 0x00XXXXXX, PREFIX1 = 0x00XXXXXX
        */
        if ((NRF_RADIO->BASE0 & base_address_mask) == 0 && (NRF_RADIO->PREFIX0 & 0x000000FF) == 0)
        {
            return NRF_ERROR_INVALID_PARAM;
        }
        if ((NRF_RADIO->BASE1 & base_address_mask) == 0 && ((NRF_RADIO->PREFIX0 & 0x0000FF00) == 0 ||(NRF_RADIO->PREFIX0 & 0x00FF0000) == 0 || (NRF_RADIO->PREFIX0 & 0xFF000000) == 0 ||
           (NRF_RADIO->PREFIX1 & 0xFF000000) == 0 || (NRF_RADIO->PREFIX1 & 0x00FF0000) == 0 ||(NRF_RADIO->PREFIX1 & 0x0000FF00) == 0 || (NRF_RADIO->PREFIX1 & 0x000000FF) == 0))
        {
            return NRF_ERROR_INVALID_PARAM;
        }
    }
#endif

    m_esb_addr.addr_length = length;

    update_rf_payload_format(m_config_local.payload_length);

#ifdef NRF52832_XXAA
    if ((NRF_FICR->INFO.VARIANT & 0x0000FF00) == 0x00004500)  //Check if the device is an nRF52832 Rev. 2.
    {
        return apply_address_workarounds();
    }
    else
    {
        return NRF_SUCCESS;
    }
#else
    return NRF_SUCCESS;
#endif
}

//设置PIPE0的基址
uint32_t nrf_esb_set_base_address_0(uint8_t const * p_addr)
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);
    VERIFY_PARAM_NOT_NULL(p_addr);

#ifdef NRF52832_XXAA
    if ((NRF_FICR->INFO.VARIANT & 0x0000FF00) == 0x00004200)  //Check if the device is an nRF52832 Rev. 1.
    {
        /*
        Workaround for nRF52832 Rev 1 Errata 107
        Check if pipe 0 or pipe 1-7 has a 'zero address'.
        Avoid using access addresses in the following pattern (where X is don't care): 
        ADDRLEN=5 
        BASE0 = 0x0000XXXX, PREFIX0 = 0xXXXXXX00 
        BASE1 = 0x0000XXXX, PREFIX0 = 0xXXXX00XX 
        BASE1 = 0x0000XXXX, PREFIX0 = 0xXX00XXXX 
        BASE1 = 0x0000XXXX, PREFIX0 = 0x00XXXXXX 
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXXXX00 
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXX00XX 
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXX00XXXX 
        BASE1 = 0x0000XXXX, PREFIX1 = 0x00XXXXXX 

        ADDRLEN=4 
        BASE0 = 0x00XXXXXX, PREFIX0 = 0xXXXXXX00 
        BASE1 = 0x00XXXXXX, PREFIX0 = 0xXXXX00XX 
        BASE1 = 0x00XXXXXX, PREFIX0 = 0xXX00XXXX 
        BASE1 = 0x00XXXXXX, PREFIX0 = 0x00XXXXXX 
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXXXX00 
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXX00XX 
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXX00XXXX 
        BASE1 = 0x00XXXXXX, PREFIX1 = 0x00XXXXXX
        */
        uint32_t base_address_mask = m_esb_addr.addr_length == 5 ? 0xFFFF0000 : 0xFF000000;
        if ((addr_conv(p_addr) & base_address_mask) == 0 && (NRF_RADIO->PREFIX0 & 0x000000FF) == 0)
        {
            return NRF_ERROR_INVALID_PARAM;
        }
    }
#endif



    memcpy(m_esb_addr.base_addr_p0, p_addr, 4);

    update_radio_addresses(NRF_ESB_ADDR_UPDATE_MASK_BASE0);
#ifdef NRF52832_XXAA
    return apply_address_workarounds();
#else
    return NRF_SUCCESS;
#endif
}

//设置PIPE1~7的基址
uint32_t nrf_esb_set_base_address_1(uint8_t const * p_addr)
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);
    VERIFY_PARAM_NOT_NULL(p_addr);

#ifdef NRF52832_XXAA
    if ((NRF_FICR->INFO.VARIANT & 0x0000FF00) == 0x00004200)  //Check if the device is an nRF52832 Rev. 1.
    {
        /*
        Workaround for nRF52832 Rev 1 Errata 107
        Check if pipe 0 or pipe 1-7 has a 'zero address'.
        Avoid using access addresses in the following pattern (where X is don't care): 
        ADDRLEN=5 
        BASE0 = 0x0000XXXX, PREFIX0 = 0xXXXXXX00
        BASE1 = 0x0000XXXX, PREFIX0 = 0xXXXX00XX
        BASE1 = 0x0000XXXX, PREFIX0 = 0xXX00XXXX
        BASE1 = 0x0000XXXX, PREFIX0 = 0x00XXXXXX
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXXXX00
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXX00XX
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXX00XXXX
        BASE1 = 0x0000XXXX, PREFIX1 = 0x00XXXXXX

        ADDRLEN=4 
        BASE0 = 0x00XXXXXX, PREFIX0 = 0xXXXXXX00
        BASE1 = 0x00XXXXXX, PREFIX0 = 0xXXXX00XX
        BASE1 = 0x00XXXXXX, PREFIX0 = 0xXX00XXXX
        BASE1 = 0x00XXXXXX, PREFIX0 = 0x00XXXXXX
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXXXX00
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXX00XX
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXX00XXXX
        BASE1 = 0x00XXXXXX, PREFIX1 = 0x00XXXXXX
        */
        uint32_t base_address_mask = m_esb_addr.addr_length == 5 ? 0xFFFF0000 : 0xFF000000;
        if ((addr_conv(p_addr) & base_address_mask) == 0 &&
            ((NRF_RADIO->PREFIX0 & 0x0000FF00) == 0 ||(NRF_RADIO->PREFIX0 & 0x00FF0000) == 0 ||
            (NRF_RADIO->PREFIX0 & 0xFF000000) == 0 || (NRF_RADIO->PREFIX1 & 0xFF000000) == 0 ||
            (NRF_RADIO->PREFIX1 & 0x00FF0000) == 0 ||(NRF_RADIO->PREFIX1 & 0x0000FF00) == 0 ||
            (NRF_RADIO->PREFIX1 & 0x000000FF) == 0))
        {
            return NRF_ERROR_INVALID_PARAM;
        }
    }
#endif

    memcpy(m_esb_addr.base_addr_p1, p_addr, 4);

    update_radio_addresses(NRF_ESB_ADDR_UPDATE_MASK_BASE1);

#ifdef NRF52832_XXAA
    return apply_address_workarounds();
#else
    return NRF_SUCCESS;
#endif
}

//设置PIPE0~7的前缀
uint32_t nrf_esb_set_prefixes(uint8_t const * p_prefixes, uint8_t num_pipes)
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);
    VERIFY_PARAM_NOT_NULL(p_prefixes);
    VERIFY_TRUE(num_pipes <= NRF_ESB_PIPE_COUNT, NRF_ERROR_INVALID_PARAM);
    
#ifdef NRF52832_XXAA
    if ((NRF_FICR->INFO.VARIANT & 0x0000FF00) == 0x00004200)  //Check if the device is an nRF52832 Rev. 1.
    {
        /*
        Workaround for nRF52832 Rev 1 Errata 107
        Check if pipe 0 or pipe 1-7 has a 'zero address'.
        Avoid using access addresses in the following pattern (where X is don't care): 
        ADDRLEN=5 
        BASE0 = 0x0000XXXX, PREFIX0 = 0xXXXXXX00 
        BASE1 = 0x0000XXXX, PREFIX0 = 0xXXXX00XX 
        BASE1 = 0x0000XXXX, PREFIX0 = 0xXX00XXXX 
        BASE1 = 0x0000XXXX, PREFIX0 = 0x00XXXXXX 
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXXXX00 
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXX00XX 
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXX00XXXX 
        BASE1 = 0x0000XXXX, PREFIX1 = 0x00XXXXXX 

        ADDRLEN=4 
        BASE0 = 0x00XXXXXX, PREFIX0 = 0xXXXXXX00 
        BASE1 = 0x00XXXXXX, PREFIX0 = 0xXXXX00XX 
        BASE1 = 0x00XXXXXX, PREFIX0 = 0xXX00XXXX 
        BASE1 = 0x00XXXXXX, PREFIX0 = 0x00XXXXXX 
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXXXX00 
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXX00XX 
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXX00XXXX 
        BASE1 = 0x00XXXXXX, PREFIX1 = 0x00XXXXXX
        */
        uint32_t base_address_mask = m_esb_addr.addr_length == 5 ? 0xFFFF0000 : 0xFF000000;
        if (num_pipes >= 1 && (NRF_RADIO->BASE0 & base_address_mask) == 0 && p_prefixes[0] == 0)
        {
            return NRF_ERROR_INVALID_PARAM;
        }

        if ((NRF_RADIO->BASE1 & base_address_mask) == 0)
        {
            for (uint8_t i = 1; i < num_pipes; i++)
            {
                if (p_prefixes[i] == 0)
                {
                    return NRF_ERROR_INVALID_PARAM;
                }
            }
        }
    }
#endif
    
    memcpy(m_esb_addr.pipe_prefixes, p_prefixes, num_pipes);
    m_esb_addr.num_pipes = num_pipes;
    m_esb_addr.rx_pipes_enabled = BIT_MASK_UINT_8(num_pipes);

    update_radio_addresses(NRF_ESB_ADDR_UPDATE_MASK_PREFIX);

#ifdef NRF52832_XXAA
    return apply_address_workarounds();
#else
    return NRF_SUCCESS;
#endif
}


uint32_t nrf_esb_update_prefix(uint8_t pipe, uint8_t prefix)
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);
    VERIFY_TRUE(pipe < NRF_ESB_PIPE_COUNT, NRF_ERROR_INVALID_PARAM);
    
#ifdef NRF52832_XXAA
    if ((NRF_FICR->INFO.VARIANT & 0x0000FF00) == 0x00004200)  //Check if the device is an nRF52832 Rev. 1.
    {
        /*
        Workaround for nRF52832 Rev 1 Errata 107
        Check if pipe 0 or pipe 1-7 has a 'zero address'.
        Avoid using access addresses in the following pattern (where X is don't care): 
        ADDRLEN=5 
        BASE0 = 0x0000XXXX, PREFIX0 = 0xXXXXXX00 
        BASE1 = 0x0000XXXX, PREFIX0 = 0xXXXX00XX 
        BASE1 = 0x0000XXXX, PREFIX0 = 0xXX00XXXX 
        BASE1 = 0x0000XXXX, PREFIX0 = 0x00XXXXXX 
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXXXX00 
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXXXX00XX 
        BASE1 = 0x0000XXXX, PREFIX1 = 0xXX00XXXX 
        BASE1 = 0x0000XXXX, PREFIX1 = 0x00XXXXXX 

        ADDRLEN=4 
        BASE0 = 0x00XXXXXX, PREFIX0 = 0xXXXXXX00 
        BASE1 = 0x00XXXXXX, PREFIX0 = 0xXXXX00XX 
        BASE1 = 0x00XXXXXX, PREFIX0 = 0xXX00XXXX 
        BASE1 = 0x00XXXXXX, PREFIX0 = 0x00XXXXXX 
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXXXX00 
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXXXX00XX 
        BASE1 = 0x00XXXXXX, PREFIX1 = 0xXX00XXXX 
        BASE1 = 0x00XXXXXX, PREFIX1 = 0x00XXXXXX
        */
        uint32_t base_address_mask = m_esb_addr.addr_length == 5 ? 0xFFFF0000 : 0xFF000000;
        if (pipe == 0)
        {
            if ((NRF_RADIO->BASE0 & base_address_mask) == 0 && prefix == 0)
            {
                return NRF_ERROR_INVALID_PARAM;
            }
        }
        else
        {
            if ((NRF_RADIO->BASE1 & base_address_mask) == 0 && prefix == 0)
            {
                return NRF_ERROR_INVALID_PARAM;
            }
        }
    }
#endif
    m_esb_addr.pipe_prefixes[pipe] = prefix;

    update_radio_addresses(NRF_ESB_ADDR_UPDATE_MASK_PREFIX);

#ifdef NRF52832_XXAA
    return apply_address_workarounds();
#else
    return NRF_SUCCESS;
#endif
}


uint32_t nrf_esb_enable_pipes(uint8_t enable_mask)
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);
    VERIFY_TRUE((enable_mask | BIT_MASK_UINT_8(NRF_ESB_PIPE_COUNT)) == BIT_MASK_UINT_8(NRF_ESB_PIPE_COUNT), NRF_ERROR_INVALID_PARAM);

    m_esb_addr.rx_pipes_enabled = enable_mask;

#ifdef NRF52832_XXAA
    return apply_address_workarounds();
#else
    return NRF_SUCCESS;
#endif
}


uint32_t nrf_esb_set_rf_channel(uint32_t channel)
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);
    VERIFY_TRUE(channel <= 100, NRF_ERROR_INVALID_PARAM);

    m_esb_addr.rf_channel = channel;

    return NRF_SUCCESS;
}


uint32_t nrf_esb_get_rf_channel(uint32_t * p_channel)
{
    VERIFY_PARAM_NOT_NULL(p_channel);

    *p_channel = m_esb_addr.rf_channel;

    return NRF_SUCCESS;
}


uint32_t nrf_esb_set_tx_power(nrf_esb_tx_power_t tx_output_power)
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);

    if ( m_config_local.tx_output_power != tx_output_power )
    {
        m_config_local.tx_output_power = tx_output_power;
        update_radio_tx_power();
    }

    return NRF_SUCCESS;
}


uint32_t nrf_esb_set_retransmit_delay(uint16_t delay)
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);
    VERIFY_TRUE(delay >= NRF_ESB_RETRANSMIT_DELAY_MIN, NRF_ERROR_INVALID_PARAM);

    m_config_local.retransmit_delay = delay;
    return NRF_SUCCESS;
}


uint32_t nrf_esb_set_retransmit_count(uint16_t count)
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);

    m_config_local.retransmit_count = count;
    return NRF_SUCCESS;
}


uint32_t nrf_esb_set_bitrate(nrf_esb_bitrate_t bitrate)
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);

    m_config_local.bitrate = bitrate;
    return update_radio_bitrate() ? NRF_SUCCESS : NRF_ERROR_INVALID_PARAM;
}


uint32_t nrf_esb_reuse_pid(uint8_t pipe)
{
    VERIFY_TRUE(m_nrf_esb_mainstate == NRF_ESB_STATE_IDLE, NRF_ERROR_BUSY);
    VERIFY_TRUE(pipe < NRF_ESB_PIPE_COUNT, NRF_ERROR_INVALID_PARAM);

    m_pids[pipe] = (m_pids[pipe] + NRF_ESB_PID_MAX) % (NRF_ESB_PID_MAX + 1);
    return NRF_SUCCESS;
}


#ifdef NRF52832_XXAA
// Workaround neccessary on nRF52832 Rev. 1.
void NRF_ESB_BUGFIX_TIMER_IRQHandler(void)
{
    if (NRF_ESB_BUGFIX_TIMER->EVENTS_COMPARE[0])
    {
        NRF_ESB_BUGFIX_TIMER->EVENTS_COMPARE[0] = 0;

        // If the timeout timer fires and we are in the PTX receive ACK state, disable the radio
        if (m_nrf_esb_mainstate == NRF_ESB_STATE_PTX_RX_ACK)
        {
            NRF_RADIO->TASKS_DISABLE = 1;
        }
    }
}
#endif

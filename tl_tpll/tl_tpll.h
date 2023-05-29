/********************************************************************************************************
 * @file	tl_tpll.h
 *
 * @brief	This is the header file for b80
 *
 * @author	2.4G Group
 * @date	2022
 *
 * @par     Copyright (c) 2022, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *          All rights reserved.
 *
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions are met:
 *
 *              1. Redistributions of source code must retain the above copyright
 *              notice, this list of conditions and the following disclaimer.
 *
 *              2. Unless for usage inside a TELINK integrated circuit, redistributions
 *              in binary form must reproduce the above copyright notice, this list of
 *              conditions and the following disclaimer in the documentation and/or other
 *              materials provided with the distribution.
 *
 *              3. Neither the name of TELINK, nor the names of its contributors may be
 *              used to endorse or promote products derived from this software without
 *              specific prior written permission.
 *
 *              4. This software, with or without modification, must only be used with a
 *              TELINK integrated circuit. All other usages are subject to written permission
 *              from TELINK and different commercial license may apply.
 *
 *              5. Licensee shall be solely responsible for any claim to the extent arising out of or
 *              relating to such deletion(s), modification(s) or alteration(s).
 *
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *          ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *          WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *          DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
 *          DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *          (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *          LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *          ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *          (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *          SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************************************/
#ifndef _TL_TPLL_H_
#define _TL_TPLL_H_

#include "common.h"
#include "tl_tpll_util.h"

#define trf_delay_us                    sleep_us
#define trf_delay_ms(t)                 sleep_us((t)*1000)

#define TRF_SUCCESS                     0
#define TRF_ERROR_NULL_PARAM            1
#define TRF_ERROR_INVALID_PARAM         2
#define TRF_ERROR_BUSY                  3
#define TRF_ERROR_INVALID_STATE         4

#define TRF_RX_WAIT_US                  5
#define TRF_TX_WAIT_US                  5
#define TRF_RX_SETTLE_US                114
#define TRF_TX_SETTLE_US                114
#define TRF_RX_WAIT_ACK_TIMEOUT_US      500 // default 500us
// if the mode is 250k ,the rx_time_out need more time, as so 1000us is ok!

#define TRF_TPLL_RX_WAIT_TIME_MAX        (4096)
#define TRF_TPLL_TX_WAIT_TIME_MAX        (4096)

#define TRF_TPLL_RX_SETTLE_TIME_MIN      (85)
#define TRF_TPLL_RX_SETTLE_TIME_MAX      (4095)

#define TRF_TPLL_TX_SETTLE_TIME_MIN      (108)
#define TRF_TPLL_TX_SETTLE_TIME_MAX      (4095)

#define TRF_TPLL_RX_TIMEOUT_TIME_MIN     (85)
#define TRF_TPLL_RX_TIMEOUT_TIME_MAX     (4095)

#define TRF_TPLL_MAX_PAYLOAD_LENGTH      64

#ifndef TRF_TPLL_PIPE_COUNT
#define TRF_TPLL_PIPE_COUNT              6
#endif
STATIC_ASSERT(TRF_TPLL_PIPE_COUNT <= 6);

#define TRF_TPLL_CREATE_PAYLOAD(_pipeid, ...)                    \
{.pipe_id = _pipeid, .length = TRF_NUM_VA_ARGS(__VA_ARGS__),    \
.data = {__VA_ARGS__}};                                         \
STATIC_ASSERT((TRF_NUM_VA_ARGS(__VA_ARGS__) > 0 && TRF_NUM_VA_ARGS(__VA_ARGS__) <= 64))

#define TRF_TPLL_ADDR_DEFALT                                     \
{                                                               \
    .base_address_0     = {0xe7, 0xe7, 0xe7, 0xe7},             \
    .base_address_1     = {0xc2, 0xc2, 0xc2, 0xc2},             \
    .pipe_prefixes      = {0xe7, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6}, \
    .addr_length        = 5,                                    \
    .pipe_nums          = 6                                     \
}

#define TRF_TPLL_DEFALT_CONFIG                                   \
{                                                               \
    .mode               = TRF_TPLL_MODE_PTX,                     \
    .bitrate            = TRF_TPLL_BITRATE_2MBPS,                \
    .crc                = TRF_TPLL_CRC_16BIT,                    \
    .tx_power           = TRF_TPLL_POWER_N0p22dBm,                   \
    .event_handler      = 0,                                    \
    .retry_delay        = 150,                                  \
    .retry_times        = 5,                                    \
    .preamble_len       = 8,                                    \
    .payload_len        = 32                                    \
}

/**@brief Telink primary link layer address width. */
typedef enum {
    TRF_TPLL_ADDRESS_WIDTH_3BYTES = 3,      /**< Set address width to 3 bytes */
    TRF_TPLL_ADDRESS_WIDTH_4BYTES,          /**< Set address width to 4 bytes */
    TRF_TPLL_ADDRESS_WIDTH_5BYTES           /**< Set address width to 5 bytes */
} trf_tpll_address_width_t;

/**@brief Telink primary link layer pipe IDs. */
typedef enum {
    TRF_TPLL_PIPE0 = 0,          /**< Select pipe0 */
    TRF_TPLL_PIPE1,              /**< Select pipe1 */
    TRF_TPLL_PIPE2,              /**< Select pipe2 */
    TRF_TPLL_PIPE3,              /**< Select pipe3 */
    TRF_TPLL_PIPE4,              /**< Select pipe4 */
    TRF_TPLL_PIPE5,              /**< Select pipe5 */
    TRF_TPLL_TX,                 /**< Refer to TX address*/
    TRF_TPLL_PIPE_ALL = 0xFF     /**< Close or open all pipes*/
} trf_tpll_pipeid_t;

/**@brief Telink primary link layer state machine status. */
typedef enum {
    TRF_TPLL_STATE_MACHINE_STATUS_IDLE = 0,          /**< Idle */
    TRF_TPLL_STATE_MACHINE_STATUS_TX_SETTLE,         /**< TX Settle*/
    TRF_TPLL_STATE_MACHINE_STATUS_TX,                /**< TX */
    TRF_TPLL_STATE_MACHINE_STATUS_RX_WAIT,           /**< RX Wait */
    TRF_TPLL_STATE_MACHINE_STATUS_RX,                /**< RX */
    TRF_TPLL_STATE_MACHINE_STATUS_TX_WAIT,           /**< RX Wait */
} trf_state_machine_status_t;

typedef enum {
    TRF_TPLL_MODE_PTX = 0,
    TRF_TPLL_MODE_PRX
} trf_tpll_mode_t;

typedef enum {
    TRF_TPLL_BITRATE_1MBPS = 0,      /**< 1Mbit radio mode. */
    TRF_TPLL_BITRATE_2MBPS,          /**< 2Mbit radio mode. */
    TRF_TPLL_BITRATE_500KBPS,        /**< 500kbit radio mode. */
    TRF_TPLL_BITRATE_250KBPS,        /**< 250Kbit radio mode. */
} trf_tpll_bitrate_t;

/**@brief Telink primary link layer modulation index. */
typedef enum {
	TRF_TPLL_MI_P0p00 = 0,        /**< MI = 0 */
	TRF_TPLL_MI_P0p076 = 76,	    /**< MI = 0.076 This gear is only available in private mode*/
	TRF_TPLL_MI_P0p32 = 320,		/**< MI = 0.32 */
	TRF_TPLL_MI_P0p50 = 500,		/**< MI = 0.5 */
	TRF_TPLL_MI_P0p60 = 600,		/**< MI = 0.6 */
	TRF_TPLL_MI_P0p70 = 700,		/**< MI = 0.7 */
	TRF_TPLL_MI_P0p80 = 800,		/**< MI = 0.8 */
	TRF_TPLL_MI_P0p90 = 900,		/**< MI = 0.9 */
	TRF_TPLL_MI_P1p20 = 1200,		/**< MI = 1.2 */
	TRF_TPLL_MI_P1p30 = 1300,		/**< MI = 1.3 */
	TRF_TPLL_MI_P1p40 = 1400,		/**< MI = 1.4 */
} trf_tpll_mi_t;

typedef enum {
    TRF_TPLL_CRC_16BIT = 0,
    TRF_TPLL_CRC_8BIT,
} trf_tpll_crc_t;

/**@brief Telink primary link layer radio transmission power modes. */
typedef enum {
	 /*VBAT*/
	 TRF_TPLL_POWER_P11p46dBm = 63,  /**< 11.5 dbm */
	 TRF_TPLL_POWER_P11p28dBm = 61,  /**< 11.3 dbm */
	 TRF_TPLL_POWER_P11p00dBm = 58,  /**< 11.0 dbm */
	 TRF_TPLL_POWER_P10p78dBm = 56,  /**< 10.8 dbm */
	 TRF_TPLL_POWER_P10p44dBm = 53,  /**< 10.4 dbm */
	 TRF_TPLL_POWER_P10p19dBm = 51,  /**< 10.2 dbm */
	 TRF_TPLL_POWER_P9p92dBm  = 49,  /**<  9.9 dbm */
	 TRF_TPLL_POWER_P9p60dBm  = 47,  /**<  9.6 dbm */
	 TRF_TPLL_POWER_P9p31dBm  = 45,  /**<  9.3 dbm */
	 TRF_TPLL_POWER_P8p99dBm  = 43,  /**<  9.0 dbm */
	 TRF_TPLL_POWER_P8p63dBm  = 41,  /**<  8.6 dbm */
	 TRF_TPLL_POWER_P8p28dBm  = 39,  /**<  8.3 dbm */
	 TRF_TPLL_POWER_P7p87dBm  = 37,  /**<  7.8 dbm */
	 TRF_TPLL_POWER_P7p43dBm  = 35,  /**<  7.4 dbm */
	 TRF_TPLL_POWER_P6p97dBm  = 33,  /**<  7.0 dbm */
	 TRF_TPLL_POWER_P6p45dBm  = 31,  /**<  6.4 dbm */
	 TRF_TPLL_POWER_P5p92dBm  = 29,  /**<  5.9 dbm */
	 TRF_TPLL_POWER_P5p36dBm  = 27,  /**<  5.5 dbm */
	 TRF_TPLL_POWER_P4p73dBm  = 25,  /**<  4.7 dbm */
	 TRF_TPLL_POWER_P4p08dBm  = 23,  /**<  4.1 dbm */
	 /*VANT*/
	 TRF_TPLL_POWER_P3p95dBm  = BIT(7) | 63,    /**<  4.0 dbm */
	 TRF_TPLL_POWER_P3p72dBm  = BIT(7) | 61,    /**<  3.7 dbm */
	 TRF_TPLL_POWER_P3p48dBm  = BIT(7) | 59,    /**<  3.5 dbm */
	 TRF_TPLL_POWER_P3p24dBm  = BIT(7) | 57,    /**<  3.2 dbm */
	 TRF_TPLL_POWER_P2p87dBm  = BIT(7) | 54,    /**<  2.9 dbm */
	 TRF_TPLL_POWER_P2p60dBm  = BIT(7) | 52,    /**<  2.6 dbm */
	 TRF_TPLL_POWER_P2p30dBm  = BIT(7) | 50,    /**<  2.3 dbm */
	 TRF_TPLL_POWER_P1p99dBm  = BIT(7) | 48,    /**<  2.0 dbm */
	 TRF_TPLL_POWER_P1p67dBm  = BIT(7) | 46,    /**<  1.7 dbm */
	 TRF_TPLL_POWER_P1p33dBm  = BIT(7) | 44,    /**<  1.3 dbm */
	 TRF_TPLL_POWER_P0p78dBm  = BIT(7) | 41,    /**<  0.8 dbm */
	 TRF_TPLL_POWER_P0p59dBm  = BIT(7) | 40,    /**<  0.6 dbm */
	 TRF_TPLL_POWER_N0p22dBm  = BIT(7) | 36,    /**< -0.2 dbm */
	 TRF_TPLL_POWER_N0p44dBm  = BIT(7) | 35,    /**< -0.4 dbm */
	 TRF_TPLL_POWER_N0p67dBm  = BIT(7) | 34,    /**< -0.7 dbm */
	 TRF_TPLL_POWER_N1p15dBm  = BIT(7) | 32,    /**< -1.2 dbm */
	 TRF_TPLL_POWER_N1p71dBm  = BIT(7) | 30,    /**< -1.7 dbm */
	 TRF_TPLL_POWER_N2p26dBm  = BIT(7) | 28,    /**< -2.3 dbm */
	 TRF_TPLL_POWER_N2p84dBm  = BIT(7) | 26,    /**< -2.8 dbm */
	 TRF_TPLL_POWER_N3p51dBm  = BIT(7) | 24,    /**< -3.5 dbm */
	 TRF_TPLL_POWER_N4p18dBm  = BIT(7) | 22,    /**< -4.2 dbm */
	 TRF_TPLL_POWER_N4p97dBm  = BIT(7) | 20,    /**< -5.0 dbm */
	 TRF_TPLL_POWER_N5p85dBm  = BIT(7) | 18,    /**< -5.9 dbm */
	 TRF_TPLL_POWER_N6p83dBm  = BIT(7) | 16,    /**< -6.8 dbm */
	 TRF_TPLL_POWER_N7p97dBm  = BIT(7) | 14,    /**< -8.0 dbm */
	 TRF_TPLL_POWER_N9p27dBm  = BIT(7) | 12,    /**< -9.3 dbm */
	 TRF_TPLL_POWER_N10p84dBm = BIT(7) | 10,    /**<-10.8 dbm */
	 TRF_TPLL_POWER_N12p76dBm = BIT(7) | 8,     /**<-12.8 dbm */
	 TRF_TPLL_POWER_N15p01dBm = BIT(7) | 6,     /**<-15.0 dbm */
	 TRF_TPLL_POWER_N18p40dBm = BIT(7) | 4,     /**<-18.4 dbm */
	 TRF_TPLL_POWER_N24p28dBm = BIT(7) | 2,     /**<-24.3 dbm */

	 TRF_TPLL_POWER_N30dBm    = 0xff,           /**<-30.0 dbm */
	 TRF_TPLL_POWER_N50dBm    = BIT(7) | 0,     /**<-50.0 dbm */

/*-----------------------------For Internal Test only-----------------------------*/
  /*
	* Customer attention:
	*
	* 	The following settings are for internal testing only, and customers
	* 	are prohibited from using those settings.
	*
	* 	The following energy values are measured under 3.3V power supply
	* 	voltage.The energy will decrease as the power supply voltage drops.
	* 	Customers are prohibited from using the following energy settings
	* 	in product development.
	*/
	 TRF_TPLL_VBAT_POWER_P2p45dBm  = 0x15, 		//   2.45 dbm
	 TRF_TPLL_VBAT_POWER_P0p95dBm  = 0x11, 		//   0.95 dbm
	 TRF_TPLL_VBAT_POWER_P0p0dBm   = 0x10, 		//   0.00 dbm
	 TRF_TPLL_VBAT_POWER_N1p35dBm  = 0x0d, 		//   -1.35 dbm
	 TRF_TPLL_VBAT_POWER_N2p75dBm  = 0x0b, 		//   -2.75 dbm
	 TRF_TPLL_VBAT_POWER_P4p75dBm  = 0x09, 		//   -4.75 dbm
} trf_tpll_tx_power_t;

typedef enum {
    TRF_TPLL_EVENT_TX_FINISH,    // transmit finished
    TRF_TPLL_EVENT_TX_FALIED,    // retransmit reached threshold
    TRF_TPLL_EVENT_RX_RECEIVED,  // receive a new-valid-packet
} trf_tpll_event_id_t;

typedef void (*trf_tpll_event_handler_t)(trf_tpll_event_id_t evt_id);

typedef struct {
    trf_tpll_mode_t mode;
    trf_tpll_bitrate_t bitrate;
    trf_tpll_crc_t crc;
    trf_tpll_tx_power_t tx_power;
    trf_tpll_event_handler_t event_handler;
    unsigned short retry_delay;
    unsigned char retry_times;
    unsigned char preamble_len;
    unsigned char payload_len;
} trf_tpll_config_t;

typedef struct {
    unsigned char length;
    unsigned char pipe_id;
    unsigned char noack;
    unsigned char pid;
    signed int rssi;
    unsigned char data[TRF_TPLL_MAX_PAYLOAD_LENGTH];
} trf_tpll_payload_t;

/**
 * @brief       Initiate the the Telink primary link layer module.
 * @note        This function must be called at the beginning of the TPLL configuration.
 * @param       bitrate  Radio bitrate.
 * @return      error code for init result.
 */
unsigned char trf_tpll_init(trf_tpll_config_t *tpll_config);

/**
 * @brief       Set the radio bitrate.
 * @param       bitrate  Radio bitrate.
 * @return      error code for init result.
 */
unsigned char trf_tpll_set_bitrate(trf_tpll_bitrate_t bitrate);

/**
 * @brief       Set the channel to use for the radio.
 * @param       channel Channel to use for the radio.
 * @return      none.
 */
void trf_tpll_set_rf_channel(unsigned char channel);

/**
 * @brief       Set the new channel to use for the radio.
 * @param       channel New channel to use for the radio.
 * @return      none.
 */
void trf_tpll_set_new_rf_channel(unsigned char channel);

/**
 * @brief       Set the radio output power.
 * @param       power   Output power.
 * @return      none.
  */
void trf_tpll_set_txpower(trf_tpll_tx_power_t power);

/**
 * @brief       Set one pipe as a TX pipe.
 * @param       pipe_id Pipe to be set as a TX pipe.
 * @return      none.
 */
void trf_tpll_set_txpipe(trf_tpll_pipeid_t pipe_id);

/**
 * @brief       Get the current TX pipe.
 * @param       none.
 * @return      The pipe set as a TX pipe.
*/
unsigned char trf_tpll_get_txpipe(void);

/**
 * @brief       Get the current TX pipe.
 * @param       none.
 * @return      The pipe set as a TX pipe.
*/
unsigned char trf_tpll_get_txpipe(void);

/**
 * @brief       Update the read pointer of the TX FIFO.
 * @param       pipe_id Pipe id.
 * @return      none.
 */
void trf_tpll_update_txfifo_rptr(trf_tpll_pipeid_t pipe_id);

/**
 * @brief       Open one or all pipes.
 * @param       pipe_id Radio pipes to open.
 * @return      none.
 */
void trf_tpll_open_pipe(trf_tpll_pipeid_t pipe_id);

/**
 * @brief       Close one or all pipes.
 * @param       pipe_id Radio pipes to close.
 * @return      none.
 */
void trf_tpll_close_pipe(trf_tpll_pipeid_t pipe_id);

/**
 * @brief       Set the address for pipes.
 *              Beware of the difference for single and multibyte address registers
 * @param       pipe_id Radio pipe to set.
 * @param       addr    Buffer from which the address is stored in
 * @return      /
 */
void trf_tpll_set_address(trf_tpll_pipeid_t pipe_id, const unsigned char *addr);
/**
 * @brief       Get the address for selected pipe.
 * @param       pipe_id Pipe for which to get the address.
 * @param       addr    Pointer t a buffer that address bytes are written to.
 *               <BR><BR>For pipes containing only LSB byte of address, this byte is returned.
 * @return      Numbers of bytes copied to addr.
 */
unsigned char trf_tpll_get_address(trf_tpll_pipeid_t pipe_id, unsigned char *addr);

/**
 * @brief       Set the width of the address.
 * @param       address_width   Width of the TPLL address (in bytes).
 * @return      none.
 */
unsigned char trf_tpll_set_address_width(trf_tpll_address_width_t address_width);
/**
 * @brief       Get the width of the address.
 * @param       none.
 * @return      Width of the TPLL address (in bytes).
 */
unsigned char trf_tpll_get_address_width(void);

/**
 * @brief       Set the the number of retransmission attempts and the packet retransmit delay.
 * @param       retr_times  Number of retransmissions. Setting the parmater to 0 disables retransmission.
 * @param       retry_delay Delay between retransmissions.
 * @return      none.
 */
void trf_tpll_set_auto_retry(unsigned char retry_times, unsigned short retry_delay);

/**
 * @brief       Check status for a selected pipe.
 * @param       pipe_id Pipe number to check status for.
 * @return      Pipe status.
 */
unsigned char trf_tpll_get_pipe_status(trf_tpll_pipeid_t pipe_id);

/**
 * @brief       Get the packet(s) counter.
 * @param       none.
 * @return      packet lost counter.
 */
unsigned char trf_tpll_get_packet_lost_ctr(void);

/**
 * @brief       Check if the TX FIFO is empty.
 * @param       pipe_id pipe id for which to check.
 * @return      1: the TX FIFO is empty; 0: the packet is not empty.
 */
unsigned char trf_tpll_txfifo_empty(trf_tpll_pipeid_t pipe_id);

/**
 * @brief       Check if TX FIFO is full.
 * @param       pipe_id pipe id for which to check.
 * @return      TRUE TX FIFO empty or not.
 */
unsigned char trf_tpll_txfifo_full(trf_tpll_pipeid_t pipe_id);

/**
 * @brief       Get the number of retransmission attempts.
 * @return      Number of retransmissions.
 */
unsigned char trf_tpll_get_transmit_attempts(void);

/**
 * @brief       Read an RX payload.
 * @param       rx_pload   Pointer to buffer where the RX payload is stored.
 * @return      pipe number (MSB byte) and packet length (LSB byte).
 */
unsigned short trf_tpll_read_rx_payload(trf_tpll_payload_t *p_payload);

/**
 * @brief       Get the RX timestamp.
 * @note        It is required to call trf_tpll_read_rx_payload() before this function is called.
 * @return      RX timestamp.
 */
unsigned int trf_tpll_get_timestamp(void);

/**
 * @brief       Write a packet of TX payload into the radio.
 * @param       payload     TX payload.
 * @return      the length of payload written in to tx-fifo.
 * @retval      return 0  :error
 * @retval      return >0 :success, the length of written bytes
 */
unsigned char trf_tpll_write_payload(trf_tpll_payload_t *p_payload);

/**
 * @brief       Reuse the last transmitted payload for the next packet.
 * @param       pipe_id pipe id.
 * @return      none.
 */
void trf_tpll_reuse_tx(trf_tpll_pipeid_t pipe_id);

/**
 * @brief       Remove remaining items from the TX buffer.
 * @param       pipe_id Pipe id.
 * @return      none.
 */
void trf_tpll_flush_tx(trf_tpll_pipeid_t pipe_id);

/**
 * @brief       Remove remaining items from the RX buffer.
 * @param       none.
 * @return      none.
 */
void trf_tpll_flush_rx(void);

/**
 * @brief       Trigger the transmission in the specified pipe
 * @param       none.
 * @return      none.
 */
int trf_tpll_start_tx(void);

/**
 * @brief       Trigger transmission in the specified pipe.
 * @retval      TRF_SUCCESS                If the operation completed successfully.
 * @retval      TRF_ERROR_BUSY             If the function failed because the radio was busy.
 */
int trf_tpll_start_rx(void);

/**
 * @brief       Set the TX wait time.
 * @param       wait_us   Period in microseconds.
 * @return      none.
 */
int trf_tpll_set_tx_wait(unsigned short wait_us);

/**
 * @brief       Set the wait time between the end of an Ack-required packet's transmission
 *              and the start of Ack receiving to accommodate with another chip.
 * @param       wait_us Wait time between the end of an Ack-required packet's transmission
 *              and the start of Ack receiving.
 * @return      none.
 */
int trf_tpll_set_rx_wait(unsigned short wait_us);

/**
 * @brief       Set the rx duration when an Ack-required packet has been
 *              transmitted and an Ack is expected.
 * @param       period_us   specifies the time of rx duration.
 * @return      none.
 */
int trf_tpll_set_rx_timeout(unsigned short period_us);

/**
 * @brief       Set the TX Settle phase.
 * @param       period_us   specifies the time.
 * @return      none.
 */
int trf_tpll_set_tx_settle(unsigned short period_us);

/**
 * @brief       Set the RX Settle phase.
 * @param       period_us   specifies the time.
 * @return      none.
 */
int trf_tpll_set_rx_settle(unsigned short period_us);

/**
 * @brief       Set the mode of TPLL radio.
 * @param       mode    TPLL_MODE_PTX or TPLL_MODE_PRX.
 * @return      none.
 */
void trf_tpll_set_mode(trf_tpll_mode_t mode);

/**
 * @brief       Stop the TPLL state machine.
 * @param       none.
 * @return      none.
 */
void trf_tpll_disable(void);

/**
 * @brief       Set the frequency deviation of the transmitter, which follows the equation below.
 *              frequency deviation = bitrate/(modulation index)^2
 * @param       mi_value    Modulation index.
 * @return      none.
 */
void trf_tpll_set_txmi(trf_tpll_mi_t mi_value);

/**
 * @brief       Set the frequency deviation of the transmitter, which follows the equation below.
 *              frequency deviation = bitrate/(modulation index)^2
 * @param       mi_value    Modulation index.
 * @return      none.
 */
void trf_tpll_set_rxmi(trf_tpll_mi_t mi_value);

/**
 * @brief      Set the length of the preamble field of the on-air data packet.
 * @note       The valid range of this parameter is 1-16.
 * @param      preamble_len Preamble length.
 * @return     error code for result.
 */
unsigned char trf_tpll_set_preamble_len(unsigned char preamble_len);

/**
 * @brief      Read the length of the preamble field of the on-air data packet.
 * @return     Preamble length.
 */
unsigned char trf_tpll_get_preamble_len(void);

/**
 * @brief      disable the receiver preamble detection banking duiring the first byte of pdu.
 * @param      none.
 * @return     none.
 */
void trf_tpll_disable_preamble_detect(void);

/**
 * @brief      Function for setting the base address for pipe 0.
 * @param      base_addr_0.
 * @return     error code for result. 1:bad addr, 0:success.
 */
unsigned char trf_tpll_set_base_address_0(unsigned char *base_addr);

/**
 * @brief      Function for setting the base address for pipe 1 ~ pipe 5.
 * @param      base_addr_1.
 * @return     error code for result. 1:bad addr, 0:success.
 */
unsigned char trf_tpll_set_base_address_1(unsigned char *base_addr);

/**
 * @brief      Function for configures the number of available pipes's prefiex and enables the pipes.
 * @param      prefix for appointed pipe.
 * @return     error code for result. 2: bad pipe nums, 1:bad prefix, 0:success.
 */
unsigned char trf_tpll_set_prefixes(unsigned char *addr_prefix, unsigned char pipe_num);

/**
 * @brief      enable sending an ACK packet when a crc error occurs
 * @param      enable / disable
 * @return     none
 */
void trf_tpll_enable_crcfilter(unsigned char enable);

// Note that this api is not intended for user
void trf_tpll_rxirq_handler(trf_tpll_event_handler_t p_event_handler);
// Note that this api is not intended for user
trf_tpll_event_handler_t trf_tpll_get_event_handler(void);

#endif /*_TL_TPLL_H_*/

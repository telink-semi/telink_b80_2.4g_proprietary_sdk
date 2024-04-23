/********************************************************************************************************
 * @file    tpsll.h
 *
 * @brief   This is the header file for B80
 *
 * @author  2.4G Group
 * @date    2023
 *
 * @par     Copyright (c) 2023, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *
 *          Licensed under the Apache License, Version 2.0 (the "License");
 *          you may not use this file except in compliance with the License.
 *          You may obtain a copy of the License at
 *
 *              http://www.apache.org/licenses/LICENSE-2.0
 *
 *          Unless required by applicable law or agreed to in writing, software
 *          distributed under the License is distributed on an "AS IS" BASIS,
 *          WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *          See the License for the specific language governing permissions and
 *          limitations under the License.
 *
 *******************************************************************************************************/
#ifndef ASYNC_INIT_H_
#define ASYNC_INIT_H_

#define TPSLL_MAX_PAYLOAD_LEN     252

enum
{
    TPSLL_CMD_START = 0,
    TPSLL_SYNC_REQ = 0x01,
    TPSLL_SYNC_RSP = 0x02,
    TPSLL_ACCEPTED = 0X03,
    TPSLL_NOT_ACCEPTED = 0X04,
    TPSLL_CONN_REQ = 0X05,
    TPSLL_CONN_RSP = 0X06,
    TPSLL_CHNL_CLASSIFICATION_REQ = 0X11,
    TPSLL_CHG_CHNL_MAP_REQ = 0X12,
    TPSLL_CHNL_CLASSIFICATION_IND = 0X13,
    TPSLL_SYNC_DATA = 0x14,
    TPSLL_CMD_END
};

enum
{
	TPSLL_FLOW_NESN = 0x01,
	TPSLL_FLOW_SN = 0x02,
	TPSLL_FLOW_SENT = 0x04,
	TPSLL_FLOW_RCVD = 0x08,
};


/** An enum describing the radio's data rate.
 *
 */
typedef enum {
    TPSLL_DATARATE_1MBPS = 0,
    TPSLL_DATARATE_2MBPS = 1,
} tpsll_datarate_t;

/** An enum describing the radio's sync word length.
 *
 */
typedef enum {
    SYNC_WORD_LEN_4BYTE = 4         /**< Set  length to 4 bytes */
} tpsll_sync_word_len_t;

/** An enum describing the pipe IDs.
 *
 */
typedef enum {
	TPSLL_PIPE0 = 0,          /**< Select pipe0 */
	TPSLL_PIPE1,              /**< Select pipe1 */
	TPSLL_PIPE2,              /**< Select pipe2 */
	TPSLL_PIPE3,              /**< Select pipe3 */
	TPSLL_PIPE4,              /**< Select pipe4 */
	TPSLL_PIPE5,              /**< Select pipe5 */
	TPSLL_PIPE_ALL = 0x3F     /**< Close or open all pipes*/
} tpsll_pipe_id_t;

/** An enum describing the radio's crc length.
 *
 */
typedef enum {
	TPSLL_CRC_3BYTE = 3             /**< 3byte crc */
} tpsll_crc_len_t;

/** An enum describing the radio's power level.
 *
 */
typedef enum {
	 /*VBAT*/
	TPSLL_RADIO_POWER_P11p46dBm = 63,  /**< 11.5 dbm */
	TPSLL_RADIO_POWER_P11p28dBm = 61,  /**< 11.3 dbm */
	TPSLL_RADIO_POWER_P11p00dBm = 58,  /**< 11.0 dbm */
	TPSLL_RADIO_POWER_P10p78dBm = 56,  /**< 10.8 dbm */
	TPSLL_RADIO_POWER_P10p44dBm = 53,  /**< 10.4 dbm */
	TPSLL_RADIO_POWER_P10p19dBm = 51,  /**< 10.2 dbm */
	TPSLL_RADIO_POWER_P9p92dBm  = 49,  /**<  9.9 dbm */
	TPSLL_RADIO_POWER_P9p60dBm  = 47,  /**<  9.6 dbm */
	TPSLL_RADIO_POWER_P9p31dBm  = 45,  /**<  9.3 dbm */
	TPSLL_RADIO_POWER_P8p99dBm  = 43,  /**<  9.0 dbm */
	TPSLL_RADIO_POWER_P8p63dBm  = 41,  /**<  8.6 dbm */
	TPSLL_RADIO_POWER_P8p28dBm  = 39,  /**<  8.3 dbm */
	TPSLL_RADIO_POWER_P7p87dBm  = 37,  /**<  7.8 dbm */
	TPSLL_RADIO_POWER_P7p43dBm  = 35,  /**<  7.4 dbm */
	TPSLL_RADIO_POWER_P6p97dBm  = 33,  /**<  7.0 dbm */
	TPSLL_RADIO_POWER_P6p45dBm  = 31,  /**<  6.4 dbm */
	TPSLL_RADIO_POWER_P5p92dBm  = 29,  /**<  5.9 dbm */
	TPSLL_RADIO_POWER_P5p36dBm  = 27,  /**<  5.5 dbm */
	TPSLL_RADIO_POWER_P4p73dBm  = 25,  /**<  4.7 dbm */
	TPSLL_RADIO_POWER_P4p08dBm  = 23,  /**<  4.1 dbm */
	 /*VANT*/
	TPSLL_RADIO_POWER_P3p95dBm  = BIT(7) | 63,    /**<  4.0 dbm */
	TPSLL_RADIO_POWER_P3p72dBm  = BIT(7) | 61,    /**<  3.7 dbm */
	TPSLL_RADIO_POWER_P3p48dBm  = BIT(7) | 59,    /**<  3.5 dbm */
	TPSLL_RADIO_POWER_P3p24dBm  = BIT(7) | 57,    /**<  3.2 dbm */
	TPSLL_RADIO_POWER_P2p87dBm  = BIT(7) | 54,    /**<  2.9 dbm */
	TPSLL_RADIO_POWER_P2p60dBm  = BIT(7) | 52,    /**<  2.6 dbm */
	TPSLL_RADIO_POWER_P2p30dBm  = BIT(7) | 50,    /**<  2.3 dbm */
	TPSLL_RADIO_POWER_P1p99dBm  = BIT(7) | 48,    /**<  2.0 dbm */
	TPSLL_RADIO_POWER_P1p67dBm  = BIT(7) | 46,    /**<  1.7 dbm */
	TPSLL_RADIO_POWER_P1p33dBm  = BIT(7) | 44,    /**<  1.3 dbm */
	TPSLL_RADIO_POWER_P0p78dBm  = BIT(7) | 41,    /**<  0.8 dbm */
	TPSLL_RADIO_POWER_P0p59dBm  = BIT(7) | 40,    /**<  0.6 dbm */
	TPSLL_RADIO_POWER_N0p22dBm  = BIT(7) | 36,    /**< -0.2 dbm */
	TPSLL_RADIO_POWER_N0p44dBm  = BIT(7) | 35,    /**< -0.4 dbm */
	TPSLL_RADIO_POWER_N0p67dBm  = BIT(7) | 34,    /**< -0.7 dbm */
	TPSLL_RADIO_POWER_N1p15dBm  = BIT(7) | 32,    /**< -1.2 dbm */
	TPSLL_RADIO_POWER_N1p71dBm  = BIT(7) | 30,    /**< -1.7 dbm */
	TPSLL_RADIO_POWER_N2p26dBm  = BIT(7) | 28,    /**< -2.3 dbm */
	TPSLL_RADIO_POWER_N2p84dBm  = BIT(7) | 26,    /**< -2.8 dbm */
	TPSLL_RADIO_POWER_N3p51dBm  = BIT(7) | 24,    /**< -3.5 dbm */
	TPSLL_RADIO_POWER_N4p18dBm  = BIT(7) | 22,    /**< -4.2 dbm */
	TPSLL_RADIO_POWER_N4p97dBm  = BIT(7) | 20,    /**< -5.0 dbm */
	TPSLL_RADIO_POWER_N5p85dBm  = BIT(7) | 18,    /**< -5.9 dbm */
	TPSLL_RADIO_POWER_N6p83dBm  = BIT(7) | 16,    /**< -6.8 dbm */
	TPSLL_RADIO_POWER_N7p97dBm  = BIT(7) | 14,    /**< -8.0 dbm */
	TPSLL_RADIO_POWER_N9p27dBm  = BIT(7) | 12,    /**< -9.3 dbm */
	TPSLL_RADIO_POWER_N10p84dBm = BIT(7) | 10,    /**<-10.8 dbm */
	TPSLL_RADIO_POWER_N12p76dBm = BIT(7) | 8,     /**<-12.8 dbm */
	TPSLL_RADIO_POWER_N15p01dBm = BIT(7) | 6,     /**<-15.0 dbm */
	TPSLL_RADIO_POWER_N18p40dBm = BIT(7) | 4,     /**<-18.4 dbm */
	TPSLL_RADIO_POWER_N24p28dBm = BIT(7) | 2,     /**<-24.3 dbm */

	TPSLL_RADIO_POWER_N30dBm    = 0xff,           /**<-30.0 dbm */
	TPSLL_RADIO_POWER_N50dBm    = BIT(7) | 0,     /**<-50.0 dbm */

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
	TPSLL_VBAT_POWER_P2p45dBm  = 0x15, 		//   2.45 dbm
	TPSLL_VBAT_POWER_P0p95dBm  = 0x11, 		//   0.95 dbm
	TPSLL_VBAT_POWER_P0p0dBm   = 0x10, 		//   0.00 dbm
	TPSLL_VBAT_POWER_N1p35dBm  = 0x0d, 		//   -1.35 dbm
	TPSLL_VBAT_POWER_N2p75dBm  = 0x0b, 		//   -2.75 dbm
	TPSLL_VBAT_POWER_P4p75dBm  = 0x09, 		//   -4.75 dbm
} tpsll_radio_power_t;

/**@brief Telink proprietary stack link layer modulation index. */
typedef enum {
	TPSLL_RF_MI_0000 = 0,		 	/**< MI = 0 */
	TPSLL_RF_MI_0076 = 76,		    /**< MI = 0.076 This gear is only available in private mode*/
	TPSLL_RF_MI_0320 = 320,		    /**< MI = 0.32 */
	TPSLL_RF_MI_0500 = 500,		    /**< MI = 0.5 */
	TPSLL_RF_MI_0600 = 600,		    /**< MI = 0.6 */
	TPSLL_RF_MI_0700 = 700,		    /**< MI = 0.7 */
	TPSLL_RF_MI_0800 = 800,		    /**< MI = 0.8 */
	TPSLL_RF_MI_0900 = 900,		    /**< MI = 0.9 */
	TPSLL_RF_MI_1200 = 1200,		/**< MI = 1.2 */
	TPSLL_RF_MI_1300 = 1300,		/**< MI = 1.3 */
	TPSLL_RF_MI_1400 = 1400,		/**< MI = 1.4 */
}TPSLL_MIVauleTypeDef;

/**
 * @brief Telink proprietary stack link layer rx packet format.
 * @note  packet format should not be changed,header = type + len,payload = cmd + data.
 */
typedef struct tpsll_rx_packet
{
    int dma_size;  //dma_size = len + 2
    unsigned char  type;
    unsigned char  len; //length of data + cmd (MAX 252byte)
    unsigned char  cmd;
    unsigned char  data[98];
} __attribute__((packed)) __attribute__ ((aligned (4))) tpsll_rx_packet_t;
extern tpsll_rx_packet_t tpsll_rx_pkt;

/**
 * @brief Telink proprietary stack link layer tx packet format.
 * @note  packet format should not be changed,header = type + len,payload = cmd + data.
 */
typedef struct tpsll_tx_packet
{
    int dma_size;  //dma_size = len + 2
    unsigned char  type;
    unsigned char  len; //length of data + cmd (MAX 252byte)
    unsigned char  cmd;
    unsigned char  data[98];
} __attribute__((packed)) __attribute__ ((aligned (4))) tpsll_tx_packet_t;
extern tpsll_tx_packet_t tpsll_tx_pkt;

tpsll_tx_packet_t tpsll_tx_pkt;
tpsll_rx_packet_t tpsll_rx_pkt;

/**
 * @brief      This function servers to init rf and set radio's on-air datarate.
 * @param[in]  datarate  specify the radio's datarate
 * @param[out] none
 * @return     none.
 */
_attribute_ram_code_sec_noinline_ void tpsll_init(tpsll_datarate_t datarate);

/**
 * @brief      This function servers to set the channel.
 * @param[in]  channel  the valid range is 0-160.
 * @param[out] none.
 * @return     none.
 */
_attribute_ram_code_sec_noinline_ void tpsll_channel_set(signed short channel_num);

/**
 * @brief      This function servers to set the length in octet of the preamble
 *             field of the on-air data packet.Note that the valid range is 1-2.
 * @param[in]  preamble_len  specify the length of preamble field in octet.
 * @param[out] none
 * @return     none.
 */
_attribute_ram_code_sec_noinline_ void tpsll_preamble_len_set(unsigned char preamble_len);

/**
 * @brief      This function servers to set the length in octet of the sync word
 *             field of the on-air data packet.
 * @param[in]  length  specify the length of sync word field in octet, note that
 *                     the valid range of it is 3-5.
 * @param[out] none
 * @return     none.
 */
_attribute_ram_code_sec_noinline_ void tpsll_sync_word_set(tpsll_pipe_id_t pipe, unsigned char *sync_word);

/**
 * @brief      This function servers to set the length of sync word.
 * @param[in]  sync word length.
 * @param[out] none
 * @return     none.
 */
_attribute_ram_code_sec_noinline_ void tpsll_sync_word_len_set(tpsll_sync_word_len_t length);

/**
 * @brief      This function sets DMA RX buffer. Note that the start address of RX
 *             buffer must be 4-byte aligned and the length of RX buffer must be
 *             integral multiple of 16-byte. Also the RX buffer length must be larger
 *             than the whole packet length plus 16 to accommodate data and necessary
 *             RX related information.
 * @param[in]  rx_buffer  pointer to the RX buffer
 * @param[in]  rx_buffer_len  length of RX buffer in octet
 * @param[out] none
 * @return     none.
 */
_attribute_ram_code_sec_noinline_ void tpsll_rx_buffer_set(unsigned char *rx_buffer, unsigned char rx_buffer_len);

/**
 * @brief      This function servers to set the radio's TX output power.
 * @param[in]  level  specify the tx power level
 * @param[out] none
 * @return     none.
 */
_attribute_ram_code_sec_noinline_ void tpsll_radio_power_set(tpsll_radio_power_t level);

/**
 * @brief      This function servers to set the length of crc.
 * @param[in]  crc length.
 * @param[out] none
 * @return     none.
 */
_attribute_ram_code_sec_noinline_ void tpsll_crc_len_set(tpsll_crc_len_t crc_len);

/**
 * @brief      This function sets the rx settle period of transceiver for automatic Single-RX, Single-RX-to-TX and Single-TX-to-RX.
 *             In those three automatic modes, the actual reception starts a short while later after the transceiver enters RX
 *             state. That short while is so-called rx settle period which is used to wait for the RF PLL settling down before
 *             reception. The RX settle period must be larger than 85uS. The default value is 90uS.
 * @param[in]  period_us  specifying the RX settle period in microsecond.
 * @param[out] none
 * @return     none
 */
void tpsll_rx_settle_set(unsigned short period_us);

/**
 * @brief      This function sets the tx settle period of transceiver for automatic Single-TX , Single-TX-to-RX and Single-RX-to-TX.
 *             In those three automatic modes, the actual transmission starts a short while later after the transceiver enters TX
 *             state. That short while is so-called tx settle period which is used to wait for the RF PLL settling down before
 *             transmission. The TX settle period must be larger than 130uS. The default value is 150uS.
 * @param[in]  period_us  specifying the TX settle period in microsecond.
 * @param[out] none
 * @return     none
 */
void tpsll_tx_settle_set(unsigned short period_us);

/**
 * @brief      This function returns the rx payload and rx payload length.
 * @param[in]  rx_buffer and rx_payload_len  pointer to the RX buffer containing currently received packet,
 *             and pointer to store rx payload length.
 * @param[out] rx payload length.
 * @return     rx payload.
 */
_attribute_ram_code_sec_noinline_ unsigned char *tpsll_rx_payload_get(unsigned char *rx_buffer, unsigned char *payload_len);

/**
 * @brief      This function returns the freezing rssi of received packet.
 * @param[in]  rx_buffer  pointer to the RX buffer containing currently received packet.
 * @param[out] none
 * @return     rssi in dBm of the received packet.
 */
_attribute_ram_code_sec_noinline_ signed char tpsll_rx_packet_rssi_get(unsigned char *rx_buffer);

/**
 * @brief      This function returns the instantaneous rssi of current channel.
 * @param[in]  none
 * @param[out] none
 * @return     instantaneous rssi in dBm of current channel.
 */
_attribute_ram_code_sec_noinline_ signed char tpsll_rx_instantaneous_rssi_get(void);

/**
 * @brief      This function returns the timestamp of received packet.
 * @param[in]  rx_buffer  pointer to the RX buffer containing currently received packet.
 * @param[out] none
 * @return     timestamp of the received packet, i.e. the freezing system-timer tick count
 *             value once the sync word is recongnized.
 */
_attribute_ram_code_sec_noinline_ unsigned int tpsll_rx_timestamp_get(unsigned char *rx_buffer);

/**
 * @brief      This function determines whether the result of RX CRC check is ok.
 * @param[in]  rx_buffer  pointer to the RX buffer containing currently received packet.
 * @param[out] none
 * @return     1: the RX CRC check passes. 0: the RX CRC check fails.
 */
_attribute_ram_code_sec_noinline_ unsigned char tpsll_is_rx_crc_ok(unsigned char *rx_buffer);

/**
 * @brief      This function servers to judge whether tx is done .
 * @param[in]  none.
 * @param[out] none.
 * @return     tx done: 1 tx not done: 0.
 */
_attribute_ram_code_sec_noinline_ unsigned char tpsll_is_tx_done(void);

/**
 * @brief      This function servers to clear tx_done_status.
 * @param[in]  none.
 * @param[out] none.
 * @return     tx_done_status cleared: 1 else 0.
 */
_attribute_ram_code_sec_noinline_ void tpsll_tx_done_status_clear(void);

/**
 * @brief      This function starts the automatic Single-TX-to-RX FSM to schedule one transmission and
 *             corresponding reception activity. The transceiver enters TX state and start the packet's
 *             transmission when the system timer matches the start_point. Once the trannsmission is done,
 *             the transceiver transits into RX State and waits for a packet coming. The RX state lasts for
 *             timeout_us microseconds at most. If no packet arrives during that period, a RX Timeout irq
 *             will occur immediately. It usually applies to the case where a packet needs transmitting
 *             with an ACK packet required.
 * @param[in]  tx_buffer  pointer to the TX buffer which has been filled with the packet needs transmitting.
 * @param[in]  start_point  a timer tick count value, when the system-timer's tick count matches this value,
 *                          the transceiver transits into TX state and starts the trannsmission of the packet
 *                          in TX Buffer.
 * @param[in]  timeout_us  specifies the period the transceiver will remain in RX state to waits for
 *                         a packet coming, and a RX timeout irq occurs if no packet is received
 *                         during that period. Note that RX settle period is included in timeout_us,
 *                         namely:
 *                         timeout_us = Rx settle period + Actual Rx period.
 * @param[out] none
 * @return     none
 */
_attribute_ram_code_sec_ void tpsll_stx2rx_start(unsigned int start_point, unsigned int timeout_us);

/**
 * @brief      This function starts the automatic Single-TX FSM to schedule one transmission activity.
 * @param[in]  tx_buffer  pointer to the TX buffer which has been filled with the packet needs transmitting.
 * @param[in]  start_point  a timer tick count value, when the system-timer's tick count matches this value,
 *                          the transmission activity starts immediately.
 * @param[out] none
 * @return     none
 */
_attribute_ram_code_sec_ void tpsll_stx_start(unsigned int start_point);

/**
 * @brief      This function starts the automatic Single-RX FSM to schedule one reception activity.
 * @param[in]  start_point  a timer tick count value, when the system-timer's tick count matches this value,
 *                          the transceiver enters RX state and waits for a packet coming.
 * @param[in]  timeout_us  specifies the period the transceiver will remain in RX state to waits for
 *                         a packet coming, and a RX first timeout irq occurs if no packet is received
 *                         during that period. If timeout_us equals to 0, it denotes the RX first timeout is
 *                         disabled and the transceiver won't exit the RX state until a packet arrives. Note
 *                         that RX settle period is included in timeout_us, namely:
 *                         timeout_us = Rx settle period + Actual Rx period.
 * @param[out] none
 * @return     none
 */
_attribute_ram_code_sec_ void tpsll_srx_start(unsigned int start_point, unsigned int timeout_us);

/**
 * @brief      This function starts the automatic Single-RX-to-TX FSM to schedule one reception and
 *             corresponding transmission activity. The transceiver enters RX state and waits for a
 *             packet coming. The RX state lasts for timeout_us microseconds at most. If no packet arrives,
 *             a RX First Timeout irq will occur immediately. If a packet is received before the timeout,
 *             the transceiver transits into TX state and start the transmission of the packet in TX Buffer.
 *             It usually applies to the case where a packet needs receiving first and then a response packet
 *             needs sending back.
 * @param[in]  tx_buffer  pointer to the TX buffer which has been filled with the packet needs transmitting.
 * @param[in]  start_point  a timer tick count value, when the system-timer's tick count matches this value,
 *                          the transceiver transits into RX state and waits for a packet coming.
 * @param[in]  timeout_us  specifies the period the transceiver will remain in RX state to waits for
 *                         a packet coming, and a RX first timeout irq occurs if no packet is received
 *                         during that period. If timeout_us equals to 0, it denotes the RX first timeout is
 *                         disabled and the transceiver won't exit the RX state until a packet arrives. Note
 *                         that RX settle period is included in timeout_us, namely:
 *                         timeout_us = Rx settle period + Actual Rx period.
 * @param[out] none
 * @return     none
 */
_attribute_ram_code_sec_ void tpsll_srx2tx_start(unsigned int start_point, unsigned int timeout_us);

/**
 * @brief      This function servers to write tx payload.Note that the valid range is 1-252.
 * @param[in]  tx payload and tx payload len.
 * @param[out] none
 * @return     none.
 */
_attribute_ram_code_sec_ int tpsll_tx_write_payload(unsigned char *payload,unsigned char payload_len);

/**
 * @brief      This function servers to open one or all pipes.
 *             field of the on-air data packet.
 * @param[in]  pipe  specify which pipe needs to be open
 * @param[out] none
 * @return     none.
 */
_attribute_ram_code_sec_noinline_ void tpsll_pipe_open(tpsll_pipe_id_t pipe);

/**
 * @brief      This function servers to close one or all pipes.
 *             field of the on-air data packet.
 * @param[in]  pipe  specify which pipe needs to be closed
 * @param[out] none
 * @return     none.
 */
_attribute_ram_code_sec_noinline_ void tpsll_pipe_close(tpsll_pipe_id_t pipe);


#endif /* ASYNC_INIT_H_ */

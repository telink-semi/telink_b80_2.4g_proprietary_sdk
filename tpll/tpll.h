/********************************************************************************************************
 * @file    tpll.h
 *
 * @brief   This is the header file for B80
 *
 * @author  2.4G Group
 * @date    2021
 *
 * @par     Copyright (c) 2021, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
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
#ifndef _TPLL_H_
#define _TPLL_H_
#define RX_SETTLE_TIME_US                120
#define TX_SETTLE_TIME_US                110
#define PTX_RETRY_DELAY_TIME_US                 10


#define TPLL_RF_PACKET_LENGTH_OK(p)           (p[0] == (p[12]&0x3f)+15)
#define TPLL_RF_PACKET_CRC_OK(p)              ((p[p[0]+3] & 0x51) == 0x40)

#define TLSR_SUCCESS                            (0)
#define TLSR_ERROR_INVALID_PARAM                (-1)
#define TLSR_ERROR_BUSY                         (-2)


/**@brief Telink primary link layer address width. */
typedef enum {
    ADDRESS_WIDTH_3BYTES = 3,      /**< Set address width to 3 bytes */
    ADDRESS_WIDTH_4BYTES,          /**< Set address width to 4 bytes */
    ADDRESS_WIDTH_5BYTES           /**< Set address width to 5 bytes */
} TPLL_AddressWidthTypeDef;

/**@brief Telink primary link layer pipe IDs. */
typedef enum {
    TPLL_PIPE0 = 0,          /**< Select pipe0 */
    TPLL_PIPE1,              /**< Select pipe1 */
    TPLL_PIPE2,              /**< Select pipe2 */
    TPLL_PIPE3,              /**< Select pipe3 */
    TPLL_PIPE4,              /**< Select pipe4 */
    TPLL_PIPE5,              /**< Select pipe5 */
    TPLL_TX,                 /**< Refer to TX address*/
    TPLL_PIPE_ALL = 0xFF     /**< Close or open all pipes*/
} TPLL_PipeIDTypeDef;

/**@brief Telink primary link layer state machine status. */
typedef enum {
    TPLL_STATE_MACHINE_STATUS_IDLE = 0,          /**< Idle */
    TPLL_STATE_MACHINE_STATUS_TX_SETTLE,         /**< TX Settle*/
    TPLL_STATE_MACHINE_STATUS_TX,                /**< TX */
    TPLL_STATE_MACHINE_STATUS_RX_WAIT,           /**< RX Wait */
    TPLL_STATE_MACHINE_STATUS_RX,                /**< RX */
    TPLL_STATE_MACHINE_STATUS_TX_WAIT,           /**< RX Wait */
} TPLL_StatemachineStatusTypeDef;

/**@brief Telink primary link layer mode. */
typedef enum {
    TPLL_MODE_PTX = 0,      /**< PTX Mode */
    TPLL_MODE_PRX,          /**< PRX Mode */
} TPLL_ModeTypeDef;

/**@brief Telink primary link layer bitrate mode. */
typedef enum {
    TPLL_BITRATE_1MBPS = 0,      /**< 1Mbit radio mode. */
    TPLL_BITRATE_2MBPS,          /**< 2Mbit radio mode. */
    TPLL_BITRATE_500kBPS,        /**< 500kbit radio mode. */
    TPLL_BITRATE_250KBPS,        /**< 250Kbit radio mode. */
} TPLL_BitrateTypeDef;

/**@brief Telink primary link layer modulation index. */
typedef enum {
	TPLL_RF_MI_0000 = 0,             /**< MI = 0 */
	TPLL_RF_MI_0076 = 76,            /**< MI = 0.076 This gear is only available in private mode*/
	TPLL_RF_MI_0320 = 320,		 	/**< MI = 0.32 */
	TPLL_RF_MI_0500 = 500,		  	/**< MI = 0.5 */
	TPLL_RF_MI_0600 = 600,		  	/**< MI = 0.6 */
	TPLL_RF_MI_0700 = 700,		  	/**< MI = 0.7 */
	TPLL_RF_MI_0800 = 800,		  	/**< MI = 0.8 */
	TPLL_RF_MI_0900 = 900,		  	/**< MI = 0.9 */
	TPLL_RF_MI_1200 = 1200,		    /**< MI = 1.2 */
	TPLL_RF_MI_1300 = 1300,		    /**< MI = 1.3 */
	TPLL_RF_MI_1400 = 1400,		    /**< MI = 1.4 */
}TPLL_MIVauleTypeDef;

/**@brief Telink primary link layer radio transmission power modes. */
typedef enum {
	 /*VBAT*/
	TPLL_RF_POWER_P11p46dBm = 63,  /**< 11.5 dbm */
	TPLL_RF_POWER_P11p28dBm = 61,  /**< 11.3 dbm */
	TPLL_RF_POWER_P11p00dBm = 58,  /**< 11.0 dbm */
	TPLL_RF_POWER_P10p78dBm = 56,  /**< 10.8 dbm */
	TPLL_RF_POWER_P10p44dBm = 53,  /**< 10.4 dbm */
	TPLL_RF_POWER_P10p19dBm = 51,  /**< 10.2 dbm */
	TPLL_RF_POWER_P9p92dBm  = 49,  /**<  9.9 dbm */
	TPLL_RF_POWER_P9p60dBm  = 47,  /**<  9.6 dbm */
	TPLL_RF_POWER_P9p31dBm  = 45,  /**<  9.3 dbm */
	TPLL_RF_POWER_P8p99dBm  = 43,  /**<  9.0 dbm */
	TPLL_RF_POWER_P8p63dBm  = 41,  /**<  8.6 dbm */
	TPLL_RF_POWER_P8p28dBm  = 39,  /**<  8.3 dbm */
	TPLL_RF_POWER_P7p87dBm  = 37,  /**<  7.8 dbm */
	TPLL_RF_POWER_P7p43dBm  = 35,  /**<  7.4 dbm */
	TPLL_RF_POWER_P6p97dBm  = 33,  /**<  7.0 dbm */
	TPLL_RF_POWER_P6p45dBm  = 31,  /**<  6.4 dbm */
	TPLL_RF_POWER_P5p92dBm  = 29,  /**<  5.9 dbm */
	TPLL_RF_POWER_P5p36dBm  = 27,  /**<  5.5 dbm */
	TPLL_RF_POWER_P4p73dBm  = 25,  /**<  4.7 dbm */
	TPLL_RF_POWER_P4p08dBm  = 23,  /**<  4.1 dbm */
	 /*VANT*/
	TPLL_RF_POWER_P3p95dBm  = BIT(7) | 63,    /**<  4.0 dbm */
	TPLL_RF_POWER_P3p72dBm  = BIT(7) | 61,    /**<  3.7 dbm */
	TPLL_RF_POWER_P3p48dBm  = BIT(7) | 59,    /**<  3.5 dbm */
	TPLL_RF_POWER_P3p24dBm  = BIT(7) | 57,    /**<  3.2 dbm */
	TPLL_RF_POWER_P2p87dBm  = BIT(7) | 54,    /**<  2.9 dbm */
	TPLL_RF_POWER_P2p60dBm  = BIT(7) | 52,    /**<  2.6 dbm */
	TPLL_RF_POWER_P2p30dBm  = BIT(7) | 50,    /**<  2.3 dbm */
	TPLL_RF_POWER_P1p99dBm  = BIT(7) | 48,    /**<  2.0 dbm */
	TPLL_RF_POWER_P1p67dBm  = BIT(7) | 46,    /**<  1.7 dbm */
	TPLL_RF_POWER_P1p33dBm  = BIT(7) | 44,    /**<  1.3 dbm */
	TPLL_RF_POWER_P0p78dBm  = BIT(7) | 41,    /**<  0.8 dbm */
	TPLL_RF_POWER_P0p59dBm  = BIT(7) | 40,    /**<  0.6 dbm */
	TPLL_RF_POWER_N0p22dBm  = BIT(7) | 36,    /**< -0.2 dbm */
	TPLL_RF_POWER_N0p44dBm  = BIT(7) | 35,    /**< -0.4 dbm */
	TPLL_RF_POWER_N0p67dBm  = BIT(7) | 34,    /**< -0.7 dbm */
	TPLL_RF_POWER_N1p15dBm  = BIT(7) | 32,    /**< -1.2 dbm */
	TPLL_RF_POWER_N1p71dBm  = BIT(7) | 30,    /**< -1.7 dbm */
	TPLL_RF_POWER_N2p26dBm  = BIT(7) | 28,    /**< -2.3 dbm */
	TPLL_RF_POWER_N2p84dBm  = BIT(7) | 26,    /**< -2.8 dbm */
	TPLL_RF_POWER_N3p51dBm  = BIT(7) | 24,    /**< -3.5 dbm */
	TPLL_RF_POWER_N4p18dBm  = BIT(7) | 22,    /**< -4.2 dbm */
	TPLL_RF_POWER_N4p97dBm  = BIT(7) | 20,    /**< -5.0 dbm */
	TPLL_RF_POWER_N5p85dBm  = BIT(7) | 18,    /**< -5.9 dbm */
	TPLL_RF_POWER_N6p83dBm  = BIT(7) | 16,    /**< -6.8 dbm */
	TPLL_RF_POWER_N7p97dBm  = BIT(7) | 14,    /**< -8.0 dbm */
	TPLL_RF_POWER_N9p27dBm  = BIT(7) | 12,    /**< -9.3 dbm */
	TPLL_RF_POWER_N10p84dBm = BIT(7) | 10,    /**<-10.8 dbm */
	TPLL_RF_POWER_N12p76dBm = BIT(7) | 8,     /**<-12.8 dbm */
	TPLL_RF_POWER_N15p01dBm = BIT(7) | 6,     /**<-15.0 dbm */
	TPLL_RF_POWER_N18p40dBm = BIT(7) | 4,     /**<-18.4 dbm */
	TPLL_RF_POWER_N24p28dBm = BIT(7) | 2,     /**<-24.3 dbm */

	TPLL_RF_POWER_N30dBm    = 0xff,           /**<-30.0 dbm */
	TPLL_RF_POWER_N50dBm    = BIT(7) | 0,     /**<-50.0 dbm */

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
	TPLL_RF_VBAT_POWER_P2p45dBm  = 0x15, 		//   2.45 dbm
	TPLL_RF_VBAT_POWER_P0p95dBm  = 0x11, 		//   0.95 dbm
	TPLL_RF_VBAT_POWER_P0p0dBm   = 0x10, 		//   0.00 dbm
	TPLL_RF_VBAT_POWER_N1p35dBm  = 0x0d, 		//   -1.35 dbm
	TPLL_RF_VBAT_POWER_N2p75dBm  = 0x0b, 		//   -2.75 dbm
	TPLL_RF_VBAT_POWER_P4p75dBm  = 0x09, 		//   -4.75 dbm
}TPLL_OutputPowerTypeDef;



/**
 * @brief       Initiate the the Telink primary link layer module.
 * @note        This function must be called at the beginning of the TPLL configuration.
 * @param       bitrate  Radio bitrate.
 * @return      none.
 */
extern void TPLL_Init(TPLL_BitrateTypeDef bitrate);

/**
 * @brief       Set the radio bitrate.
 * @param       bitrate  Radio bitrate.
 * @return      none.
 */
extern void TPLL_SetBitrate(TPLL_BitrateTypeDef bitrate);

/**
 * @brief       Set the channel to use for the radio. 
 * @param       channel Channel to use for the radio.
 * @return      none.
 */
extern void TPLL_SetRFChannel(signed short channel);

/**
 * @brief       Set the new channel to use for the radio. 
 * @param       channel New channel to use for the radio.
 * @return      none.
 */
extern void TPLL_SetNewRFChannel(signed short channel);

/**
 * @brief       Set the radio output power.
 * @param       power   Output power.
 * @return      none.
  */
extern void TPLL_SetOutputPower(TPLL_OutputPowerTypeDef power);

/**
 * @brief       Set one pipe as a TX pipe.
 * @param       pipe_id Pipe to be set as a TX pipe.
 * @return      none.
 */
extern void TPLL_SetTXPipe(TPLL_PipeIDTypeDef pipe_id);

/**
 * @brief       Get the current TX pipe.
 * @return      The pipe set as a TX pipe.
*/
extern unsigned char TPLL_GetTXPipe(void);

/**
 * @brief       Update the read pointer of the TX FIFO.
 * @param       pipe_id Pipe id.
 * @return      none.
 */
extern void TPLL_UpdateTXFifoRptr(TPLL_PipeIDTypeDef pipe_id);

/**
 * @brief       Enable the W_TX_PAYLOAD_NOACK command.
 * @param       enable  Whether to enable or disable NoAck option in 9-bit Packet control field.
 * @return      none.
 */
extern void TPLL_EnableNoAck(unsigned char enable);

/**
 * @brief       Write the payload that will be transmitted with ACK in the specified pipe.
 * @param       pipe_id     Pipe that transmits the payload.
 * @param       payload     Pointer to the payload data.
 * @param       length      Size of the data to transmit.
 * @return
 */
extern void TPLL_WriteAckPayload(TPLL_PipeIDTypeDef pipe_id, const unsigned char *payload, unsigned char length);

/**
 * @brief       Open one or all pipes.
 * @param       pipe_id Radio pipes to open.
 * @return      none.
 */
extern void TPLL_OpenPipe(TPLL_PipeIDTypeDef pipe_id);

/**
 * @brief       Close one or all pipes.
 * @param       pipe_id Radio pipes to close.
 * @return      none.
 */
extern void TPLL_ClosePipe(TPLL_PipeIDTypeDef pipe_id);

/**
 * @brief       Set the address for pipes.
 * @param       pipe_id Radio pipe to set the address for.
 * @param       addr    Pointer to the address data.
 * @return      none.
 */
extern void TPLL_SetAddress(TPLL_PipeIDTypeDef pipe_id, const unsigned char *addr);

/**
 * @brief       Get the address for pipes.
 * @param       pipe_id Radio pipe to get the address for
 * @param       addr    Pointer to a buffer that address data are written to.
 * @return      Numbers of bytes copied to addr.
 */
extern unsigned char TPLL_GetAddress(TPLL_PipeIDTypeDef pipe_id, unsigned char *addr);

/**
 * @brief       Set the the number of retransmission attempts and the packet retransmit delay.
 * @param       retry_times Number of retransmissions. Setting the parmater to 0 disables retransmission.
 * @param       retry_delay Delay between retransmissions.
 * @return      none.
 */
extern void TPLL_SetAutoRetry(unsigned char retry_times, unsigned short retry_delay);

/**
 * @brief       Set the width of the address.
 * @param       address_width   Width of the TPLL address (in bytes).
 * @return      none.
 */
extern void TPLL_SetAddressWidth(TPLL_AddressWidthTypeDef address_width);

/**
 * @brief       Get the width of the address.
 * @return      Width of the TPLL address (in bytes).
 */
extern unsigned char TPLL_GetAddressWidth(void);

/**
 * @brief       Check status for a selected pipe.
 * @param       pipe_id Pipe id to check status for.
 * @return      Pipe status.
 */
extern unsigned char TPLL_GetPipeStatus(TPLL_PipeIDTypeDef pipe_id);

/**
 * @brief       Get the dropped packet count.
 * @return      Dropped packet count.
 */
extern unsigned char TPLL_GetPacketLostCtr(void);

/* Status functions prototypes */

/**
 * @brief       Check if the TX FIFO is empty.
 * @param       pipe_id pipe id for which to check.
 * @return      1: the TX FIFO is empty; 0: the packet is not empty.
 */
extern unsigned char TPLL_TxFifoEmpty(TPLL_PipeIDTypeDef pipe_id);

/**
 * @brief       Check if TX FIFO is full.
 * @param       pipe_id pipe id for which to check.
 * @return      TRUE TX FIFO empty or not.
 */
extern unsigned char TPLL_TxFifoFull(TPLL_PipeIDTypeDef pipe_id);

/**
 * @brief       Get the number of retransmission attempts.
 * @return      Number of retransmissions.
 */
extern unsigned char TPLL_GetTransmitAttempts(void);

/**
 * @brief       Get the carrier detect status.
 * @return      Carrier detect status.
 */
extern unsigned char TPLL_GetCarrierDetect(void);

/**
 * @brief       Get the pipe that has received a packet.
 * @return      Pipe id.
 */
extern unsigned char TPLL_GetRxDataSource(void);

/**
 * @brief       Read an RX payload.
 * @param       rx_pload   Pointer to the buffer where the payload will be stored.
 * @return      pipe number (MSB) and packet length (LSB).
 */
extern unsigned short TPLL_ReadRxPayload(unsigned char *rx_pload);

/**
 * @brief       Get the RX timestamp. 
 * @note        It is required to call TPLL_ReadRxPayload() before this function is called.
 * @return      RX timestamp.
 */
extern unsigned int TPLL_GetTimestamp(void);

/**
 * @brief       Get the RX RSSI value. 
 * @note        It is required to call TPLL_ReadRxPayload() before this function is called.
 * @return      RSSI value.
 */
extern signed int TPLL_GetRxRssiValue(void);

/**
 * @brief       Write a payload for transmission.
 * @param       pipe_id     Pipe id used for this payload.
 * @param       tx_pload    Pointer to the buffer that contains the payload data.
 * @param       length      Length of the payload.
 * @retval      0           Error
 * @retval      >0          The length of written bytes
 */
extern unsigned char TPLL_WriteTxPayload(TPLL_PipeIDTypeDef pipe_id, const unsigned char *tx_pload, unsigned char length);

/**
 * @brief       Reuse the last transmitted payload for the next packet.
 * @param       pipe_id pipe id.
 * @return      none.
 */
extern void TPLL_ReuseTx(TPLL_PipeIDTypeDef pipe_id);

/**
 * @brief       Remove remaining items from the RX buffer.
 * @return      none.
 */
extern void TPLL_FlushRx(void);

/**
 * @brief       Remove remaining items from the TX buffer.
 * @param       pipe_id Pipe id.
 * @return      none.
 */
extern void TPLL_FlushTx(TPLL_PipeIDTypeDef pipe_id);

/**
 * @brief       Trigger transmission in the specified pipe.
 * @retval      TLSR_SUCCESS                If the operation completed successfully.
 * @retval      TLSR_ERROR_BUSY             If the function failed because the radio was busy.
 */
extern int TPLL_PTXTrig(void);

/**
 * @brief       Trigger reception in the specified pipe.
 * @retval      TLSR_SUCCESS                If the operation completed successfully.
 * @retval      TLSR_ERROR_BUSY             If the function failed because the radio was busy.
 */
extern int TPLL_PRXTrig(void);

/**
 * @brief       Set the RX wait time.
 * @param       wait_us     Period in microseconds.
 * @retval      TLSR_SUCCESS                If the operation completed successfully.
 * @retval      TLSR_ERROR_INVALID_PARAM    If the function failed because the input parameter was out of valid range.
 */
extern int TPLL_RxWaitSet(unsigned short wait_us);

/**
 * @brief       Set the TX wait time.
 * @param       wait_us     Period in microseconds.
 * @retval      TLSR_SUCCESS                If the operation completed successfully.
 * @retval      TLSR_ERROR_INVALID_PARAM    If the function failed because the input parameter was out of valid range.
 */
extern int TPLL_TxWaitSet(unsigned short wait_us);

/**
 * @brief       Set the RX timerout time.
 * @param       period_us   Period in microseconds.
 * @retval      TLSR_SUCCESS                If the operation completed successfully.
 * @retval      TLSR_ERROR_INVALID_PARAM    If the function failed because the input parameter was out of valid range.
 */
extern int TPLL_RxTimeoutSet(unsigned short period_us);

/**
 * @brief       Set the TX settle time.
 * @param       period_us   Period in microseconds.
 * @retval      TLSR_SUCCESS                If the operation completed successfully.
 * @retval      TLSR_ERROR_INVALID_PARAM    If the function failed because the input parameter was out of valid range.
 */
extern int TPLL_TxSettleSet(unsigned short period_us);

/**
 * @brief       Set the RX settle time.
 * @param       period_us   Period in microseconds.
 * @retval      TLSR_SUCCESS                If the operation completed successfully.
 * @retval      TLSR_ERROR_INVALID_PARAM    If the function failed because the input parameter was out of valid range.
 */
extern int TPLL_RxSettleSet(unsigned short period_us);

/**
 * @brief       Set the mode of TPLL radio.
 * @param       mode    TPLL_MODE_PTX or TPLL_MODE_PRX.
 * @return      none.
 */
extern void TPLL_ModeSet(TPLL_ModeTypeDef mode);

/**
 * @brief       Stop the TPLL state machine.
 * @return      none.
 */
extern void TPLL_ModeStop(void);

/**
 * @brief       Check whether the received packet is valid.
 * @return      1: the packet is valid; 0: the packet is invalid.
 */
extern unsigned char TPLL_IsRxPacketValid(void);

/**
 * @brief       Get the packet received.
 * @param       none.
 * @return      rx_packet.
 */
unsigned char *TPLL_GetRxPacket(void);

/**
 * @brief       Get the pid of the received packet.
 * @param       rx_packet.
 * @return      packet id.
 */
unsigned char TPLL_GetRxPacketId(unsigned char *rx_packet);


/**
 * @brief       Set the frequency deviation of the transmitter, which follows the equation below.
 *              frequency deviation = bitrate/(modulation index)^2
 * @param       mi_value    Modulation index.
 * @return      none.
 */
extern void TPLL_SetTxMI(TPLL_MIVauleTypeDef mi_value);

/**
 * @brief       Set the frequency deviation of the receiver, which follows the equation below.
 *              frequency deviation = bitrate/(modulation index)^2
 * @param       mi_value    Modulation index.
 * @return      none.
 */
extern void  TPLL_SetRxMI(TPLL_MIVauleTypeDef mi_value);

/**
 * @brief      Set the length of the preamble field of the on-air data packet.
 * @note       The valid range of this parameter is 1-16.
 * @param      preamble_len Preamble length.
 * @return     none.
 */
extern void TPLL_Preamble_Set(unsigned char preamble_len);

/**
 * @brief      Read the length of the preamble field of the on-air data packet.
 * @return     Preamble length.
 */
extern unsigned char TPLL_Preamble_Read(void);

/**
 * @brief      disable the receiver preamble detection banking duiring the first byte of pdu.
 * @param      none.
 * @return     none.
 */
extern void TPLL_Preamble_Detect_Disable(void);
#endif /*_TPLL_H_*/

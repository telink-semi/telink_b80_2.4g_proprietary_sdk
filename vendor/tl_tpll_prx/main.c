/********************************************************************************************************
 * @file	main.c
 *
 * @brief	This is the source file for B80
 *
 * @author	2.4G Group
 * @date	2021
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

#include "driver.h"
#include "tl_tpll.h"

#define GREEN_LED_PIN           GPIO_PA5
#define WHITE_LED_PIN           GPIO_PA6
#define RED_LED_PIN             GPIO_PA7

trf_tpll_payload_t rx_payload;
trf_tpll_payload_t ack_payload = TRF_TPLL_CREATE_PAYLOAD(TRF_TPLL_PIPE0,
                                0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                                0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10,
                                0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
                                0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20);

unsigned int evt_tx_finish_cnt, evt_tx_failed_cnt, evt_rx_cnt;
__attribute__((section(".ram_code")))__attribute__((optimize("-Os")))
void trf_tpll_event_handler(trf_tpll_event_id_t evt_id)
{
    switch (evt_id)
    {
    case TRF_TPLL_EVENT_TX_FINISH:
        gpio_toggle(GREEN_LED_PIN);
        evt_tx_finish_cnt++;
        break;
    case TRF_TPLL_EVENT_TX_FALIED:
        evt_tx_failed_cnt++;
        break;
    case TRF_TPLL_EVENT_RX_RECEIVED:
        trf_tpll_read_rx_payload(&rx_payload);
        gpio_toggle(WHITE_LED_PIN);
        trf_tpll_flush_tx(ack_payload.pipe_id); // it is necessary in this demo
        ack_payload.data[2] = rx_payload.data[2];
        if (TRF_SUCCESS == trf_tpll_write_payload(&ack_payload))
        {

        }
        else
        {
            gpio_toggle(RED_LED_PIN);
        }
        evt_rx_cnt++;
        break;
    }
}

void gpio_config_init(void)
{
    gpio_set_func(GREEN_LED_PIN|WHITE_LED_PIN|RED_LED_PIN, AS_GPIO);
    gpio_set_output_en(GREEN_LED_PIN|WHITE_LED_PIN|RED_LED_PIN, 1);
    gpio_set_input_en(GREEN_LED_PIN|WHITE_LED_PIN|RED_LED_PIN, 0);
    gpio_write(GREEN_LED_PIN|WHITE_LED_PIN|RED_LED_PIN, 0);
}

unsigned char tpll_config_init(void)
{
    unsigned char err_code = 0;
    unsigned char base_address_0[4] = {0xe7, 0xe7, 0xe7, 0xe7};
    unsigned char base_address_1[4] = {0xc2, 0xc2, 0xc2, 0xc2};
    unsigned char addr_prefix[6] = {0xe7, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6};

    trf_tpll_config_t trf_tpll_config = TRF_TPLL_DEFALT_CONFIG;
    trf_tpll_config.mode             = TRF_TPLL_MODE_PRX;
    trf_tpll_config.bitrate          = TRF_TPLL_BITRATE_2MBPS;
    trf_tpll_config.crc              = TRF_TPLL_CRC_16BIT;
    trf_tpll_config.tx_power         = TRF_TPLL_POWER_N0p22dBm;
    trf_tpll_config.event_handler    = trf_tpll_event_handler;
    trf_tpll_config.preamble_len     = 2;
    trf_tpll_config.payload_len      = 32;

    err_code = trf_tpll_init(&trf_tpll_config);
    TRF_RETVAL_CHECK((err_code == TRF_SUCCESS));

    trf_tpll_set_address_width(TRF_TPLL_ADDRESS_WIDTH_5BYTES);

    err_code = trf_tpll_set_base_address_0(base_address_0);
    TRF_RETVAL_CHECK((err_code == TRF_SUCCESS));

    err_code = trf_tpll_set_base_address_1(base_address_1);
    TRF_RETVAL_CHECK((err_code == TRF_SUCCESS));

    err_code = trf_tpll_set_prefixes(addr_prefix, 6);
    TRF_RETVAL_CHECK((err_code == TRF_SUCCESS));

    trf_tpll_set_txpipe(TRF_TPLL_PIPE0);

    trf_tpll_set_rf_channel(5);
    return err_code;
}

unsigned char payload_errcode = 0;
int main(void)
{
    unsigned char err_code;

    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    wd_32k_stop();

	user_read_flash_value_calib();

    clock_init(SYS_CLK_24M_Crystal);

    gpio_config_init();

    err_code = tpll_config_init();
    TRF_RETVAL_CHECK((err_code == TRF_SUCCESS));

    trf_tpll_enable_crcfilter(1);

    trf_tpll_start_rx();

    ack_payload.data[1] = 0xbc; // for debug
    while (1)
    {

    }
    return 0;
}





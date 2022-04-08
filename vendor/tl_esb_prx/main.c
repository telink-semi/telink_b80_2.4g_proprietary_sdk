/********************************************************************************************************
 * @file    main.c
 *
 * @brief   This is the source file for Eaglet
 *
 * @author  2.4G Group
 * @date    2022
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

#include "driver.h"
#include "tl_esb_ll.h"

#define GREEN_LED_PIN           GPIO_PB4
#define WHITE_LED_PIN           GPIO_PB5
#define RED_LED_PIN             GPIO_PB6

trf_esb_payload_t rx_payload;
trf_esb_payload_t ack_payload = TRF_ESB_CREATE_PAYLOAD(TRF_ESB_PIPE0,
                                0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                                0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10,
                                0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
                                0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20);

unsigned int evt_tx_finish_cnt, evt_tx_failed_cnt, evt_rx_cnt;
__attribute__((section(".ram_code")))__attribute__((optimize("-Os")))
void trf_esb_event_handler(trf_esb_event_id_t evt_id)
{
    switch (evt_id)
    {
    case TRF_ESB_EVENT_TX_FINISH:
        gpio_toggle(GREEN_LED_PIN);
        evt_tx_finish_cnt++;
        break;
    case TRF_ESB_EVENT_TX_FALIED:
        evt_tx_failed_cnt++;
        break;
    case TRF_ESB_EVENT_RX_RECEIVED:
        trf_esb_read_rx_payload(&rx_payload);
        gpio_toggle(WHITE_LED_PIN);
        trf_esb_flush_tx(ack_payload.pipe_id); // it is necessary in this demo
        ack_payload.data[2] = rx_payload.data[2];
        if (TRF_SUCCESS == trf_esb_write_payload(&ack_payload))
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

unsigned char esb_config_init(void)
{
    unsigned char err_code = 0;
    unsigned char base_address_0[4] = {0xe7, 0xe7, 0xe7, 0xe7};
    unsigned char base_address_1[4] = {0xc2, 0xc2, 0xc2, 0xc2};
    unsigned char addr_prefix[6] = {0xe7, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6};

    trf_esb_config_t trf_esb_config = TRF_ESB_DEFALT_CONFIG;
    trf_esb_config.mode             = TRF_ESB_MODE_PRX;
    trf_esb_config.bitrate          = TRF_ESB_BITRATE_2MBPS;
    trf_esb_config.crc              = TRF_ESB_CRC_16BIT;
    trf_esb_config.tx_power         = TRF_ESB_POWER_0DBM;
    trf_esb_config.event_handler    = trf_esb_event_handler;
    trf_esb_config.preamble_len     = 2;
    trf_esb_config.payload_len      = 32;

    err_code = trf_esb_init(&trf_esb_config);
    TRF_RETVAL_CHECK((err_code == TRF_SUCCESS));

    trf_esb_set_address_width(TRF_ESB_ADDRESS_WIDTH_5BYTES);

    err_code = trf_esb_set_base_address_0(base_address_0);
    TRF_RETVAL_CHECK((err_code == TRF_SUCCESS));

    err_code = trf_esb_set_base_address_1(base_address_1);
    TRF_RETVAL_CHECK((err_code == TRF_SUCCESS));

    err_code = trf_esb_set_prefixes(addr_prefix, 6);
    TRF_RETVAL_CHECK((err_code == TRF_SUCCESS));

    trf_esb_set_txpipe(TRF_ESB_PIPE0);

    trf_esb_set_rf_channel(5);
    return err_code;
}

unsigned char payload_errcode = 0;
int main(void)
{
    unsigned char err_code;

    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    clock_init(SYS_CLK_24M_Crystal);

    gpio_config_init();

    err_code = esb_config_init();
    TRF_RETVAL_CHECK((err_code == TRF_SUCCESS));

    trf_esb_enable_crcfilter(1);

    trf_esb_start_rx();

    ack_payload.data[1] = 0xbc; // for debug
    while (1)
    {

    }
    return 0;
}





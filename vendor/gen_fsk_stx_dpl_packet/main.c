/********************************************************************************************************
 * @file	main.c
 *
 * @brief	This is the source file for b80
 *
 * @author	2.4G Group
 * @date	2021
 *
 * @par     Copyright (c) 2021, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
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
#include "genfsk_ll.h"

#define GREEN_LED_PIN       GPIO_PA5
#define TX_PAYLOAD_LEN      32

volatile unsigned char pid = 3;
static unsigned char __attribute__ ((aligned (4))) tx_buffer[64] = {0};
unsigned char tx_payload[32] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
								0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
								0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
								0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f};

void user_init(unsigned char chnn)
{
    unsigned char sync_word[5] = {0xe7, 0xe7, 0xe7, 0xe7, 0xe7};

    // io init
    gpio_set_func(GREEN_LED_PIN, AS_GPIO);
    gpio_set_output_en(GREEN_LED_PIN, 1);
    gpio_set_input_en(GREEN_LED_PIN, 0);
    gpio_write(GREEN_LED_PIN, 0);

    //generic FSK Link Layer configuratioin
    gen_fsk_datarate_set(GEN_FSK_DATARATE_1MBPS); //Note that this API must be invoked first before all other APIs

    gen_fsk_preamble_len_set(4);

    gen_fsk_sync_word_len_set(SYNC_WORD_LEN_5BYTE);

    gen_fsk_sync_word_set(GEN_FSK_PIPE0, sync_word); //set pipe0's sync word

    gen_fsk_pipe_open(GEN_FSK_PIPE0); //enable pipe0's reception

    gen_fsk_tx_pipe_set(GEN_FSK_PIPE0); //set pipe0 as the TX pipe

    gen_fsk_packet_format_set(GEN_FSK_PACKET_FORMAT_VARIABLE_PAYLOAD, 0);

    gen_fsk_radio_power_set(GEN_FSK_RADIO_POWER_N0p22dBm);

    gen_fsk_channel_set(chnn);

    gen_fsk_radio_state_set(GEN_FSK_STATE_AUTO); //set transceiver to basic TX state

    gen_fsk_tx_settle_set(149);

    gen_fsk_auto_pid_disable();
}

_attribute_ram_code_sec_noinline_ int main(void)
{
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    wd_32k_stop();

	user_read_flash_value_calib();

    clock_init(SYS_CLK_24M_Crystal);

    user_init(4);

    //fill the DMA tx buffer
    tx_buffer[0] = sizeof(tx_payload) + 1;
    tx_buffer[1] = 0x00;
    tx_buffer[2] = 0x00;
    tx_buffer[3] = 0x00;
    tx_buffer[4] = sizeof(tx_payload);
    memcpy(&tx_buffer[5], tx_payload, sizeof(tx_payload));

    while (1)
    {
    	gen_fsk_set_pid(&tx_buffer, pid);
    	gen_fsk_stx_start(tx_buffer, clock_time() + 100 * 16);
    	while(rf_tx_finish() == 0);
    	rf_tx_finish_clear_flag();
    	pid = (pid + 1) & 0x03;
    	gpio_toggle(GREEN_LED_PIN);
    	WaitMs(1000);
    }
    return 0;
}



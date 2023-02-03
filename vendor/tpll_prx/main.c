/********************************************************************************************************
 * @file    main.c
 *
 * @brief   This is the source file for b80
 *
 * @author  2.4G Group
 * @date    2021
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
#include "tpll.h"

#define GREEN_LED_PIN           GPIO_PA5
#define ACK_PAYLOAD_LEN         32

static volatile unsigned int rx_interval_us, rx_timestamp, rx_rssi = 0;
volatile unsigned char rx_flag, rx_dr_flag, tx_flag, ds_flag, invalid_pid_flag = 0;
volatile unsigned char rx_payload[128] = {0};
unsigned short length_pip_ret = 0;
static unsigned char ack_payload[ACK_PAYLOAD_LEN] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                                      0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
                                                      0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
                                                      0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f};

static void user_init(signed short chnn)
{
    WaitMs(3000);
    unsigned char rx_address[5] = {0xe7,0xe7,0xe7,0xe7,0xe7};
    // io init
    gpio_set_output_en(GREEN_LED_PIN, 1); //enable output
    gpio_set_input_en(GREEN_LED_PIN, 0); //disable input
    gpio_write(GREEN_LED_PIN,0);

    /*
     * rf configuration
     * notes:b80 rx does not support multiple pipes
     */
    TPLL_Init(TPLL_BITRATE_250KBPS);

    TPLL_SetOutputPower(TPLL_RF_POWER_0DBM);

    TPLL_SetAddressWidth(ADDRESS_WIDTH_5BYTES);

    TPLL_ClosePipe(TPLL_PIPE_ALL);

    TPLL_SetAddress(TPLL_PIPE0, rx_address);

    TPLL_OpenPipe(TPLL_PIPE0);

    TPLL_ModeSet(TPLL_MODE_PRX);

    TPLL_SetRFChannel(chnn);

    TPLL_TxSettleSet(149);

    TPLL_RxSettleSet(80);

    irq_clr_src();
    rf_irq_clr_src(FLD_RF_IRQ_ALL);
    irq_enable_type(FLD_IRQ_ZB_RT_EN);
    rf_irq_disable(FLD_RF_IRQ_ALL);
    rf_irq_enable(FLD_RF_IRQ_TX | FLD_RF_IRQ_TX_DS | FLD_RF_IRQ_RX_DR);
    irq_enable();
}

int main(void)
{
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    wd_32k_stop();

	user_read_flash_value_calib();

    clock_init(SYS_CLK_24M_Crystal);

    user_init(4);

    TPLL_PRXTrig();
    rx_timestamp = clock_time();
    while (1)
    {
        if (1 == rx_dr_flag)
        {
            rx_dr_flag = 0;
            gpio_toggle(GREEN_LED_PIN);
            length_pip_ret = TPLL_ReadRxPayload(&rx_payload);
 
        }
    }
    return 0;
}





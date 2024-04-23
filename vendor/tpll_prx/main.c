/********************************************************************************************************
 * @file	main.c
 *
 * @brief	This is the source file for B80
 *
 * @author	2.4G Group
 * @date	2024
 *
 * @par     Copyright (c) 2024, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
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
#include "tpll.h"

#if (MCU_CORE_B80)
#define GREEN_LED_PIN           GPIO_PA5
#elif(MCU_CORE_B80B)
#define GREEN_LED_PIN           GPIO_PB4
#endif
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
    TPLL_Init(TPLL_BITRATE_2MBPS);

    TPLL_SetOutputPower(TPLL_RF_POWER_N0p22dBm);

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

    clock_init(SYS_CLK_16M_Crystal);

    user_init(4);

    TPLL_PRXTrig();
    rx_timestamp = clock_time();
    while (1)
    {
        if (1 == rx_dr_flag)
        {
            rx_dr_flag = 0;
            gpio_toggle(GREEN_LED_PIN);
            length_pip_ret = TPLL_ReadRxPayload((unsigned char *)&rx_payload);
    		while(!TPLL_TxFifoEmpty(0));
    		TPLL_WriteAckPayload(TPLL_PIPE0, ack_payload, ACK_PAYLOAD_LEN);
 
        }
    }
    return 0;
}





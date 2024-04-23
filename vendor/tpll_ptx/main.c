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

#define PTX_CHANNEL              0//warning:B80 only support pipe0,B80B support pipe0~5
unsigned char preamble_len = 0;
unsigned char tx_payload_len = 32;
volatile unsigned char maxretry_flag, rx_flag, rx_dr_flag, ds_flag = 0;
static unsigned char tx_payload[32] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                                                 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10,
                                                 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
                                                 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20};
void user_init(signed short chnn)
{
    gpio_set_output_en(GREEN_LED_PIN, 1); //enable output
    gpio_set_input_en(GREEN_LED_PIN, 0); //disable input
    gpio_write(GREEN_LED_PIN, 1);

    //rf configuration
    TPLL_Init(TPLL_BITRATE_2MBPS);
    TPLL_SetOutputPower(TPLL_RF_POWER_N0p22dBm);
    TPLL_SetAddressWidth(ADDRESS_WIDTH_5BYTES);
    TPLL_ClosePipe(TPLL_PIPE_ALL);

#if PTX_CHANNEL == 0
    //unsigned char tx_address[3] = {0xe7,0xe7,0xe7};
    unsigned char tx_address[5] = {0xe7,0xe7,0xe7,0xe7,0xe7}; //{0xaa,0xbb,0xcc,0xdd,0xee};
    TPLL_SetAddress(TPLL_PIPE0, tx_address);
    TPLL_OpenPipe(TPLL_PIPE0);
    TPLL_SetTXPipe(TPLL_PIPE0);
#endif

#if PTX_CHANNEL == 1
    unsigned char tx_address1[5] = {0x55, 0x44, 0x33, 0x22, 0x11};
//    unsigned char tx_address1[5] = {0xe7, 0xe7, 0xe7, 0xe7, 0xe7};
    TPLL_SetAddress(TPLL_PIPE1, tx_address1);
    TPLL_OpenPipe(TPLL_PIPE1);
    TPLL_SetTXPipe(TPLL_PIPE1);
#endif

#if PTX_CHANNEL == 2
    unsigned char tx_address2[5] = {0x55, 0x44, 0x33, 0x22, 0x11};
    TPLL_SetAddress(TPLL_PIPE1, tx_address2);
    tx_address2[0] = 0x22;
    TPLL_SetAddress(TPLL_PIPE2, &tx_address2[0]);
    TPLL_OpenPipe(TPLL_PIPE2);
    TPLL_SetTXPipe(TPLL_PIPE2);
#endif

#if PTX_CHANNEL == 3
    unsigned char tx_address3[5] = {0x55, 0x44, 0x33, 0x22, 0x11};
    TPLL_SetAddress(TPLL_PIPE1, tx_address3);
    tx_address3[0] = 0x33;
    TPLL_SetAddress(TPLL_PIPE3, &tx_address3[0]);
    TPLL_OpenPipe(TPLL_PIPE3);
    TPLL_SetTXPipe(TPLL_PIPE3);
#endif

    TPLL_ModeSet(TPLL_MODE_PTX);

    TPLL_SetRFChannel(chnn);

    TPLL_SetAutoRetry(0, 150);  //5,150

    TPLL_RxTimeoutSet(500);//if the mode is 250k ,the rx_time_out need more time, as so 1000us is ok!

    TPLL_RxSettleSet(80);

    TPLL_TxSettleSet(149);

    TPLL_Preamble_Set(8);

    WaitUs(150);
    //configure irq
    irq_clr_src();
    rf_irq_clr_src(FLD_RF_IRQ_ALL);

    irq_enable_type(FLD_IRQ_ZB_RT_EN); //enable RF irq
    rf_irq_disable(FLD_RF_IRQ_ALL);
    rf_irq_enable(FLD_RF_IRQ_TX | FLD_RF_IRQ_TX_DS | FLD_RF_IRQ_TX_RETRYCNT | FLD_RF_IRQ_RX_DR);
    irq_enable(); //enable general irq
}

int main(void)
{
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    wd_32k_stop();

	user_read_flash_value_calib();

    clock_init(SYS_CLK_16M_Crystal);

    user_init(4);

    preamble_len = TPLL_Preamble_Read();

    int ret = 0;
    ds_flag = 1; // for first start
    while (1)
    {
        if (1 == ds_flag || 1 == maxretry_flag)
        {
            if (1 == ds_flag)
            {
                gpio_toggle(GREEN_LED_PIN);
            }
            ds_flag = 0;
            maxretry_flag = 0;
            WaitMs(500);
            tx_payload[1]++;
            ret = TPLL_WriteTxPayload(PTX_CHANNEL, tx_payload, tx_payload_len);
            if (ret)
            {
                TPLL_PTXTrig();
            }
        }
    }
}

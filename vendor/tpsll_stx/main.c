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
#include "tpsll.h"

unsigned char payload_len = 32;
static unsigned char payload[32] __attribute__((aligned(4))) ={
		TPSLL_SYNC_DATA,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x11,0x11,0x11,0x11,0x11,0x11,0x11,0x11,
		0x22,0x22,0x22,0x22,0x22,0x22,0x22,0x22,
		0x33,0x33,0x33,0x33,0x33,0x33,0x33,0x33
};

#if (MCU_CORE_B80)
#define GREEN_LED_PIN           GPIO_PA5
#elif(MCU_CORE_B80B)
#define GREEN_LED_PIN           GPIO_PB4
#endif
#define DEBUG_PIN           GPIO_PD6

//RX Buffer related
volatile static unsigned char chn = 10;
volatile static unsigned char tx_done_flag = 0;
int cnt;

_attribute_ram_code_sec_noinline_ __attribute__((optimize("-Os")))void irq_handler (void)
{
    unsigned int irq_src = irq_get_src();
    unsigned short rf_irq_src = rf_irq_src_get();

    if (irq_src & FLD_IRQ_ZB_RT_EN) {//if rf irq occurs

        if (rf_irq_src & FLD_RF_IRQ_TX) {//if rf tx irq occurs
                rf_irq_clr_src(FLD_RF_IRQ_TX);
                tx_done_flag = 1;
            }
    }

    irq_clr_src2(FLD_IRQ_ALL);
}

int main(void)
{
    
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    wd_32k_stop();

	user_read_flash_value_calib();

    //LED pin config
    gpio_set_func(GREEN_LED_PIN, AS_GPIO);
    gpio_set_output_en(GREEN_LED_PIN, 1); //enable output
    gpio_write(GREEN_LED_PIN, 0); //LED Off

    gpio_set_func(DEBUG_PIN, AS_GPIO);
    gpio_set_output_en(DEBUG_PIN, 1); //enable output
    gpio_write(DEBUG_PIN, 0); //LED Off

    unsigned char sync_word[4] = {0x11, 0x22, 0x33, 0x44};
    //init Link Layer configuratioin
    tpsll_init(TPSLL_DATARATE_2MBPS);
    tpsll_channel_set(chn);
    tpsll_preamble_len_set(2);
    tpsll_sync_word_len_set(SYNC_WORD_LEN_4BYTE);
    tpsll_sync_word_set(TPSLL_PIPE0,sync_word);
    tpsll_pipe_open(TPSLL_PIPE0);
    tpsll_radio_power_set(TPSLL_RADIO_POWER_P5p92dBm);
    tpsll_tx_settle_set(113);

    //irq configuration
    rf_irq_disable(FLD_RF_IRQ_ALL);
    rf_irq_enable(FLD_RF_IRQ_TX); //enable rf rx and rx first timeout irq
    irq_enable_type(FLD_IRQ_ZB_RT_EN); //enable RF irq
    irq_enable(); //enable general irq

    //start the STX
    tpsll_tx_write_payload(payload,payload_len);
    tpsll_stx_start(clock_time()+50*16);

    while (1) {

        tx_done_flag = 0;
        payload[4]++;
        gpio_write(DEBUG_PIN,1);
        tpsll_tx_write_payload(payload,payload_len);
        tpsll_stx_start(clock_time()+100*16);
        gpio_write(DEBUG_PIN,0);
        while (tx_done_flag == 0);
        cnt++;

        gpio_write(GREEN_LED_PIN, 1); //LED On
        WaitMs(200);
        gpio_write(GREEN_LED_PIN, 0); //LED Off
    }
}

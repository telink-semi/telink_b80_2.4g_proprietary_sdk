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

#define RX_BUF_SIZE                     252
volatile unsigned char tpsll_rxbuf[RX_BUF_SIZE]  __attribute__((aligned(4)));

#if (MCU_CORE_B80)
#define GREEN_LED_PIN           GPIO_PA5
#elif(MCU_CORE_B80B)
#define GREEN_LED_PIN           GPIO_PB4
#endif
#define DEBUG_PIN           GPIO_PD6

//RX Buffer related
volatile static unsigned char chn = 10;
volatile static unsigned char rx_flag = 0;
volatile static unsigned char rx_first_timeout = 0;
volatile static unsigned int rx_test_cnt = 0;
volatile static unsigned int rx_first_timeout_cnt = 0;
volatile static unsigned char *rx_payload = 0;
volatile static unsigned char rx_payload_len = 0;
volatile static unsigned char rssi = 0;
volatile static unsigned char rssi_ins = 0;
volatile static unsigned int rx_timestamp = 0;
volatile static unsigned char payload_len = 0;
int stimer_irq_cnt = 0;

_attribute_ram_code_sec_noinline_ __attribute__((optimize("-Os")))void irq_handler (void)
{
    unsigned int irq_src = irq_get_src();
    unsigned short rf_irq_src = rf_irq_src_get();

    if (irq_src & FLD_IRQ_ZB_RT_EN) {//if rf irq occurs
    	if (rf_irq_src & FLD_RF_IRQ_RX) {//if rf rx irq occurs
			rf_irq_clr_src(FLD_RF_IRQ_RX);

			rx_test_cnt++;

			if (tpsll_is_rx_crc_ok((unsigned char *)tpsll_rxbuf)) {
				rx_flag = 1;
			}

		}
        if (rf_irq_src & FLD_RF_IRQ_FIRST_TIMEOUT) {//if rf tx irq occurs
                rf_irq_clr_src(FLD_RF_IRQ_FIRST_TIMEOUT);
                rx_first_timeout = 1;
                rx_first_timeout_cnt++;
            }
    }

    irq_clr_src2(FLD_IRQ_ALL);
}

int main(void)
{
    
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    wd_32k_stop();

	user_read_flash_value_calib();

    clock_init(SYS_CLK_24M_Crystal);

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
    tpsll_rx_buffer_set((unsigned char *)tpsll_rxbuf,RX_BUF_SIZE);
    tpsll_radio_power_set(TPSLL_RADIO_POWER_P5p92dBm);
    tpsll_rx_settle_set(90);

    //irq configuration
    rf_irq_disable(FLD_RF_IRQ_ALL);
    rf_irq_enable(FLD_RF_IRQ_RX | FLD_RF_IRQ_FIRST_TIMEOUT); //enable rf rx and rx first timeout irq
    irq_enable_type(FLD_IRQ_ZB_RT_EN); //enable RF irq
    irq_enable(); //enable general irq

    //start the SRX
    tpsll_srx_start(clock_time()+50*16, 0); //RX first timeout is disabled and the transceiver won't exit the RX state until a packet arrives

    while (1) {

        if (rx_flag) {
            rx_flag = 0;
            //get rx_payload
            rx_payload = tpsll_rx_payload_get((unsigned char *)tpsll_rxbuf, (unsigned char *)&rx_payload_len);
            rssi = (tpsll_rx_packet_rssi_get((unsigned char *)tpsll_rxbuf) + 110);
            rssi_ins = (tpsll_rx_instantaneous_rssi_get() + 110);
            rx_timestamp = tpsll_rx_timestamp_get((unsigned char *)tpsll_rxbuf);

            //start the SRX
            gpio_toggle(GREEN_LED_PIN);
            tpsll_srx_start(clock_time()+50*16, 500);
            gpio_write(DEBUG_PIN,1);
            gpio_write(DEBUG_PIN,0);
            tpsll_rx_payload_get((unsigned char *)tpsll_rxbuf,(unsigned char *)&payload_len);
        }
        if (rx_first_timeout) {
            rx_first_timeout = 0;
            //start the SRX
            tpsll_srx_start(clock_time()+50*16, 500);
        }
    }
}

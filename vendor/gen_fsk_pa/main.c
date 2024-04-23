/********************************************************************************************************
 * @file	main.c
 *
 * @brief	This is the header file for B80
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
#include "genfsk_ll.h"
#include "string.h"
#include "printf.h"

#if (MCU_CORE_B80)
#define BLUE_LED_PIN     		        GPIO_PA4
#define GREEN_LED_PIN     		        GPIO_PA5
#define WHITE_LED_PIN     		        GPIO_PA6
#define RED_LED_PIN     		        GPIO_PA7
#define RF_TX_DONE_PIN     		        GPIO_PC0
#define RF_RX_DONE_PIN     		        GPIO_PC1
#define RF_RX_TIMEOUT_PIN     		    GPIO_PC2
#define RF_FIRST_TIMEOUT_PIN     		GPIO_PC3
#elif (MCU_CORE_B80B)
#define BLUE_LED_PIN                    GPIO_PB3
#define GREEN_LED_PIN                   GPIO_PB4
#define WHITE_LED_PIN                   GPIO_PB5
#define RED_LED_PIN                     GPIO_PB6
#define RF_TX_DONE_PIN     		        GPIO_PD0
#define RF_RX_DONE_PIN     		        GPIO_PD1
#define RF_RX_TIMEOUT_PIN     		    GPIO_PD2
#define RF_FIRST_TIMEOUT_PIN     		GPIO_PD3
#endif
#define    TIME_DEBUG_PIN     GPIO_PD7
#define    FUNC_STX_TEST             1
#define    FUNC_SRX_TEST             2
#define    FUNC_STX2RX_TEST          3
#define    FUNC_SRX2TX_TEST          4

#define    FUNC_TEST         FUNC_STX2RX_TEST

volatile unsigned char rx_ptr = 0;
volatile unsigned char rx_flag = 0;
volatile unsigned char rx_first_timeout = 0;
volatile unsigned char rx_payload_len = 0;
volatile unsigned int rx_test_cnt = 0;
volatile unsigned char tx_done_flag = 0;
volatile unsigned char *rx_packet = 0;
volatile unsigned char *rx_payload = 0;
volatile unsigned char rx_timeout_flag = 0;
volatile unsigned char rssi = 0;
volatile unsigned int rx_timestamp = 0;
volatile unsigned int t0;

void func_stx_test(void);
void func_srx_test(void);
void func_stx2rx_test(void);
void func_srx2tx_test(void);

//RX Buffer related
#define    RX_BUF_LEN                     64
#define    RX_BUF_NUM                     4
 volatile unsigned char rx_buf[RX_BUF_LEN*RX_BUF_NUM] __attribute__ ((aligned (4))) = {};

//TX Buffer related
unsigned char __attribute__ ((aligned (4))) tx_buffer[64] = {0};
unsigned char tx_payload[8] = {0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0xaa, 0xbb};
extern void usb_loginit(void);
_attribute_ram_code_sec_noinline_ __attribute__((optimize("-Os"))) void irq_handler(void)
{
    unsigned int irq_src = irq_get_src();
    unsigned short rf_irq_src = rf_irq_src_get();

if (irq_src & FLD_IRQ_ZB_RT_EN) {//if rf irq occurs

    	if (rf_irq_src & FLD_RF_IRQ_TX)
    	{//if rf tx irq occurs
    		rf_irq_clr_src(FLD_RF_IRQ_TX);
    		tx_done_flag = 1;
    		rf_pa_handler(FLD_RF_IRQ_TX);
    		gpio_toggle(RF_TX_DONE_PIN);
    	}
    	if (rf_irq_src & FLD_RF_IRQ_RX) {//if rf rx irq occurs
			rf_irq_clr_src(FLD_RF_IRQ_RX);
			rx_test_cnt++;
			rx_packet = rx_buf + rx_ptr*RX_BUF_LEN;

			//set to next rx_buf
			rx_ptr = (rx_ptr + 1) % RX_BUF_NUM;
			gen_fsk_rx_buffer_set((unsigned char *)(rx_buf + rx_ptr*RX_BUF_LEN), RX_BUF_LEN);

			if (gen_fsk_is_rx_crc_ok((unsigned char *)rx_packet)) {
				rx_flag = 1;
				rf_pa_handler(FLD_RF_IRQ_RX);
				gpio_toggle(RF_RX_DONE_PIN);
			}
    	}
       if (rf_irq_src & FLD_RF_IRQ_FIRST_TIMEOUT) {//if rf rx irq occurs
			rf_irq_clr_src(FLD_RF_IRQ_FIRST_TIMEOUT);
			rf_pa_handler(FLD_RF_IRQ_FIRST_TIMEOUT);
			rx_first_timeout = 1;
			gpio_toggle(RF_RX_TIMEOUT_PIN);
		}
       if (rf_irq_src & FLD_RF_IRQ_RX_TIMEOUT) {//if rf tx irq occurs
		   rf_irq_clr_src(FLD_RF_IRQ_RX_TIMEOUT);
		   rf_pa_handler(FLD_RF_IRQ_RX_TIMEOUT);
		   rx_timeout_flag = 1;
		   gpio_toggle(RF_FIRST_TIMEOUT_PIN);
		}
}
    rf_irq_clr_src(FLD_RF_IRQ_ALL);
    irq_clr_src2(FLD_IRQ_ALL);
}

int main(void)
{
    unsigned char sync_word[4] = {0x53, 0x78, 0x56, 0x52};

    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    wd_32k_stop();

	user_read_flash_value_calib();

    clock_init(SYS_CLK_24M_Crystal);

	usb_loginit(); //config the USB Log function

    rf_pa_init();

    //LED pin config
    gpio_set_func(GREEN_LED_PIN|BLUE_LED_PIN|WHITE_LED_PIN|RED_LED_PIN, AS_GPIO);
    gpio_set_output_en(GREEN_LED_PIN|BLUE_LED_PIN|WHITE_LED_PIN|RED_LED_PIN, 1); //enable output
    gpio_write(GREEN_LED_PIN|BLUE_LED_PIN|WHITE_LED_PIN|RED_LED_PIN, 0); //LED Off

    gpio_set_func(RF_TX_DONE_PIN|RF_RX_DONE_PIN|RF_RX_TIMEOUT_PIN|RF_FIRST_TIMEOUT_PIN, AS_GPIO);
    gpio_set_output_en(RF_TX_DONE_PIN|RF_RX_DONE_PIN|RF_RX_TIMEOUT_PIN|RF_FIRST_TIMEOUT_PIN, 1); //enable output
    gpio_write(RF_TX_DONE_PIN|RF_RX_DONE_PIN|RF_RX_TIMEOUT_PIN|RF_FIRST_TIMEOUT_PIN, 0); //

    gpio_set_func(TIME_DEBUG_PIN, AS_GPIO);
    gpio_set_output_en(TIME_DEBUG_PIN, 1);
    gpio_write(TIME_DEBUG_PIN, 0);

    //generic FSK Link Layer configuratioin
    gen_fsk_datarate_set(GEN_FSK_DATARATE_1MBPS); //Note that this API must be invoked first before all other APIs
    gen_fsk_preamble_len_set(4);
    gen_fsk_sync_word_len_set(SYNC_WORD_LEN_4BYTE);
    gen_fsk_sync_word_set(GEN_FSK_PIPE0, sync_word); //set pipe0's sync word
    gen_fsk_pipe_open(GEN_FSK_PIPE0); //enable pipe0's reception
    gen_fsk_packet_format_set(GEN_FSK_PACKET_FORMAT_FIXED_PAYLOAD, 8);
    gen_fsk_radio_power_set(GEN_FSK_RADIO_POWER_N0p22dBm);
    gen_fsk_rx_buffer_set((unsigned char *)rx_buf + rx_ptr*RX_BUF_LEN, (unsigned char)RX_BUF_LEN);
    gen_fsk_channel_set(7); //set rf freq as 2403.5MHz
    gen_fsk_radio_state_set(GEN_FSK_STATE_AUTO);
    gen_fsk_rx_settle_set(89);
    //irq configuration
    rf_irq_disable(FLD_RF_IRQ_ALL);
    rf_irq_enable(FLD_RF_IRQ_RX | FLD_RF_IRQ_TX|FLD_RF_IRQ_RX_TIMEOUT|FLD_RF_IRQ_FIRST_TIMEOUT);
    irq_enable_type(FLD_IRQ_ZB_RT_EN); //enable RF irq
    irq_enable(); //enable general irq

    //fill the DMA tx buffer
    tx_buffer[0] = sizeof(tx_payload);
    tx_buffer[1] = 0x00;
    tx_buffer[2] = 0x00;
    tx_buffer[3] = 0x00;
    memcpy(&tx_buffer[4], tx_payload, sizeof(tx_payload));


    while (1)
    {
    	WaitMs(1000);
#if (FUNC_TEST == FUNC_STX_TEST)
    		func_stx_test();
#elif (FUNC_TEST == FUNC_SRX_TEST)
    		rx_flag = 0;
    		rx_first_timeout = 0;
    		gen_fsk_srx_start(clock_time()+50*16, 0); //RX first timeout is disabled and the transceiver won't exit the RX state until a packet arrives
    		func_srx_test();
#elif (FUNC_TEST == FUNC_STX2RX_TEST)
    		rx_timeout_flag = 0;
    		rx_flag = 0;
    		gen_fsk_stx2rx_start(tx_buffer, clock_time()+50*16, 250);
    		func_stx2rx_test();
#elif (FUNC_TEST == FUNC_SRX2TX_TEST)
    		rx_flag = 0;
    		tx_done_flag = 0;
    		gen_fsk_srx2tx_start(tx_buffer, clock_time()+50*16, 0);
    		func_srx2tx_test();
#endif
    	}

}

void func_stx_test(void)
{
	//the STX
	t0= clock_time();
	while(!clock_time_exceed(t0,1000000))
	{
	tx_done_flag = 0;
	gen_fsk_stx_start(tx_buffer, clock_time()+100*16);
	gpio_toggle(TIME_DEBUG_PIN);
	while (tx_done_flag == 0);

	gpio_toggle(RED_LED_PIN);
	WaitMs(100); //delay 100 ms
	tx_buffer[4]++;
	}
}

void func_srx_test(void)
{
	//the SRX
	//start the SRX

	t0= clock_time();
	while(!clock_time_exceed(t0,1000000))
	{
			if (rx_flag) {
				rx_flag = 0;
				rx_payload = gen_fsk_rx_payload_get((unsigned char *)rx_packet, (unsigned char *)&rx_payload_len);
				rssi = (gen_fsk_rx_packet_rssi_get((unsigned char *)rx_packet) + 110);
				rx_timestamp = gen_fsk_rx_timestamp_get((unsigned char *)rx_packet);

				gpio_toggle(WHITE_LED_PIN);

				//start the SRX
				gen_fsk_srx_start(clock_time()+50*16, 120);
				gpio_toggle(TIME_DEBUG_PIN);
			}

			if (rx_first_timeout) {
				rx_first_timeout = 0;
				gen_fsk_srx_start(clock_time()+50*16, 0);
				gpio_toggle(TIME_DEBUG_PIN);
			}
	}
}

void func_stx2rx_test(void)
{
	//the STX2RX
	t0= clock_time();
	while(!clock_time_exceed(t0,1000000))
	{
			if (rx_timeout_flag) {
				rx_timeout_flag = 0;

				WaitMs(100);
				gen_fsk_stx2rx_start(tx_buffer, clock_time()+50*16, 250);
				gpio_toggle(TIME_DEBUG_PIN);
			}

			if (rx_flag) {
				rx_flag = 0;
				rx_payload = gen_fsk_rx_payload_get((unsigned char *)rx_packet, (unsigned char *)&rx_payload_len);
				rssi = (gen_fsk_rx_packet_rssi_get((unsigned char *)rx_packet) + 110);
				rx_timestamp = gen_fsk_rx_timestamp_get((unsigned char *)rx_packet);

				gpio_toggle(GREEN_LED_PIN);

				WaitMs(100);
				gen_fsk_stx2rx_start(tx_buffer, clock_time()+50*16, 250);
				gpio_toggle(TIME_DEBUG_PIN);
			}
	}
}

void func_srx2tx_test(void)
{
	//the SRX2TX
	t0= clock_time();
	while(!clock_time_exceed(t0,1000000))
	{
		if (rx_flag) {

			rx_flag = 0;
			rx_payload = gen_fsk_rx_payload_get((unsigned char *)rx_packet, (unsigned char *)&rx_payload_len);
			rssi = (gen_fsk_rx_packet_rssi_get((unsigned char *)rx_packet) + 110);
			rx_timestamp = gen_fsk_rx_timestamp_get((unsigned char *)rx_packet);

			gpio_toggle(BLUE_LED_PIN);
		}

		if (tx_done_flag) {
			tx_done_flag = 0;
			//start the SRX2TX
			gen_fsk_srx2tx_start(tx_buffer, clock_time()+50*16, 0);
			gpio_toggle(TIME_DEBUG_PIN);

		}
	}

}




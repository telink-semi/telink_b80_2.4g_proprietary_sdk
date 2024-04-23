/********************************************************************************************************
 * @file	app.c
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
#include "app_config.h"

#define WAKEUP_PAD						GPIO_PD0
#define DEBUG_IO_PIN                    GPIO_PB0
#define CURRENT_TEST	     			0
#define CRC_OK			     			1
#define MAX_TIMES                       8

#define TX								1
#define RX								2
#define RF_TX_RX_MODE					TX

#define RF_POWER                        RF_POWER_P11p46dBm

#define RF_FREQ							17
#define ACCESS_CODE						0x29417671
#define TX_INTERVAL_MS					1
volatile unsigned int rx_cnt=0;
volatile unsigned int tx_cnt=0;
unsigned char  rx_packet[64]  __attribute__ ((aligned (4)));
unsigned char  Private_SB_tx_packet[48] __attribute__ ((aligned (4))) = {0x20,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x00,0x33,0x44,0x55,0x66,0x77,0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff};

_attribute_data_retention_ unsigned int retention_data_test = 0;

void user_init()
{
	sleep_ms(2000);
#if CURRENT_TEST
	gpio_shutdown(GPIO_ALL);

#else
	//1.init the LED pin,for indication
	gpio_set_func(LED1 ,AS_GPIO);
	gpio_set_output_en(LED1, 1); 		//enable output
	gpio_set_input_en(LED1 ,0);			//disable input
	gpio_write(LED1, 1);              	//LED On
	sleep_ms(1000);
	gpio_write(LED1, 0);

    gpio_set_func(DEBUG_IO_PIN, AS_GPIO);
    gpio_set_output_en(DEBUG_IO_PIN, 1); //enable output
    gpio_set_input_en(DEBUG_IO_PIN, 0); //disable input

	gpio_set_func(LED2 | LED3,AS_GPIO);		//enable output
	gpio_set_output_en(LED2 | LED3, 1);
	gpio_set_input_en(LED2 | LED3,0);			//disable input

    if (retention_data_test == MAX_TIMES)
    {
        //resume the SWS for debug
        gpio_set_input_en(GPIO_SWS, 1);
        while (1)
        {
            WaitMs(200);
            gpio_toggle(LED3);
        }
    }
    gpio_set_func(DEBUG_IO_PIN, AS_GPIO);
    gpio_set_output_en(DEBUG_IO_PIN, 1);
    gpio_write(DEBUG_IO_PIN, 1);
    WaitMs(10);

	retention_data_test++;

#endif

#if(PM_MODE==IDLE_TIMER_WAKEUP)



#elif(PM_MODE==SUSPEND_PAD_WAKEUP)
	/* Caution: if wake-up source is only pad, 32K clock source MUST be 32K RC * */

	cpu_set_gpio_wakeup(WAKEUP_PAD, Level_High, 1);
	gpio_setup_up_down_resistor(WAKEUP_PAD, PM_PIN_PULLDOWN_100K);

#elif(PM_MODE==DEEP_PAD_WAKEUP)
	/* Caution: if wake-up source is only pad, 32K clock source MUST be 32K RC * */

	cpu_set_gpio_wakeup(WAKEUP_PAD, Level_High, 1);
	gpio_setup_up_down_resistor(WAKEUP_PAD, PM_PIN_PULLDOWN_100K);

	cpu_sleep_wakeup(DEEPSLEEP_MODE , PM_WAKEUP_PAD, 0);

#elif(PM_MODE==DEEP_32K_RC_WAKEUP||PM_MODE==DEEP_32K_XTAL_WAKEUP)

    cpu_sleep_wakeup(DEEPSLEEP_MODE, PM_WAKEUP_TIMER, (clock_time() + 1000*CLOCK_16M_SYS_TIMER_CLK_1MS));

#elif(PM_MODE==DEEP_LONG_32K_RC_WAKEUP||PM_MODE==DEEP_LONG_32K_XTAL_WAKEUP)

    cpu_long_sleep_wakeup(DEEPSLEEP_MODE, PM_WAKEUP_TIMER, 500*CLOCK_32K_SYS_TIMER_CLK_1MS);

#elif(PM_MODE==DEEP_RET_PAD_WAKEUP)
	/* Caution: if wake-up source is only pad, 32K clock source MUST be 32K RC * */

	cpu_set_gpio_wakeup(WAKEUP_PAD, Level_High, 1);
	gpio_setup_up_down_resistor(WAKEUP_PAD, PM_PIN_PULLDOWN_100K);

	cpu_sleep_wakeup(DEEPSLEEP_MODE_RET_SRAM_LOW16K , PM_WAKEUP_PAD, 0);

#elif(PM_MODE==DEEP_RET_32K_RC_WAKEUP||PM_MODE==DEEP_RET_32K_XTAL_WAKEUP)

	cpu_sleep_wakeup(DEEPSLEEP_MODE_RET_SRAM_LOW16K, PM_WAKEUP_TIMER, (clock_time() + 4000*CLOCK_16M_SYS_TIMER_CLK_1MS));

#elif(PM_MODE==DEEP_RET_LONG_32K_RC_WAKEUP||PM_MODE==DEEP_RET_LONG_32K_XTAL_WAKEUP)

    cpu_long_sleep_wakeup(DEEPSLEEP_MODE_RET_SRAM_LOW16K, PM_WAKEUP_TIMER, 500*CLOCK_32K_SYS_TIMER_CLK_1MS);

#endif

}

/////////////////////////////////////////////////////////////////////
// main loop flow
/////////////////////////////////////////////////////////////////////
void main_loop (void)
{

	rc_24m_cal();

#if(PM_MODE == IDLE_TIMER_WAKEUP)
	//After stall wakes up, it does not enter the interrupt, but only continues to execute, and the interrupt mask needs to be cleared.
	timer0_set_mode(TIMER_MODE_SYSCLK, 0, 2*CLOCK_SYS_CLOCK_1S);
	timer_clr_irq_mask(TMR_STA_TMR0);		//Clear the interrupt mask.
	timer_start(TIMER0);
	cpu_stall_wakeup(FLD_IRQ_TMR0_EN);
	timer_clear_interrupt_status(FLD_TMR_STA_TMR0);	//Clear the interrupt status.
	timer_stop(TIMER0);

#elif(PM_MODE == IDLE_STIMER_WAKEUP)
	stimer_set_capture_tick(clock_time() + CLOCK_16M_SYS_TIMER_CLK_1S);
	stimer_enable();
	cpu_stall_wakeup(FLD_IRQ_SYSTEM_TIMER);
	stimer_clr_irq_status();				//Clear the interrupt status.
	stimer_disable();

#elif(PM_MODE == IDLE_RF_WAKEUP)

	rf_mode_init();
	rf_set_pri_1M_mode();
	rf_set_power_level_index (RF_POWER);
	rf_set_tx_rx_off_auto_mode();
	rf_set_channel(RF_FREQ,0);
    irq_disable();
    irq_clr_src();
    rf_irq_disable(FLD_RF_IRQ_ALL);
    rf_irq_enable(FLD_RF_IRQ_TX|FLD_RF_IRQ_RX|FLD_RF_IRQ_RX_TIMEOUT);
    rf_access_code_comm(ACCESS_CODE);

#if(RF_TX_RX_MODE == TX)
	sleep_ms(1);
	rf_start_stx (Private_SB_tx_packet, clock_time() + 16*1000*TX_INTERVAL_MS);
	cpu_stall_wakeup(FLD_IRQ_ZB_RT_EN);
	rf_tx_finish_clear_flag();				//Clear the interrupt status.
	sleep_ms(500);
#elif(RF_TX_RX_MODE == RX)
	rf_rx_buffer_set(rx_packet,64, 0);
	rf_start_srx(clock_time() + 100*16);
	cpu_stall_wakeup(FLD_IRQ_ZB_RT_EN);
	rf_rx_finish_clear_flag();				//Clear the interrupt status.
	sleep_ms(1500);
#endif

#elif(PM_MODE == IDLE_PAD_WAKEUP)
	//After stall wakes up, the advanced interrupt continues to execute, and the interrupt mask and total interrupt need to be opened.
	gpio_set_func(GPIO_PB1 ,AS_GPIO);
	gpio_set_output_en(GPIO_PB1, 0);
	gpio_set_input_en(GPIO_PB1 ,1);
	gpio_setup_up_down_resistor(GPIO_PB1, PM_PIN_PULLUP_10K);
	gpio_set_interrupt(GPIO_PB1, POL_FALLING);
	irq_enable();		//Turn on the total interrupt.
	cpu_stall_wakeup(FLD_IRQ_GPIO_EN);
	gpio_clr_irq_status(GPIO_IRQ_MASK_GPIO);//Clear the interrupt status.

#elif(PM_MODE==SUSPEND_PAD_WAKEUP)

	cpu_sleep_wakeup(SUSPEND_MODE, PM_WAKEUP_PAD, 0);

#elif(PM_MODE==SUSPEND_32K_RC_WAKEUP||PM_MODE==SUSPEND_32K_XTAL_WAKEUP)

	cpu_sleep_wakeup(SUSPEND_MODE, PM_WAKEUP_TIMER, clock_time() + 500*CLOCK_16M_SYS_TIMER_CLK_1MS);

#elif(PM_MODE==SUSPEND_LONG_32K_RC_WAKEUP||PM_MODE==SUSPEND_LONG_32K_XTAL_WAKEUP)

	cpu_long_sleep_wakeup(SUSPEND_MODE, PM_WAKEUP_TIMER, 500*CLOCK_32K_SYS_TIMER_CLK_1MS);

#endif

#if !CURRENT_TEST
	gpio_toggle(LED1);
#endif
	sleep_ms(1000);

}




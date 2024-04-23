/********************************************************************************************************
 * @file	master.c
 *
 * @brief	This is the source file for B85m
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
#if(UART_FW_UPDATE_ROLE == MASTER)
#define FW_UPDATE_MASTER_BIN_ADDR   0x20000
#define FW_UPDATE_FW_VERSION        0x0001

#define BATT_CHECK_ENABLE       0
#define VBAT_ALRAM_THRES_MV     2000

#if (MCU_CORE_B80)
#define BLUE_LED_PIN     		        GPIO_PA4
#define GREEN_LED_PIN     		        GPIO_PA5
#define WHITE_LED_PIN     		        GPIO_PA6
#define RED_LED_PIN     		        GPIO_PA7
#elif (MCU_CORE_B80B)
#define BLUE_LED_PIN                    GPIO_PB3
#define GREEN_LED_PIN                   GPIO_PB4
#define WHITE_LED_PIN                   GPIO_PB5
#define RED_LED_PIN                     GPIO_PB6
#endif
#define GPIO_IRQ_PIN				GPIO_PF0 //B80:SW2 B80B:SW7

unsigned long firmwareVersion;
volatile unsigned char FW_UPDATE_MasterTrig = 0;

#if(BATT_CHECK_ENABLE)
static unsigned char  battery_power_check()
{
	volatile unsigned char i;
	volatile unsigned short sample_result = 0;
	adc_init();
    adc_vbat_channel_init();
	adc_power_on_sar_adc(1);
	WaitMs(1);
	for(i = 0; i < 4; i++)
	{
		sample_result = adc_sample_and_get_result();
		if(sample_result < VBAT_ALRAM_THRES_MV)
		{
			return 0;
		}
	}
	return 1;
}
#endif

void user_init(void)
{
    // indicate LED Pins
    gpio_set_func(BLUE_LED_PIN|GREEN_LED_PIN|WHITE_LED_PIN|RED_LED_PIN, AS_GPIO);
	gpio_set_output_en(BLUE_LED_PIN|GREEN_LED_PIN|WHITE_LED_PIN|RED_LED_PIN, 1); //enable output
    gpio_set_input_en(BLUE_LED_PIN|GREEN_LED_PIN|WHITE_LED_PIN|RED_LED_PIN, 0); //disable input
    gpio_write(BLUE_LED_PIN|GREEN_LED_PIN|WHITE_LED_PIN|RED_LED_PIN, 0);

#if (MCU_CORE_B80)
    gpio_set_func(GPIO_PA0 ,AS_GPIO);
	gpio_set_output_en(GPIO_PA0, 1); 		//enable output
	gpio_set_input_en(GPIO_PA0 ,0);			//disable input
	gpio_write(GPIO_PA0, 0);
#endif
    gpio_set_func(GPIO_IRQ_PIN, AS_GPIO);
    gpio_set_output_en(GPIO_IRQ_PIN, 0);            // disable output
    gpio_set_input_en(GPIO_IRQ_PIN, 1);             // enable input
    gpio_setup_up_down_resistor(GPIO_IRQ_PIN, PM_PIN_PULLUP_10K);
    gpio_set_interrupt(GPIO_IRQ_PIN, POL_FALLING);

    //enable the irq
    gpio_set_interrupt(GPIO_IRQ_PIN, POL_FALLING);
    irq_enable_type(FLD_IRQ_GPIO_EN);
    irq_enable();
}

int main(void)
{
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    wd_32k_stop();

	user_read_flash_value_calib();

    clock_init(SYS_CLK_24M_Crystal);

    user_init();

    while (1)
    {
        if (FW_UPDATE_MasterTrig)
        {
            FW_UPDATE_MasterTrig = 0;

#if (BATT_CHECK_ENABLE)
            if(!battery_power_check())
            {
                while(1)
                {
                    gpio_toggle(RED_LED_PIN);
                    WaitMs(50);
                }
            }
#endif
            gpio_toggle(BLUE_LED_PIN);
            WaitMs(100);
            gpio_toggle(BLUE_LED_PIN);
            WaitMs(100);
            gpio_toggle(BLUE_LED_PIN);
            WaitMs(100);
            gpio_toggle(BLUE_LED_PIN);
            WaitMs(100);

            FW_UPDATE_PHY_Init(FW_UPDATE_RxIrq);
            FW_UPDATE_MasterInit(FW_UPDATE_MASTER_BIN_ADDR, FW_UPDATE_FW_VERSION);

            while (1)
            {
                FW_UPDATE_MasterStart();
                ev_process_timer();
            }
        }
        gpio_toggle(GREEN_LED_PIN);
        WaitMs(1000);
    }

    return 0;
}
#endif

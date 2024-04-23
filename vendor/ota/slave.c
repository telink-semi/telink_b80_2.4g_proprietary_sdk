/********************************************************************************************************
 * @file	slave.c
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
#if(OTA_ROLE == SLAVE1 || OTA_ROLE == SLAVE2)
#define OTA_SLAVE_PANID         0xcafe
#define OTA_SLAVE_CHANNEL       70
#define OTA_FW_VERSION          0x0000



#define BATT_CHECK_ENABLE       1
#define VBAT_ALRAM_THRES_MV     2000

#define Flash_Addr				0x08
#define Flash_Buff_Len			1

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
#define OTA_SLAVE_TRIG_PIN      GPIO_PF0

unsigned long firmwareVersion;
volatile unsigned char OTA_SlaveTrig = 0;
volatile unsigned char Flash_Read_Buff[Flash_Buff_Len]={0};


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
#if (MCU_CORE_B80)
    // key related pins config
    gpio_set_func(GPIO_PA0 ,AS_GPIO);
	gpio_set_output_en(GPIO_PA0, 1); 		//enable output
	gpio_set_input_en(GPIO_PA0 ,0);			//disable input
	gpio_write(GPIO_PA0, 0);
#endif
    gpio_set_func(OTA_SLAVE_TRIG_PIN, AS_GPIO);
    gpio_set_output_en(OTA_SLAVE_TRIG_PIN, 0);            // disable output
    gpio_set_input_en(OTA_SLAVE_TRIG_PIN, 1);             // enable input
    gpio_setup_up_down_resistor(OTA_SLAVE_TRIG_PIN, PM_PIN_PULLUP_10K);
    gpio_set_interrupt(OTA_SLAVE_TRIG_PIN, POL_FALLING);

    // enable the irq
    gpio_set_interrupt(OTA_SLAVE_TRIG_PIN, POL_FALLING);//press SW7(eaglet_B)/SW2(eaglet) to trigger ota
    irq_enable_type(FLD_IRQ_GPIO_EN);
    irq_enable();

    // indicate LED Pins
    gpio_set_func(BLUE_LED_PIN|GREEN_LED_PIN|WHITE_LED_PIN|RED_LED_PIN, AS_GPIO);
	gpio_set_output_en(BLUE_LED_PIN|GREEN_LED_PIN|WHITE_LED_PIN|RED_LED_PIN, 1); //enable output
    gpio_set_input_en(BLUE_LED_PIN|GREEN_LED_PIN|WHITE_LED_PIN|RED_LED_PIN, 0); //disable input
    gpio_write(BLUE_LED_PIN|GREEN_LED_PIN|WHITE_LED_PIN|RED_LED_PIN, 0);
}

int main(void)
{
//	blc_pm_select_external_32k_crystal();
	blc_pm_select_internal_32k_crystal();

    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    wd_32k_stop();

	user_read_flash_value_calib();

    clock_init(SYS_CLK_24M_Crystal);

    user_init();

	while (1)
	{
		if (OTA_SlaveTrig)
		{
			OTA_SlaveTrig = 0;

#if(BATT_CHECK_ENABLE)
            if (!battery_power_check())
            {
                while(1)
                {
                     gpio_toggle(RED_LED_PIN);
                     WaitMs(50);
                }
            }
#endif
            MAC_Init(OTA_SLAVE_CHANNEL,
                     OTA_RxIrq,
                     OTA_RxTimeoutIrq,
                     OTA_RxTimeoutIrq);

			flash_read_page(Flash_Addr,Flash_Buff_Len, (unsigned char *)Flash_Read_Buff);
			if (Flash_Read_Buff[0] == 0x4b)
			{
				OTA_SlaveInit(OTA_SLAVE_BIN_ADDR, OTA_FW_VERSION);
			}
			else
			{
				OTA_SlaveInit(0, OTA_FW_VERSION);
			}
			gpio_write(BLUE_LED_PIN,1);
			WaitMs(80);
			gpio_write(BLUE_LED_PIN,0);
			WaitMs(80);
			gpio_write(BLUE_LED_PIN,1);
			WaitMs(80);
			gpio_write(BLUE_LED_PIN,0);

			while (1)
			{
				OTA_SlaveStart();
			}
		}
#if(OTA_ROLE == SLAVE1)
        gpio_toggle(GREEN_LED_PIN);
#elif(OTA_ROLE == SLAVE2)
        gpio_toggle(WHITE_LED_PIN);
#endif
        WaitMs(1000);
	}
}
#endif

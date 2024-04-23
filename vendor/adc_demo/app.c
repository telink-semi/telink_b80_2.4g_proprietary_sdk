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
#if (MCU_CORE_B80)
#define GREEN_LED_PIN           GPIO_PA5
#elif(MCU_CORE_B80B)
#define GREEN_LED_PIN           GPIO_PB4
#endif
volatile unsigned short base_val;
volatile unsigned short vbat_val;
volatile signed short temp_new_val;
volatile unsigned short vbat_channel_val;
volatile unsigned short adc_manual_val;
volatile unsigned int rns_val;
volatile unsigned short sample_result[16] = {0};
volatile unsigned short rand_result[16] = {0};
unsigned char i = 0;

void user_init()
{
    // led init
    gpio_set_output_en(GREEN_LED_PIN, 1); //enable output
    gpio_set_input_en(GREEN_LED_PIN, 0); //disable input
    gpio_write(GREEN_LED_PIN, 0); //LED Off


#if(ADC_MODE == ADC_RNG_MODE)
	 random_generator_init();
#else
	adc_init();

	#if(ADC_MODE == ADC_BASE_MODE)
	 adc_base_init(ADC_GPIO_PB0);
	 adc_set_ain_pre_scaler(ADC_PRESCALER_1F8);//ADC pre_scaling default value is ADC_PRESCALER_1F8, it can change after adc_base_init().
	#elif (ADC_MODE == ADC_VBAT_MODE)
		adc_vbat_init(GPIO_PB0);
    #elif (ADC_MODE == ADC_VBAT_CHANNEL_MODE)
		adc_vbat_channel_init();
    #elif (ADC_MODE == ADC_TEMP_MODE_EE)
		adc_temp_init();
	#endif

	adc_power_on_sar_adc(1);		//After setting the ADC parameters, turn on the ADC power supply control bit

#endif

}


void main_loop (void)
{

#if(ADC_MODE == ADC_RNG_MODE)
	rand_result[i] = rand();
	i = (i + 1) % 16;
	WaitMs(50);
	gpio_toggle(GREEN_LED_PIN);
#else

	#if(ADC_MODE == ADC_BASE_MODE || ADC_MODE == ADC_VBAT_CHANNEL_MODE)
		sample_result[i] = adc_sample_and_get_result();
		i = (i + 1) % 16;
		WaitMs(50);
		gpio_toggle(GREEN_LED_PIN);
    #elif (ADC_MODE == ADC_TEMP_MODE_EE)
		temp_new_val = adc_temp_result();
	#endif

    #if(MANUAL_MODE_GET_ADC_SAMPLE_RESULT == 1)
		adc_manual_val = adc_sample_and_get_result_manual_mode();
    #endif

#endif

}



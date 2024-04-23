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

#if (MCU_CORE_B80)
#define LED1     		        GPIO_PA4
#define LED2     		        GPIO_PA5
#define LED3     		        GPIO_PA6
#define LED4     		        GPIO_PA7
#elif (MCU_CORE_B80B)
#define LED1                    GPIO_PB3
#define LED2                    GPIO_PB4
#define LED3                    GPIO_PB5
#define LED4                    GPIO_PB6
#endif

void user_init(void)
{
    gpio_set_func(LED1|LED2|LED3|LED4 ,AS_GPIO);
    gpio_set_output_en(LED1|LED2|LED3|LED4, 1); 		//enable output
    gpio_set_input_en(LED1|LED2|LED3|LED4 ,0);			//disable input
    gpio_write(LED1|LED2|LED3|LED4, 0);              	//LED On
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
        gpio_toggle(LED1|LED2|LED3|LED4);
        WaitMs(1000);
	}

    return 0;
}


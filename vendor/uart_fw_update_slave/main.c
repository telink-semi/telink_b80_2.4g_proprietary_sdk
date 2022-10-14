/********************************************************************************************************
 * @file	main.c
 *
 * @brief	This is the source file for b80
 *
 * @author	2.4G Group
 * @date	2021
 *
 * @par     Copyright (c) 2021, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *          All rights reserved.
 *
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions are met:
 *
 *              1. Redistributions of source code must retain the above copyright
 *              notice, this list of conditions and the following disclaimer.
 *
 *              2. Unless for usage inside a TELINK integrated circuit, redistributions
 *              in binary form must reproduce the above copyright notice, this list of
 *              conditions and the following disclaimer in the documentation and/or other
 *              materials provided with the distribution.
 *
 *              3. Neither the name of TELINK, nor the names of its contributors may be
 *              used to endorse or promote products derived from this software without
 *              specific prior written permission.
 *
 *              4. This software, with or without modification, must only be used with a
 *              TELINK integrated circuit. All other usages are subject to written permission
 *              from TELINK and different commercial license may apply.
 *
 *              5. Licensee shall be solely responsible for any claim to the extent arising out of or
 *              relating to such deletion(s), modification(s) or alteration(s).
 *
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *          ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *          WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *          DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
 *          DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *          (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *          LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *          ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *          (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *          SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************************************/
#include "common.h"
#include "driver.h"
#include "fw_update_phy.h"
#include "fw_update.h"

#define FW_UPDATE_SLAVE_BIN_ADDR     0x20000
#define FW_UPDATE_FW_VERSION         0x0000

#define GREEN_LED_PIN               GPIO_PA5
#define WHITE_LED_PIN               GPIO_PA6
#define RED_LED_PIN                 GPIO_PA7
#define GPIO_IRQ_PIN                GPIO_PF0 //SW2

unsigned long firmwareVersion;
volatile unsigned char FW_UPDATE_SlaveTrig = 0;

void user_init(void)
{
    gpio_set_func(GREEN_LED_PIN, AS_GPIO);
    gpio_set_output_en(GREEN_LED_PIN, 1);           //enable output
    gpio_set_input_en(GREEN_LED_PIN, 0);            //disable input
    gpio_write(GREEN_LED_PIN, 0);

    gpio_set_func(GPIO_PA0 ,AS_GPIO);
  	gpio_set_output_en(GPIO_PA0, 1); 		//enable output
  	gpio_set_input_en(GPIO_PA0 ,0);			//disable input
  	gpio_write(GPIO_PA0, 0);

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
        if (FW_UPDATE_SlaveTrig)
        {
            FW_UPDATE_SlaveTrig = 0;
            gpio_toggle(GREEN_LED_PIN);
            WaitMs(100);
            gpio_toggle(GREEN_LED_PIN);
            WaitMs(100);
            gpio_toggle(GREEN_LED_PIN);
			WaitMs(100);
			gpio_toggle(GREEN_LED_PIN);
			WaitMs(100);

            FW_UPDATE_PHY_Init(FW_UPDATE_RxIrq);
            FW_UPDATE_SlaveInit(FW_UPDATE_SLAVE_BIN_ADDR, FW_UPDATE_FW_VERSION);

            while (1)
            {
                FW_UPDATE_SlaveStart();
                ev_process_timer();
            }
        }
        gpio_toggle(GREEN_LED_PIN);
        WaitMs(1000);
    }
    return 0;
}





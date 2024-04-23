/********************************************************************************************************
 * @file	interrupt.c
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
#if(OTA_ROLE == MASTER)
#define OTA_MASTER_TRIG_PIN    GPIO_PF0

extern volatile unsigned char OTA_MasterTrig;
volatile unsigned char tx_done_cnt, rx_done_cnt, first_timeout_done, rx_timeout_done;
_attribute_ram_code_sec_noinline_ __attribute__((optimize("-Os"))) void irq_handler(void)
{

    unsigned int irq_src = irq_get_src();
    unsigned short src_rf = rf_irq_src_get();

    if (irq_src & FLD_IRQ_GPIO_EN)
    {
    	if (0 == gpio_read(OTA_MASTER_TRIG_PIN))//press SW7(eaglet_B)/SW2(eaglet) to trigger ota
    	{
			WaitUs(10);
			if (0 == gpio_read(OTA_MASTER_TRIG_PIN))
			{
				while(0 == gpio_read(OTA_MASTER_TRIG_PIN));
				OTA_MasterTrig = 1;
			}
		}
    }

    if (irq_src & FLD_IRQ_ZB_RT_EN)
    {
        if (src_rf & FLD_RF_IRQ_RX)
        {
        	rx_done_cnt++;
            MAC_RxIrqHandler();
        }

        if (src_rf & FLD_RF_IRQ_RX_TIMEOUT)
        {
        	rx_timeout_done++;
            MAC_RxTimeOutHandler();
        }

        if (src_rf & FLD_RF_IRQ_TX)
        {
        	tx_done_cnt++;
            rf_irq_clr_src(FLD_RF_IRQ_TX);
        }
    }
    rf_irq_clr_src(FLD_RF_IRQ_ALL);
    irq_clr_src();

}
#elif(OTA_ROLE == SLAVE1 || OTA_ROLE == SLAVE2)

#define OTA_SLAVE_TRIG_PIN    GPIO_PF0

extern volatile unsigned char OTA_SlaveTrig;
unsigned char rx_test_cnt;
_attribute_ram_code_sec_noinline_ __attribute__((optimize("-Os"))) void irq_handler(void)
{

    unsigned int irq_src = irq_get_src();
    unsigned short src_rf = rf_irq_src_get();

    if (irq_src & FLD_IRQ_GPIO_EN) {
    	if (0 == gpio_read(OTA_SLAVE_TRIG_PIN)) {//press SW7(eaglet_B)/SW2(eaglet) to trigger ota
			WaitUs(10);
			if (0 == gpio_read(OTA_SLAVE_TRIG_PIN)) {
				while(0 == gpio_read(OTA_SLAVE_TRIG_PIN));
				OTA_SlaveTrig = 1;
			}
		}
    }

    if (irq_src & FLD_IRQ_ZB_RT_EN) {
        if (src_rf & FLD_RF_IRQ_RX) {
        	rx_test_cnt++;
            MAC_RxIrqHandler();
        }
        if (src_rf & FLD_RF_IRQ_RX_TIMEOUT) {
            MAC_RxTimeOutHandler();
        }
        if (src_rf & FLD_RF_IRQ_FIRST_TIMEOUT) {
            MAC_RxFirstTimeOutHandler();
        }
    }
    rf_irq_clr_src(FLD_RF_IRQ_ALL);
    irq_clr_src();

}
#endif



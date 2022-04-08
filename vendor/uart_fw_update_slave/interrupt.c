/********************************************************************************************************
 * @file	interrupt.c
 *
 * @brief	This is the source file for Eaglet
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
#define  GPIO_IRQ_PIN           GPIO_PF0

extern volatile unsigned char FW_UPDATE_SlaveTrig;
volatile unsigned int tx_cnt = 0;
volatile unsigned int rx_cnt = 0;
__attribute__((section(".ram_code"))) __attribute__((optimize("-Os")))   void irq_handler(void)
{
	unsigned char uart_dma_irqsrc= dma_chn_irq_status_get();
	unsigned int irq_src = irq_get_src();
	//gpio irq
    if (irq_src & FLD_IRQ_GPIO_EN)
    {
    	 if (0 == gpio_read(GPIO_IRQ_PIN))
    	 {
    		 WaitMs(10);
    		 if (0 == gpio_read(GPIO_IRQ_PIN))
    		 {
    			 while(0 == gpio_read(GPIO_IRQ_PIN));
    			 FW_UPDATE_SlaveTrig = 1;
    		 }
    	 }
    }
    //uart irq
    if(uart_dma_irqsrc & FLD_DMA_CHN_UART_RX)
    	{
    		dma_chn_irq_status_clr(FLD_DMA_CHN_UART_RX);
    		FW_UPDATE_PHY_RxIrqHandler();
    		rx_cnt ++;
    	}
        if(uart_dma_irqsrc & FLD_DMA_CHN_UART_TX)
        {
            dma_chn_irq_status_clr(FLD_DMA_CHN_UART_TX);
            tx_cnt ++;
            FW_UPDATE_PHY_TxIrqHandler();
        }

        irq_clr_src();

}

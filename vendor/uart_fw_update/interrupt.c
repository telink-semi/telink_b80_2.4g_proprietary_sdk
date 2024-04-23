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
#if(UART_FW_UPDATE_ROLE == MASTER)

#define GPIO_IRQ_PIN			GPIO_PF0	//SW7

extern volatile unsigned char FW_UPDATE_MasterTrig;

_attribute_session_(".ram_code") void irq_handler(void)
{
	#if(MCU_CORE_B80B)
	unsigned int irq_src = irq_get_src();
	//1. UART irq
	if(dma_chn_irq_status_get(FLD_DMA_CHN_UART_RX))
	{
		dma_chn_irq_status_clr(FLD_DMA_CHN_UART_RX);
		FW_UPDATE_PHY_RxIrqHandler();
	}
    if(dma_chn_irq_status_get(FLD_DMA_CHN_UART_TX))
    {
        dma_chn_irq_status_clr(FLD_DMA_CHN_UART_TX);

        FW_UPDATE_PHY_TxIrqHandler();
    }
	#elif(MCU_CORE_B80)
	unsigned char uart_dma_irqsrc= dma_chn_irq_status_get();
	unsigned int irq_src = irq_get_src();
	//1. UART irq
	if(uart_dma_irqsrc & FLD_DMA_CHN_UART_RX)
	{
		dma_chn_irq_status_clr(FLD_DMA_CHN_UART_RX);
		FW_UPDATE_PHY_RxIrqHandler();
	}
    if(uart_dma_irqsrc & FLD_DMA_CHN_UART_TX)
    {
        dma_chn_irq_status_clr(FLD_DMA_CHN_UART_TX);

        FW_UPDATE_PHY_TxIrqHandler();
    }
	#endif
    if (irq_src & FLD_IRQ_GPIO_EN)
    {
    	 if (0 == gpio_read(GPIO_IRQ_PIN))
    	 {
    		 WaitMs(10);
    		 if (0 == gpio_read(GPIO_IRQ_PIN))
    		 {
    			 while(0 == gpio_read(GPIO_IRQ_PIN));
    			 FW_UPDATE_MasterTrig = 1;
    		 }
    	 }
    }
    //irq_clr_src2(FLD_IRQ_GPIO_EN);
    irq_clr_src();
}
#elif(UART_FW_UPDATE_ROLE == SLAVE1 || UART_FW_UPDATE_ROLE == SLAVE2)
#define  GPIO_IRQ_PIN           GPIO_PF0

extern volatile unsigned char FW_UPDATE_SlaveTrig;
volatile unsigned int tx_cnt = 0;
volatile unsigned int rx_cnt = 0;
__attribute__((section(".ram_code"))) __attribute__((optimize("-Os")))   void irq_handler(void)
{
#if(MCU_CORE_B80)
	unsigned char uart_dma_irqsrc= dma_chn_irq_status_get();
#endif

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
#if(MCU_CORE_B80B)
    //uart irq
    if(dma_chn_irq_status_get(FLD_DMA_CHN_UART_RX))
    	{
    		dma_chn_irq_status_clr(FLD_DMA_CHN_UART_RX);
    		FW_UPDATE_PHY_RxIrqHandler();
    		rx_cnt ++;
    	}
	if(dma_chn_irq_status_get(FLD_DMA_CHN_UART_TX))
        {
            dma_chn_irq_status_clr(FLD_DMA_CHN_UART_TX);
            tx_cnt ++;
            FW_UPDATE_PHY_TxIrqHandler();
        }
#elif(MCU_CORE_B80)
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
#endif
        irq_clr_src();

}
#endif

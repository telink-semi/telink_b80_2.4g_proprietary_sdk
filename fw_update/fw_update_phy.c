/********************************************************************************************************
 * @file	fw_update_phy.c
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
#include "fw_update_phy.h"
#include "fw_update.h"
#include "driver.h"
#include "common.h"


#define PHY_TX_BUF_LEN          96
#define PHY_RX_BUF_LEN          96
#define PHY_RX_BUF_NUM          3

#define CLOCK_SYS_CLOCK_HZ      24000000
#define UART_TX_PIN_PD0         GPIO_PD0
#define UART_RX_PIN_PC6         GPIO_PC6
#define UART_DATA_LEN    		(96-4)      //data max (UART_DATA_LEN+4) must 16 byte aligned
typedef struct{
    unsigned int dma_len;        // dma len must be 4 byte
    unsigned char data[UART_DATA_LEN];
}uart_data_t;




PHY_Cb_t PHYRxCb = NULL;
static volatile unsigned char PHY_TxFinished = 0;

static uart_data_t PHY_TxBuf __attribute__ ((aligned (4))) = {};


static uart_data_t PHY_RxBuf[PHY_RX_BUF_NUM] __attribute__ ((aligned (4))) = {};
static unsigned char PHY_RxPtr = 0;



void FW_UPDATE_PHY_Init(const PHY_Cb_t RxCb)
{
    //Set UART Rx irq callback
    PHYRxCb = RxCb;

    //config UART module
    uart_recbuff_init(  (unsigned char *)&PHY_RxBuf[PHY_RxPtr], PHY_RX_BUF_LEN);

    uart_gpio_set(UART_TX_PIN_PD0, UART_RX_PIN_PC6);

    uart_reset();  //will reset uart digital registers from 0x90 ~ 0x9f, so uart setting must set after this reset

    uart_init_baudrate(115200, CLOCK_SYS_CLOCK_HZ, PARITY_NONE, STOP_BIT_ONE);

    uart_dma_enable(1, 1);     //uart data in hardware buffer moved by dma, so we need enable them first

	irq_set_mask(FLD_IRQ_DMA_EN);

	dma_chn_irq_enable(FLD_DMA_CHN_UART_RX | FLD_DMA_CHN_UART_TX, 1);       //uart Rx/Tx dma irq enable

	uart_irq_enable(0, 0);      //uart Rx/Tx irq no need, disable them

}

int FW_UPDATE_PHY_SendData(const unsigned char *Payload, const int PayloadLen)
{
    //set UART DMA length
	PHY_TxBuf.dma_len = PayloadLen;

    //fill the contents of UART transmission
    memcpy(&PHY_TxBuf.data, Payload, PayloadLen);

//    uart_dma_send((unsigned char*)&PHY_TxBuf);
    uart_send_dma((unsigned char*)&PHY_TxBuf);
    while(!PHY_TxFinished);
    PHY_TxFinished = 0;

    return PayloadLen;
}

void FW_UPDATE_PHY_RxIrqHandler(void)
{
    //set next rx_buf
	unsigned char *RxPacket = PHY_RxBuf[PHY_RxPtr].data;
	PHY_RxPtr = (PHY_RxPtr + 1) % PHY_RX_BUF_NUM;
	uart_recbuff_init((unsigned char *)&PHY_RxBuf[PHY_RxPtr], PHY_RX_BUF_LEN);
    if (PHYRxCb) {
        PHYRxCb(&RxPacket[0]);
    }
}

void FW_UPDATE_PHY_TxIrqHandler(void)
{
    PHY_TxFinished = 1;
}

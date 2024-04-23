/********************************************************************************************************
 * @file    fw_update_phy.c
 *
 * @brief   This is the source file for B80
 *
 * @author  2.4G Group
 * @date    2021
 *
 * @par     Copyright (c) 2021, Telink Semiconductor (Shanghai) Co., Ltd.
 *          All rights reserved.
 *
 *          The information contained herein is confidential property of Telink
 *          Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *          of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *          Co., Ltd. and the licensee or the terms described here-in. This heading
 *          MUST NOT be removed from this file.
 *
 *          Licensee shall not delete, modify or alter (or permit any third party to delete, modify, or
 *          alter) any information contained herein in whole or in part except as expressly authorized
 *          by Telink semiconductor (shanghai) Co., Ltd. Otherwise, licensee shall be solely responsible
 *          for any claim to the extent arising out of or relating to such deletion(s), modification(s)
 *          or alteration(s).
 *
 *          Licensees are granted free, non-transferable use of the information in this
 *          file under Mutual Non-Disclosure Agreement. NO WARRANTY of ANY KIND is provided.
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
#if(MCU_CORE_B80B)
    //config UART module
    uart_recbuff_init( 0, (unsigned char *)&PHY_RxBuf[PHY_RxPtr], PHY_RX_BUF_LEN);

    uart_gpio_set(0, UART_TX_PIN_PD0, UART_RX_PIN_PC6);

    uart_reset(0);  //will reset uart digital registers from 0x90 ~ 0x9f, so uart setting must set after this reset

    uart_init_baudrate(0, 115200, CLOCK_SYS_CLOCK_HZ, PARITY_NONE, STOP_BIT_ONE);

    uart_dma_enable(0, 1, 1);     //uart data in hardware buffer moved by dma, so we need enable them first

	irq_set_mask(FLD_IRQ_DMA_EN);

	dma_chn_irq_enable(FLD_DMA_CHN_UART_RX | FLD_DMA_CHN_UART_TX, 1);       //uart Rx/Tx dma irq enable

	uart_irq_enable(0, 0, 0);      //uart Rx/Tx irq no need, disable them
#elif(MCU_CORE_B80)
    //config UART module
    uart_recbuff_init((unsigned char *)&PHY_RxBuf[PHY_RxPtr], PHY_RX_BUF_LEN);

    uart_gpio_set(UART_TX_PIN_PD0, UART_RX_PIN_PC6);

    uart_reset();  //will reset uart digital registers from 0x90 ~ 0x9f, so uart setting must set after this reset

    uart_init_baudrate(115200, CLOCK_SYS_CLOCK_HZ, PARITY_NONE, STOP_BIT_ONE);

    uart_dma_enable(1, 1);     //uart data in hardware buffer moved by dma, so we need enable them first

	irq_set_mask(FLD_IRQ_DMA_EN);

	dma_chn_irq_enable(FLD_DMA_CHN_UART_RX | FLD_DMA_CHN_UART_TX, 1);       //uart Rx/Tx dma irq enable

	uart_irq_enable(0, 0);      //uart Rx/Tx irq no need, disable them
#endif

}

int FW_UPDATE_PHY_SendData(const unsigned char *Payload, const int PayloadLen)
{
    //set UART DMA length
	PHY_TxBuf.dma_len = PayloadLen;

    //fill the contents of UART transmission
    memcpy(&PHY_TxBuf.data, Payload, PayloadLen);

//    uart_dma_send((unsigned char*)&PHY_TxBuf);
#if(MCU_CORE_B80B)
    uart_send_dma(0, (unsigned char*)&PHY_TxBuf);
#elif(MCU_CORE_B80)
    uart_send_dma((unsigned char*)&PHY_TxBuf);
#endif
    while(!PHY_TxFinished);
    PHY_TxFinished = 0;

    return PayloadLen;
}

void FW_UPDATE_PHY_RxIrqHandler(void)
{
    //set next rx_buf
	unsigned char *RxPacket = PHY_RxBuf[PHY_RxPtr].data;
	PHY_RxPtr = (PHY_RxPtr + 1) % PHY_RX_BUF_NUM;
#if(MCU_CORE_B80B)
	uart_recbuff_init(0, (unsigned char *)&PHY_RxBuf[PHY_RxPtr], PHY_RX_BUF_LEN);
#elif(MCU_CORE_B80)
	uart_recbuff_init((unsigned char *)&PHY_RxBuf[PHY_RxPtr], PHY_RX_BUF_LEN);
#endif
    if (PHYRxCb) {
        PHYRxCb(&RxPacket[0]);
    }
}

void FW_UPDATE_PHY_TxIrqHandler(void)
{
    PHY_TxFinished = 1;
}

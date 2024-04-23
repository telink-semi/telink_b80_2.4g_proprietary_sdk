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
#include "driver.h"
#include "tpll.h"

volatile unsigned int tx_irq_cnt_invalid_pid, tx_irq_cnt_max_retry, tx_irq_cnt_tx_ds,
                tx_irq_cnt_tx, tx_irq_cnt_rx, tx_irq_cnt_rx_dr = 0;

extern volatile unsigned char rx_flag, rx_dr_flag, ds_flag, maxretry_flag;

__attribute__((section(".ram_code")))__attribute__((optimize("-Os"))) void irq_handler(void)
{
    unsigned short src_rf = rf_irq_src_get();
    unsigned char pipe = TPLL_GetTXPipe();
    if (src_rf & FLD_RF_IRQ_TX)
    {
        rf_irq_clr_src(FLD_RF_IRQ_TX);
        tx_irq_cnt_tx++;
    }

    if (src_rf & FLD_RF_IRQ_INVALID_PID)
    {
        rf_irq_clr_src(FLD_RF_IRQ_INVALID_PID);
        tx_irq_cnt_invalid_pid++;
    }

    if (src_rf & FLD_RF_IRQ_TX_RETRYCNT)
    {
        rf_irq_clr_src(FLD_RF_IRQ_TX_RETRYCNT);
        tx_irq_cnt_max_retry++;
        maxretry_flag = 1;
        //adjust rptr
        TPLL_UpdateTXFifoRptr(pipe);
    }

    if (src_rf & FLD_RF_IRQ_TX_DS)
    {
        rf_irq_clr_src(FLD_RF_IRQ_TX_DS);
        tx_irq_cnt_tx_ds++;
        ds_flag = 1;
    }

    if (src_rf & FLD_RF_IRQ_RX_DR)
    {
        rf_irq_clr_src(FLD_RF_IRQ_RX_DR);
        tx_irq_cnt_rx_dr++;
        rx_dr_flag = 1;
    }

    if (src_rf & FLD_RF_IRQ_RX)
    {
        rf_irq_clr_src(FLD_RF_IRQ_RX);
        tx_irq_cnt_rx++;
        rx_flag = 1;
    }
    irq_clr_src();
    rf_irq_clr_src(FLD_RF_IRQ_ALL);
}

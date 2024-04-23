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
#include "../tl_tpll/tl_tpll.h"

volatile unsigned int rx_irq_cnt_tx, rx_irq_cnt_rx,
                rx_irq_cnt_invalid_pid, rx_irq_cnt_tx_ds;
//extern void trf_tpll_event_handler(trf_tpll_event_id_t evt_id);
static trf_tpll_event_handler_t m_event_handler;
__attribute__((section(".ram_code"))) __attribute__((optimize("-Os")))
        void irq_handler(void)
{
    unsigned short src_rf = rf_irq_src_get();
    m_event_handler = trf_tpll_get_event_handler();
    if (src_rf & FLD_RF_IRQ_RX)
    {
        rf_irq_clr_src(FLD_RF_IRQ_RX);
        trf_tpll_rxirq_handler(m_event_handler);
        rx_irq_cnt_rx++;
    }
    if (src_rf & FLD_RF_IRQ_TX)
    {
        rf_irq_clr_src(FLD_RF_IRQ_TX);
        m_event_handler(TRF_TPLL_EVENT_TX_FINISH);
        rx_irq_cnt_tx++;
    }
    if (src_rf & FLD_RF_IRQ_TX_DS)
    {
        rf_irq_clr_src(FLD_RF_IRQ_TX_DS);
        rx_irq_cnt_tx_ds++;
    }
    if (src_rf & FLD_RF_IRQ_INVALID_PID)
    {
        rf_irq_clr_src(FLD_RF_IRQ_INVALID_PID);
        rx_irq_cnt_invalid_pid++;
    }
    irq_clr_src();
    rf_irq_clr_src(FLD_RF_IRQ_ALL);
}




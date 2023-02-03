/********************************************************************************************************
 * @file    interrupt.c
 *
 * @brief   This is the source file for b80
 *
 * @author  2.4G Group
 * @date    2021
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
#include "driver.h"
#include "tl_tpll.h"

volatile unsigned int tx_irq_cnt_invalid_pid, tx_irq_cnt_max_retry,
tx_irq_cnt_tx_ds, tx_irq_cnt_tx, tx_irq_cnt_rx, tx_irq_cnt_rx_dr = 0;

//extern void trf_tpll_event_handler(trf_tpll_event_id_t evt_id);

static trf_tpll_event_handler_t m_event_handler;
__attribute__((section(".ram_code")))__attribute__((optimize("-Os")))
        void irq_handler(void)
{
    unsigned short src_rf = rf_irq_src_get();
    unsigned char pipe = trf_tpll_get_txpipe();
    m_event_handler = trf_tpll_get_event_handler();
    if (src_rf & FLD_RF_IRQ_TX)
    {
        rf_irq_clr_src(FLD_RF_IRQ_TX);
        m_event_handler(TRF_TPLL_EVENT_TX_FINISH);
        tx_irq_cnt_tx++;
    }
    if (src_rf & FLD_RF_IRQ_RETRY_HIT)
    {
        rf_irq_clr_src(FLD_RF_IRQ_RETRY_HIT);
        m_event_handler(TRF_TPLL_EVENT_TX_FALIED);
        trf_tpll_update_txfifo_rptr(pipe); //adjust rptr
        tx_irq_cnt_max_retry++;
    }
    if (src_rf & FLD_RF_IRQ_RX)
    {
        rf_irq_clr_src(FLD_RF_IRQ_RX);
//        trf_tpll_event_handler(TRF_TPLL_EVENT_RX_RECEIVED);
        trf_tpll_rxirq_handler(m_event_handler);
        tx_irq_cnt_rx++;
    }
    if (src_rf & FLD_RF_IRQ_TX_DS)
    {
        rf_irq_clr_src(FLD_RF_IRQ_TX_DS);
        tx_irq_cnt_tx_ds++;
    }
    if (src_rf & FLD_RF_IRQ_INVALID_PID)
    {
        rf_irq_clr_src(FLD_RF_IRQ_INVALID_PID);
        tx_irq_cnt_invalid_pid++;
    }
    irq_clr_src();
    rf_irq_clr_src(FLD_RF_IRQ_ALL);
}

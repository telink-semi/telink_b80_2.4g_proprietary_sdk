#include "driver.h"
#include "genfsk_ll.h"
#include "common.h"

#define DEBUG_LOG 0 // if enable debug log, it will affect the timing
//#define DEBUG_NO_SLEEP

#define DEVICE_D_ID 0

#define BLUE_LED_PIN GPIO_PA4
#define GREEN_LED_PIN GPIO_PA5
#define WHITE_LED_PIN GPIO_PA6
#define RED_LED_PIN GPIO_PA7

#define DEBUG_PB6 GPIO_PB6
#define DEBUG_PB7 GPIO_PB7
#define DEBUG_PB0 GPIO_PB0
#define DEBUG_PB1 GPIO_PB1
#define DEBUG_PB2 GPIO_PB2

#define DBG_SUSPEND 1
#define TX_INTERVAL (500 * 1000 * 16)
#define RX_MARGIN (10 * 1000 * 16)
#define RX_MARGIN_PM_CALIB (500 * 16) // for some init after wake up

#define RX_BUF_LEN 64
#define RX_BUF_NUM 4
#define APP_FIX_PAYLOAD_LEN 32
#define SYNC_FIX_PAYLOAD_LEN 1
#define MAX_CHANNEL_MAP_SIZE 4

#define SYNC_BROADCAST_INTERVAL (1250 * 16) // 1.25MS

static unsigned char rx_crc_err_flag = 0;
static unsigned int rx_crc_err_cnt = 0;

#ifdef DEBUG_NO_SLEEP
#define RX_TO_US 600000
#endif

#define B_AC 0X38958D75

enum
{
    RX_SHORT_WINDOW_US = 625,
    RX_NORMAL_WINDOW_US = 1250,
    RX_LONG_WINDOW_US = 2500,
};

#define CLOCK_SYS_CLOCK_HZ 24000000
enum
{
    CLOCK_SYS_CLOCK_1S = CLOCK_SYS_CLOCK_HZ,
    CLOCK_SYS_CLOCK_1MS = (CLOCK_SYS_CLOCK_1S / 1000),
    CLOCK_SYS_CLOCK_1US = (CLOCK_SYS_CLOCK_1S / 1000000),
};

enum
{
    LL_CMD_START = 0,
    LL_SYNC_REQ = 0x01,
    LL_SYNC_RSP = 0x02,
    LL_ACCEPTED = 0X03,
    LL_NOT_ACCEPTED = 0X04,
    LL_CONN_REQ = 0X05,
    LL_CONN_RSP = 0X06,
    LL_CHNL_CLASSIFICATION_REQ = 0X11,
    LL_CHG_CHNL_MAP_REQ = 0X12,
    LL_CHNL_CLASSIFICATION_IND = 0X13,
    LL_SYNC_DATA = 0x14,
    LL_CMD_END
};

typedef struct
{
    u8 opcode;
    u8 ac[4];
    u16 ts_time; // n * 1.25ms
    u16 tx_int;
    u8 chm[10];
    u8 hop;
} rf_pkt_conn_req_t;
rf_pkt_conn_req_t conn_req_pkt;

#define TOTAL_NB_CHNNEL 79
#define CHNL_MAP_LEN 10
#define AFH_NB_CHANNEL_MAX 16
#define AFH_NB_CHANNEL_MIN 8
#define AFH_NB_CHANNEL_ADB (AFH_NB_CHANNEL_MAX - AFH_NB_CHANNEL_MIN)
#define AFH_BAD_CHNL_QLTY_THRES -3
#define AFH_GOOD_CHNL_QLTY_THRES 3

typedef struct ll_comm_ctrl_data
{
    /// 10-bytes channel map array
    u8 chnl_map[CHNL_MAP_LEN];
    u8 chnl_tbl[TOTAL_NB_CHNNEL];
    s8 qlty[TOTAL_NB_CHNNEL];
    u8 afh_mode;
    u8 afh_int_min_s;
    u8 afh_int_max_s;
    u8 snnesn;
    u32 clsf_tick;
    u8 con_tx_cnt;
} ll_comm_ctrl_data_t;
ll_comm_ctrl_data_t ll_ctrl_data;

enum
{
    LL_FLOW_NESN = 0x01,
    LL_FLOW_SN = 0x02,
    LL_FLOW_SENT = 0x04,
    LL_FLOW_RCVD = 0x08,
};

typedef struct sync_ll_rx_packet
{
    // int dma_len;
    u8 type;
    u8 len;
    u8 cmd;
    u8 data[64 - 7];
} __attribute__((packed)) __attribute__((aligned(4))) ll_rx_packet_t;

typedef struct sync_ll_tx_packet
{
    int dma_len;
    u8 type;
    u8 len;
    u8 cmd;
    u8 data[64 - 7];
} __attribute__((packed)) __attribute__((aligned(4))) ll_tx_packet_t;

volatile unsigned int expand_win_us = 0;
volatile unsigned int expand_step_us = 200 * CLOCK_16M_SYS_TIMER_CLK_1US;

#define AUTO_FLUSH_CNT_THRES 10

static unsigned char __attribute__((aligned(4))) tx_buffer[64] = {0};

volatile static unsigned char rx_buf[RX_BUF_LEN * RX_BUF_NUM] __attribute__((aligned(4))) = {};
volatile static unsigned char rx_ptr = 0;
volatile static unsigned char rx_payload_len = 0;
volatile static unsigned char *rx_payload = 0;
volatile static unsigned char *rx_packet = 0;

volatile static unsigned char tx_done_flag, rx_flag, rx_first_timeout_flag = 0;
volatile static unsigned int sleep_cnt, rx_cnt, rx_first_timeout_cnt = 0;
volatile static unsigned char have_rx_to = 0;

volatile static unsigned int rx_timestamp = 0;
volatile static unsigned char rx_rssi = 0;

#if (DEVICE_D_ID == 0)
unsigned char sycn_ack_frame[APP_FIX_PAYLOAD_LEN] = {
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x01, 0x01, 0x01, 0x01, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10,
    0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20,
};
#elif (DEVICE_D_ID == 1)
unsigned char sycn_ack_frame[APP_FIX_PAYLOAD_LEN] = {
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x02, 0x02, 0x02, 0x02, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10,
    0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20,
};
#elif (DEVICE_D_ID == 2)
unsigned char sycn_ack_frame[APP_FIX_PAYLOAD_LEN] = {
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x03, 0x03, 0x03, 0x03, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10,
    0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20,
};
#endif

volatile unsigned char sync_flg = 0;
volatile unsigned int next_rx_point = 0;
volatile int offset_tick = 0;

volatile unsigned char next_chnn_idx = 0;
unsigned short sync_tx_interval_ms = 0;
unsigned char sync_channel_map_size = 0;
unsigned char sync_channel_map[MAX_CHANNEL_MAP_SIZE] = {0};
unsigned int async_tx_cnt = 0, async_rx_cnt = 0, async_first_timeout_cnt = 0;
unsigned int sync_tx_cnt = 0, sync_rx_cnt = 0, sync_first_timeout_cnt = 0;
unsigned char sync_first_tx = 0;
unsigned char need_clsf_flag = 0;

unsigned int rx_to_cnt = 0;
int m_bad_chnl_times = 0;

unsigned char broadcast_chnn_idx = 0;
unsigned char broadcast_channel_map[3] = {8, 18, 28};

_attribute_ram_code_sec_noinline_ __attribute__((optimize("-Os"))) void irq_handler(void)
{
    unsigned int irq_src = irq_get_src();
    unsigned short rf_irq_src = rf_irq_src_get();

    if (irq_src & FLD_IRQ_ZB_RT_EN) // if rf irq occurs
    {
        if (rf_irq_src & FLD_RF_IRQ_RX) // if rf rx irq occurs
        {
            reg_rf_irq_status = FLD_RF_IRQ_RX;

            rx_cnt++;
            rx_packet = rx_buf + rx_ptr * RX_BUF_LEN;
            rx_ptr = (rx_ptr + 1) % RX_BUF_NUM;
            gen_fsk_rx_buffer_set((unsigned char *)(rx_buf + rx_ptr * RX_BUF_LEN), RX_BUF_LEN);
            if (gen_fsk_is_rx_crc_ok((unsigned char *)rx_packet))
            {
                if (sync_flg == 1 && ll_ctrl_data.qlty[ll_ctrl_data.chnl_tbl[next_chnn_idx]] < AFH_GOOD_CHNL_QLTY_THRES)
                    ll_ctrl_data.qlty[ll_ctrl_data.chnl_tbl[next_chnn_idx]]++;
            }
            else
            {
                rx_crc_err_cnt++;
                rx_crc_err_flag = 1;
                if (sync_flg == 1 && ll_ctrl_data.qlty[ll_ctrl_data.chnl_tbl[next_chnn_idx]] > AFH_BAD_CHNL_QLTY_THRES)
                    ll_ctrl_data.qlty[ll_ctrl_data.chnl_tbl[next_chnn_idx]]--;
            }

            rx_flag = 1;
        }

        if (rf_irq_src & FLD_RF_IRQ_FIRST_TIMEOUT) // if rf first timeout irq occurs
        {
            reg_rf_irq_status = FLD_RF_IRQ_FIRST_TIMEOUT;
            rx_first_timeout_flag = 1;
            rx_first_timeout_cnt++;
        }

        if (rf_irq_src & FLD_RF_IRQ_TX) // rf tx irq occurs
        {
            reg_rf_irq_status = FLD_RF_IRQ_TX;

            tx_done_flag = 1;
        }
    }
    irq_clr_src2(FLD_IRQ_ALL);
}

void user_init(void)
{
    unsigned int pin = GREEN_LED_PIN | RED_LED_PIN | WHITE_LED_PIN | BLUE_LED_PIN;

    gpio_set_func(pin, AS_GPIO);
    gpio_set_input_en(pin, 0);
    gpio_set_output_en(pin, 1);
    gpio_write(pin, 0);

    pin = DEBUG_PB6 | DEBUG_PB7;
    gpio_set_func(pin, AS_GPIO);
    gpio_set_input_en(pin, 0);
    gpio_set_output_en(pin, 1);
    gpio_write(pin, 0);

    pin = DEBUG_PB0 | DEBUG_PB1 | DEBUG_PB2;
    gpio_set_func(pin, AS_GPIO);
    gpio_set_input_en(pin, 0);
    gpio_set_output_en(pin, 1);
    gpio_write(pin, 0);

    memset((void *)&ll_ctrl_data, 0, sizeof(ll_ctrl_data));

    unsigned char sync_word[4] = {B_AC & 0XFF, B_AC >> 8 & 0XFF, B_AC >> 16 & 0XFF, B_AC >> 24 & 0XFF};
    gen_fsk_datarate_set(GEN_FSK_DATARATE_1MBPS); // Note that this API must be invoked first before all other APIs
    gen_fsk_preamble_len_set(4);
    gen_fsk_sync_word_len_set(SYNC_WORD_LEN_4BYTE);
    gen_fsk_sync_word_set(GEN_FSK_PIPE0, sync_word); // set pipe0's sync word
    gen_fsk_pipe_open(GEN_FSK_PIPE0);                // enable pipe0's reception
    gen_fsk_tx_pipe_set(GEN_FSK_PIPE0);              // set pipe0 as the TX pipe
    gen_fsk_packet_format_set(GEN_FSK_PACKET_FORMAT_FIXED_PAYLOAD, SYNC_FIX_PAYLOAD_LEN);
    gen_fsk_radio_power_set(GEN_FSK_RADIO_POWER_0DBM);
    gen_fsk_rx_buffer_set((unsigned char *)(rx_buf + rx_ptr * RX_BUF_LEN), RX_BUF_LEN);
    gen_fsk_channel_set(60);                     // set rf freq as 2403.5MHz
    gen_fsk_radio_state_set(GEN_FSK_STATE_AUTO); // set transceiver to basic RX state
    gen_fsk_rx_settle_set(89);

    // irq configuration
    rf_irq_disable(FLD_RF_IRQ_ALL);
    rf_irq_enable(FLD_RF_IRQ_RX | FLD_RF_IRQ_TX | FLD_RF_IRQ_FIRST_TIMEOUT); // enable rf rx and rx first timeout irq
    irq_enable_type(FLD_IRQ_ZB_RT_EN);                                       // enable RF irq
    irq_enable();                                                            // enable general irq

    usb_set_pin_en();
    usb_loginit();
    WaitMs(2000); // delay to ensure USB enumerate done
}

_attribute_ram_code_sec_noinline_ void rf_recovery_init(void)
{
    unsigned char sync_word[4] = {B_AC & 0XFF, B_AC >> 8 & 0XFF, B_AC >> 16 & 0XFF, B_AC >> 24 & 0XFF};
    // it needs to notice that this api is different from vulture / kite
    gen_fsk_datarate_set(GEN_FSK_DATARATE_1MBPS); // Note that this API must be invoked first before all other APIs
    gen_fsk_preamble_len_set(4);
    gen_fsk_sync_word_len_set(SYNC_WORD_LEN_4BYTE);
    gen_fsk_sync_word_set(GEN_FSK_PIPE0, conn_req_pkt.ac); // set pipe0's sync word
    gen_fsk_pipe_open(GEN_FSK_PIPE0);                      // enable pipe0's reception
    gen_fsk_tx_pipe_set(GEN_FSK_PIPE0);                    // set pipe0 as the TX pipe
    gen_fsk_packet_format_set(GEN_FSK_PACKET_FORMAT_FIXED_PAYLOAD, APP_FIX_PAYLOAD_LEN);
    gen_fsk_radio_power_set(GEN_FSK_RADIO_POWER_0DBM);
    gen_fsk_rx_buffer_set((unsigned char *)(rx_buf + rx_ptr * RX_BUF_LEN), RX_BUF_LEN);
    //    gen_fsk_channel_set(6); //set rf freq as 2403.5MHz
    gen_fsk_radio_state_set(GEN_FSK_STATE_AUTO); // set transceiver to basic RX state
    gen_fsk_rx_settle_set(149);
    gen_fsk_rx_settle_set(89);

    // irq configuration
    rf_irq_disable(FLD_RF_IRQ_ALL);
    rf_irq_enable(FLD_RF_IRQ_RX | FLD_RF_IRQ_TX | FLD_RF_IRQ_FIRST_TIMEOUT); // enable rf rx and rx first timeout irq
    irq_enable_type(FLD_IRQ_ZB_RT_EN);                                       // enable RF irq
    irq_enable();                                                            // enable general irq
}

_attribute_ram_code_sec_noinline_ void refresh_chnl_map()
{
    for (int i = 0; i < TOTAL_NB_CHNNEL; i++)
    {
        if (ll_ctrl_data.qlty[i] <= AFH_BAD_CHNL_QLTY_THRES)
        {
            need_clsf_flag = 1;

            BIT_CLR(ll_ctrl_data.chnl_map[i / 8], i % 8);
            ll_ctrl_data.qlty[i] = 0;
        }
    }
}

_attribute_ram_code_sec_noinline_ void proc_user_task()
{
    if (ll_ctrl_data.afh_mode == 1 &&
        clock_time_exceed(ll_ctrl_data.clsf_tick, ll_ctrl_data.afh_int_min_s * 1000 * 1000))
    {

        ll_ctrl_data.clsf_tick = clock_time();
        refresh_chnl_map();
    }
}

void app_sync_init(void);

void app_sync_task(void);

void app_low_energy_task(void);

_attribute_ram_code_sec_noinline_ int main(void)
{
    blc_pm_select_internal_32k_crystal();

    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    wd_32k_stop();

	user_read_flash_value_calib();

    clock_init(SYS_CLK_24M_Crystal);

#if PM_32KRC_CALIBRATION
    pm_32k_rc_offset_init();
    pm_register_tick32kGet_callback(pm_get_32k_rc_calib);
#endif

    user_init();

    app_sync_init();

#if DEBUG_LOG
    log_msg("----slave enter mainloop------- : \r\n", 0, 0);
#endif

    while (1)
    {
        proc_user_task();

        app_sync_task();

        app_low_energy_task();

        pm_32k_freq_track();
    }
    return 0;
}

_attribute_ram_code_sec_noinline_ static void led_blink(unsigned char cnt)
{
    while (cnt--)
    {
        gpio_toggle(GREEN_LED_PIN);
        WaitMs(50);
    }
}

_attribute_ram_code_sec_noinline_ int chn_table_calc(u8 *chn_map, u8 hop)
{
    u8 k = 0, numused = 0;
    u8 tmp_table[TOTAL_NB_CHNNEL];

    for (int i = 0; i < TOTAL_NB_CHNNEL; i++)
    {

        if (chn_map[i >> 3] & BIT(i & 0x07))
        {
            tmp_table[numused++] = i;
        }
    }

    u8 ll = 0;
    for (int i = 0; i < TOTAL_NB_CHNNEL; i++)
    {
        k += hop;
        if (k >= TOTAL_NB_CHNNEL)
        {
            k -= TOTAL_NB_CHNNEL;
        }

        if (chn_map[k >> 3] & BIT(k & 0x7))
        {
            ll_ctrl_data.chnl_tbl[ll] = k;
        }
        else
        {
            u8 m = k;
            while (m >= numused)
            {
                m -= numused;
            }
            ll_ctrl_data.chnl_tbl[ll] = tmp_table[m];
        }
        ll++;
    }
    return numused;
}

_attribute_ram_code_sec_noinline_ void app_sync_init(void)
{
    tx_buffer[0] = SYNC_FIX_PAYLOAD_LEN;
    tx_buffer[1] = 0x00;
    tx_buffer[2] = 0x00;
    tx_buffer[3] = 0x00;
    tx_buffer[4] = LL_SYNC_RSP;

    broadcast_chnn_idx = 2;
    rf_set_channel(broadcast_channel_map[broadcast_chnn_idx], 0);
    gen_fsk_srx_start(clock_time(), 3 * SYNC_BROADCAST_INTERVAL);
}

_attribute_ram_code_sec_noinline_ void ll_snnesn_init()
{
    ll_ctrl_data.snnesn = 0x2; // for slave
}

_attribute_ram_code_sec_noinline_ u8 ll_flow_process(u8 peer, u8 local)
{
    local &= ~LL_FLOW_RCVD;
    local &= ~LL_FLOW_SENT;

#if DEBUG_LOG
    log_msg("----before ll_flow_process------- local snnesn: \r\n", &local, 1);
    log_msg("----before ll_flow_process------- peer snnesn: \r\n", &peer, 1);
#endif

    u8 peer_nesn = peer & LL_FLOW_NESN;
    u8 peer_sn = peer & LL_FLOW_SN ? LL_FLOW_NESN : 0;
    u8 local_nesn = local & LL_FLOW_NESN;
    u8 local_sn = local & LL_FLOW_SN ? LL_FLOW_NESN : 0;

    if (peer_sn == local_nesn && !rx_crc_err_flag)
    {
        local = (local & ~LL_FLOW_NESN) | (peer_sn ? 0 : LL_FLOW_NESN);
        local |= LL_FLOW_RCVD;
    }

    if (rx_crc_err_flag)
    {
        rx_crc_err_flag = 0;
    }

    if (peer_nesn != local_sn) // ACK
    {
        // prepare next packet with SN = peer_nesn;
        local = (local & ~LL_FLOW_SN) | (peer_nesn << 1);
        local |= LL_FLOW_SENT;
    }
    else
    {
        ll_ctrl_data.con_tx_cnt++;
    }

#if DEBUG_LOG
    log_msg("----after ll_flow_process------- local snnesn: \n", &local, 1);
#endif
    return local;
}

_attribute_ram_code_sec_noinline_ void app_sync_task(void)
{
    if (sync_flg != 0)
        return;

    if (1 == tx_done_flag)
    {
        tx_done_flag = 0;
        async_tx_cnt++;
    }

    if (1 == rx_flag)
    {
        rx_flag = 0;
        async_rx_cnt++;
        rx_payload = gen_fsk_rx_payload_get(rx_packet, &rx_payload_len);
        if (rx_payload[0] == LL_SYNC_REQ)
        {
            tx_buffer[4] = LL_SYNC_RSP;
            gen_fsk_stx_start(tx_buffer, clock_time());
            WaitUs(500);

            gen_fsk_packet_format_set(GEN_FSK_PACKET_FORMAT_FIXED_PAYLOAD, APP_FIX_PAYLOAD_LEN);
            tx_buffer[0] = APP_FIX_PAYLOAD_LEN;

            broadcast_chnn_idx = (broadcast_chnn_idx + 1) % 3;
            rf_set_channel(broadcast_channel_map[broadcast_chnn_idx], 0);
            gen_fsk_srx_start(clock_time(), 0);
            return;
        }
        else if (rx_payload[0] == LL_CONN_REQ)
        {

            tx_buffer[4] = LL_CONN_RSP;
            gen_fsk_stx_start(tx_buffer, clock_time());
            sync_first_tx = 1;
            sync_flg = 1;

            return;
        }
    }

    if (1 == rx_first_timeout_flag)
    {
        rx_first_timeout_flag = 0;
        async_first_timeout_cnt++;

        broadcast_chnn_idx = (broadcast_chnn_idx + 1) % 3;
        rf_set_channel(broadcast_channel_map[broadcast_chnn_idx], 0);

        gen_fsk_srx_start(clock_time(), 3 * SYNC_BROADCAST_INTERVAL);
    }
}

_attribute_ram_code_sec_noinline_ void proc_rx_data_1()
{
    ll_rx_packet_t *prx = (ll_rx_packet_t *)rx_payload;

    u8 peer = prx->type >> 2 & 3;
    ll_ctrl_data.snnesn = ll_flow_process(peer, ll_ctrl_data.snnesn);
    if (!(ll_ctrl_data.snnesn & LL_FLOW_SENT))
    {
        if (ll_ctrl_data.con_tx_cnt < AUTO_FLUSH_CNT_THRES)
        {
            return;
        }
    }

    ll_ctrl_data.con_tx_cnt = 0;

    ll_tx_packet_t *ptx = (ll_tx_packet_t *)tx_buffer;
    unsigned char cmd = prx->cmd;

    if (cmd == LL_ACCEPTED)
    {
        // do something
    }
    else if (cmd == LL_CHG_CHNL_MAP_REQ)
    {
        ptx->len = 2;
        ptx->cmd = LL_ACCEPTED;
        ptx->data[0] = LL_CHG_CHNL_MAP_REQ;
    }
    else if (cmd == LL_CHNL_CLASSIFICATION_REQ)
    {
        ptx->len = 2;
        ptx->cmd = LL_ACCEPTED;
        ptx->data[0] = LL_CHNL_CLASSIFICATION_REQ;
    }
    else if (cmd = LL_SYNC_DATA)
    {
        if (need_clsf_flag == 1 && ll_ctrl_data.afh_mode == 1)
        {
            need_clsf_flag = 0;

            ptx->len = sizeof(ll_ctrl_data.chnl_map) + 1;
            ptx->cmd = LL_CHNL_CLASSIFICATION_IND;
            memcpy(ptx->data, &ll_ctrl_data.chnl_map[0], sizeof(ll_ctrl_data.chnl_map));
        }
        else
        {
            ptx->len = 29;
            ptx->cmd = LL_SYNC_DATA;
            memcpy(ptx->data, (const void *)sycn_ack_frame, ptx->len);
            ptx->data[0] = next_chnn_idx;
        }
    }
}

_attribute_ram_code_sec_noinline_ void proc_rx_data_2()
{
    ll_rx_packet_t *prx = (ll_rx_packet_t *)rx_payload;

#if DEBUG_LOG
    log_msg("rx_payload: ", rx_payload, 32);
#endif
    if (!(ll_ctrl_data.snnesn & LL_FLOW_RCVD))
        return;

    unsigned char cmd = prx->cmd;
    if (cmd == LL_ACCEPTED)
    {
        // unsigned char opcode = rx_payload[1];
    }
    else if (cmd == LL_CHG_CHNL_MAP_REQ)
    {
        memcpy((void *)&ll_ctrl_data.chnl_map[0], prx->data, sizeof(ll_ctrl_data.chnl_map));
        chn_table_calc(ll_ctrl_data.chnl_map, conn_req_pkt.hop);
        gpio_write(WHITE_LED_PIN, 1);
    }
    else if (cmd == LL_CHNL_CLASSIFICATION_REQ)
    {
        ll_ctrl_data.afh_mode = prx->data[0];
        ll_ctrl_data.afh_int_min_s = prx->data[1];
        ll_ctrl_data.afh_int_max_s = prx->data[2];

        if (ll_ctrl_data.afh_mode == 1)
        {
            ll_ctrl_data.clsf_tick = clock_time();
        }
    }
}

_attribute_ram_code_sec_noinline_ void build_ll_data_packet()
{
    ll_tx_packet_t *ptx = (ll_tx_packet_t *)tx_buffer;
    // ptx->type = ll_ctrl_data.snnesn << 2 & 0xc;
    ptx->len = 29;
    ptx->cmd = LL_SYNC_DATA;
    memcpy(ptx->data, sycn_ack_frame, ptx->len);
    ptx->data[0] = next_chnn_idx;
}

_attribute_ram_code_sec_noinline_ void app_low_energy_task(void)
{
    if (sync_flg == 0)
        return;

    if (tx_done_flag == 1)
    {
        tx_done_flag = 0;
        sync_tx_cnt++;
        gpio_toggle(DEBUG_PB7);
        if (sync_first_tx == 1)
        {
            sync_first_tx = 0;
            ll_snnesn_init();

            memcpy((const void *)&conn_req_pkt.opcode, (const void *)&rx_payload[0], sizeof(conn_req_pkt));
            memcpy(ll_ctrl_data.chnl_map, conn_req_pkt.chm, sizeof(ll_ctrl_data.chnl_map));
            sync_tx_interval_ms = conn_req_pkt.tx_int;

            gen_fsk_sync_word_len_set(SYNC_WORD_LEN_4BYTE);
            gen_fsk_sync_word_set(GEN_FSK_PIPE0, conn_req_pkt.ac);
            chn_table_calc(conn_req_pkt.chm, conn_req_pkt.hop);

            gpio_write(GREEN_LED_PIN, 0);

            next_chnn_idx = 0;
            rf_set_channel(ll_ctrl_data.chnl_tbl[next_chnn_idx], 0);
            // int marg = (sync_tx_interval_ms * 50) / 100;
            // gen_fsk_srx_start(clock_time() + marg * CLOCK_16M_SYS_TIMER_CLK_1MS, 0);
            gen_fsk_srx_start(clock_time(), 0);

            return;
        }

        proc_rx_data_2();

        if (expand_win_us >= expand_step_us)
            expand_win_us -= expand_step_us;

        if (sleep_cnt != 0)
            offset_tick = next_rx_point - rx_timestamp;

        next_rx_point = rx_timestamp + (sync_tx_interval_ms * CLOCK_16M_SYS_TIMER_CLK_1MS) - (64 * CLOCK_16M_SYS_TIMER_CLK_1US);
#if PM_32KRC_CALIBRATION
        pm_cal_32k_rc_offset(offset_tick);
#endif

#if (DBG_SUSPEND == 1)
        if (offset_tick < 0)
            offset_tick = -offset_tick;
            // it pays about 5 ms for debug printf | rx irq cnt | cmd | channel idx | channel map |
            // printf("%d. %1x, %1x, %d %d %d %d, %4x", rx_cnt, rx_payload[0], rx_payload[1], sync_channel_map[0],
            //       sync_channel_map[1], sync_channel_map[2], sync_channel_map[3], offset_tick);
#endif

#ifndef DEBUG_NO_SLEEP
        cpu_sleep_wakeup(SUSPEND_MODE, PM_WAKEUP_TIMER, next_rx_point - RX_MARGIN_PM_CALIB);
        rf_recovery_init();
#endif
        sleep_cnt++;

        next_chnn_idx = (next_chnn_idx + 1) % TOTAL_NB_CHNNEL;
        rf_set_channel(ll_ctrl_data.chnl_tbl[next_chnn_idx], 0);

#ifndef DEBUG_NO_SLEEP
        gen_fsk_srx_start(clock_time(), RX_NORMAL_WINDOW_US);
#else
        gen_fsk_srx_start(clock_time(), RX_TO_US);
#endif
        gpio_toggle(DEBUG_PB1);
    }

    if (1 == rx_flag)
    {
        gpio_toggle(GREEN_LED_PIN);

        gpio_toggle(DEBUG_PB6);
        sync_rx_cnt++;

        rx_flag = 0;
        rx_timestamp = gen_fsk_rx_timestamp_get((unsigned char *)rx_packet);
        rx_payload = gen_fsk_rx_payload_get((unsigned char *)rx_packet, (unsigned char *)&rx_payload_len);

        proc_rx_data_1();
        ll_tx_packet_t *ptx = (ll_tx_packet_t *)tx_buffer;
        ptx->type = ll_ctrl_data.snnesn << 2 & 0xc;

#if DEBUG_LOG
        log_msg("tx_payload: ", tx_buffer, 32);
#endif
        gen_fsk_stx_start(tx_buffer, clock_time());
    }

    if (1 == rx_first_timeout_flag)
    {
        rx_first_timeout_flag = 0;
        gpio_toggle(RED_LED_PIN);
        gpio_toggle(DEBUG_PB0);
        sync_first_timeout_cnt++;
        // WaitUs(500);

#if DEBUG_LOG
        log_msg("-------rx timeout----- \r\n", &sync_first_timeout_cnt, 1);
#endif

        ll_ctrl_data.con_tx_cnt++;

        if (ll_ctrl_data.con_tx_cnt >= AUTO_FLUSH_CNT_THRES) // auto flush occurs
        {
            ll_ctrl_data.con_tx_cnt = 0;
            build_ll_data_packet();
        }

        next_rx_point = next_rx_point + (sync_tx_interval_ms * CLOCK_16M_SYS_TIMER_CLK_1MS);

        if (expand_win_us < (sync_tx_interval_ms * CLOCK_16M_SYS_TIMER_CLK_1MS * 10 / 100))
        {
            expand_win_us += expand_step_us; //提前开窗
        }

#ifndef DEBUG_NO_SLEEP
        cpu_sleep_wakeup(SUSPEND_MODE, PM_WAKEUP_TIMER, next_rx_point - RX_MARGIN_PM_CALIB - expand_win_us);
        rf_recovery_init();
#endif

        next_chnn_idx = (next_chnn_idx + 1) % TOTAL_NB_CHNNEL;
        rf_set_channel(ll_ctrl_data.chnl_tbl[next_chnn_idx], 0);

#ifndef DEBUG_NO_SLEEP
        gen_fsk_srx_start(clock_time(), RX_LONG_WINDOW_US + ((expand_win_us / 16) * 2)); //两头扩窗
#else
        gen_fsk_srx_start(clock_time(), RX_TO_US - 100);
#endif

        gpio_toggle(DEBUG_PB2);
    }
}

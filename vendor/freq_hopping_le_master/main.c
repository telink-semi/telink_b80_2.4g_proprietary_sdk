#include "driver.h"
#include "genfsk_ll.h"
#include "common.h"

#define BLUE_LED_PIN GPIO_PB3
#define GREEN_LED_PIN GPIO_PB4
#define WHITE_LED_PIN GPIO_PB5
#define RED_LED_PIN GPIO_PB6
#define DEBUG_PIN GPIO_PB2
#define DBG_EXECUTE_TIME GPIO_PA6

#define RX_BUF_LEN 64
#define RX_BUF_NUM 4
#define APP_FIX_PAYLOAD_LEN 32
#define SYNC_FIX_PAYLOAD_LEN 1

#define MAX_CHANNEL_MAP_SIZE 4
#define SYNC_BROADCAST_INTERVAL (1250 * 16) // 1.25MS

volatile unsigned char timer0_expire_flg = 0;
unsigned int timer0_irq_cnt = 0;
#define REFRESH_CHNL_MAP_INT_S 5

#define AUTO_FLUSH_CNT_THRES 10

#define AFH_MODE_ENABLE 1
#define AFH_INT_MAX_S 8
#define AFH_INT_MIN_S 8

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

static unsigned char __attribute__((aligned(4))) tx_buffer[64] = {0};
volatile static unsigned char rx_buf[RX_BUF_LEN * RX_BUF_NUM] __attribute__((aligned(4))) = {0};
volatile static unsigned char rx_ptr = 0;
volatile static unsigned char rx_payload_len = 0;
volatile static unsigned char *rx_payload = 0;
volatile static unsigned char *rx_packet = 0;
volatile static unsigned char rx_flag, tx_done_flag, rx_timeout_flag = 0;

unsigned char broadcast_chnn_idx = 0;
unsigned char broadcast_channel_map[3] = {8, 18, 28};
unsigned int sync_tx_cnt, async_rx_cnt = 0, sync_rx_cnt = 0, async_rxtimeout_cnt = 0;
unsigned char need_afh_flag = 0;

static unsigned char rx_crc_err_flag = 0;
static unsigned int rx_crc_err_cnt = 0;
int m_bad_chnl_times = 0;
int slave_chnl_chg_times = 0;
unsigned char afh_bad_chnl = 100;
unsigned int sync_rx_timeout_cnt = 0;
unsigned int chg_chnl_map_times = 0;

#define CLOCK_SYS_CLOCK_HZ 24000000
enum
{
    CLOCK_SYS_CLOCK_1S = CLOCK_SYS_CLOCK_HZ,
    CLOCK_SYS_CLOCK_1MS = (CLOCK_SYS_CLOCK_1S / 1000),
    CLOCK_SYS_CLOCK_1US = (CLOCK_SYS_CLOCK_1S / 1000000),
};

#define SYNC_TX_INT_MS 500

typedef struct
{
    u8 opcode;
    u8 ac[4];
    u16 tx_int;
    u8 chm[10];
    u8 hop;
} rf_pkt_conn_req_t;

#define AFH_HOP 2
#define TOTAL_NB_CHNNEL 79
#define CHNL_MAP_LEN 10
#define AFH_NB_CHANNEL_MAX 16
#define AFH_NB_CHANNEL_MIN 8
#define AFH_NB_CHANNEL_ADB (AFH_NB_CHANNEL_MAX - AFH_NB_CHANNEL_MIN + 8)

#define AFH_BAD_CHNL_QLTY_THRES -3
#define AFH_GOOD_CHNL_QLTY_THRES 3

typedef struct ll_comm_ctrl_data
{
    /// 10-bytes channel map array
    uint8_t chnl_map[CHNL_MAP_LEN];
    uint8_t chnl_tbl[TOTAL_NB_CHNNEL];
    int8_t qlty[TOTAL_NB_CHNNEL];
    char chnl_abd[AFH_NB_CHANNEL_ADB];
    char abd_head; // chnl_abd is FIFO, head and tail is FIFO ptr
    char abd_tail;
    unsigned char chnl_used;
    u8 snnesn;
    u8 con_tx_cnt;
} ll_comm_ctrl_data_t;
ll_comm_ctrl_data_t ll_ctrl_data;

unsigned char conn_sync_word[4] = {0x52, 0x56, 0x78, 0x53};
unsigned char channel_map[10] = {0x10, 0x10, 0x11, 0x10, 0x11, 0x11, 0x11, 0x11, 0x11, 0x01};

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
    u8 data[64 - 7]; // 42-7
} __attribute__((packed)) __attribute__((aligned(4))) ll_rx_packet_t;

typedef struct sync_ll_tx_packet
{
    int dma_len;
    u8 type;
    u8 len;
    u8 cmd;
    u8 data[64 - 7];
} __attribute__((packed)) __attribute__((aligned(4))) ll_tx_packet_t;

#define AUTO_FLUSH_TIMER_MS (SYNC_TX_INT_MS * 3)

unsigned char sync_chnn_idx = 0;
unsigned int sync_anchor_point = 0;
volatile unsigned char sync_flg = 0;
unsigned char sycn_tx_frame[APP_FIX_PAYLOAD_LEN] = {
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10,
    0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20,
};

_attribute_ram_code_sec_noinline_ __attribute__((optimize("-Os"))) void irq_handler(
    void)
{
    unsigned int irq_src = irq_get_src();
    unsigned short rf_irq_src = rf_irq_src_get();

    if (timer_get_interrupt_status(FLD_TMR_STA_TMR0))
    {
        timer_clear_interrupt_status(FLD_TMR_STA_TMR0);
        timer0_irq_cnt++;
        timer0_expire_flg = 1;
    }

    if (irq_src & FLD_IRQ_ZB_RT_EN) // rf irq occurs
    {
        if (rf_irq_src & FLD_RF_IRQ_TX) // rf tx irq occurs
        {
            reg_rf_irq_status = FLD_RF_IRQ_TX;

            tx_done_flag = 1;
        }

        if (rf_irq_src & FLD_RF_IRQ_RX) // rf tx irq occurs
        {
            reg_rf_irq_status = FLD_RF_IRQ_RX;

            rx_packet = rx_buf + rx_ptr * RX_BUF_LEN;

            rx_ptr = (rx_ptr + 1) % RX_BUF_NUM;

            gen_fsk_rx_buffer_set(
                (unsigned char *)(rx_buf + rx_ptr * RX_BUF_LEN),
                RX_BUF_LEN);

            if (gen_fsk_is_rx_crc_ok((unsigned char *)rx_packet))
            {
                if (sync_flg == 1 && ll_ctrl_data.qlty[ll_ctrl_data.chnl_tbl[sync_chnn_idx]] < AFH_GOOD_CHNL_QLTY_THRES)
                {

                    ll_ctrl_data.qlty[ll_ctrl_data.chnl_tbl[sync_chnn_idx]]++;
                }
            }
            else
            {
                rx_crc_err_cnt++;
                rx_crc_err_flag = 1;
                if (sync_flg == 1 && ll_ctrl_data.qlty[ll_ctrl_data.chnl_tbl[sync_chnn_idx]] > AFH_BAD_CHNL_QLTY_THRES)
                    ll_ctrl_data.qlty[ll_ctrl_data.chnl_tbl[sync_chnn_idx]]--;
            }
#if 0 // for debug
            m_bad_chnl_times++;
            if (m_bad_chnl_times > 30)
            {
                m_bad_chnl_times = 0;
                ll_ctrl_data.qlty[ll_ctrl_data.chnl_tbl[sync_chnn_idx]] = -4;
                afh_bad_chnl = ll_ctrl_data.chnl_tbl[sync_chnn_idx];
            }
#endif
            rx_flag = 1;
        }

        if (rf_irq_src & FLD_RF_IRQ_RX_TIMEOUT) // if rf tx irq occurs
        {
            reg_rf_irq_status = FLD_RF_IRQ_RX_TIMEOUT;

            rx_timeout_flag = 1;
        }
    }

    irq_clr_src2(FLD_IRQ_ALL);
}

void user_init(void)
{
    // io init
    unsigned int pin = WHITE_LED_PIN | GREEN_LED_PIN | RED_LED_PIN | BLUE_LED_PIN;
    gpio_set_func(pin, AS_GPIO);
    gpio_set_input_en(pin, 0);  // disable output
    gpio_set_output_en(pin, 1); // enable output
    gpio_write(pin, 0);

    pin = DEBUG_PIN | DBG_EXECUTE_TIME;
    gpio_set_func(pin, AS_GPIO);
    gpio_set_input_en(pin, 0);  // disable output
    gpio_set_output_en(pin, 1); // enable output
    gpio_write(pin, 0);

    timer0_set_mode(TIMER_MODE_SYSCLK, 0, REFRESH_CHNL_MAP_INT_S * CLOCK_SYS_CLOCK_1S);

    memset((void *)&ll_ctrl_data, 0, sizeof(ll_ctrl_data));
    ll_ctrl_data.abd_head = 0;
    ll_ctrl_data.abd_tail = 0;

    // rf init
    unsigned char sync_word[4] = {0x53, 0x78, 0x56, 0x52};
    // it needs to notice that this api is different from vulture / kite
    gen_fsk_datarate_set(GEN_FSK_DATARATE_1MBPS); // Note that this API must be invoked first before all other APIs
    gen_fsk_preamble_len_set(4);
    gen_fsk_sync_word_len_set(SYNC_WORD_LEN_4BYTE);
    gen_fsk_sync_word_set(GEN_FSK_PIPE0, sync_word); // set pipe0's sync word
    gen_fsk_pipe_open(GEN_FSK_PIPE0);                // enable pipe0's reception
    gen_fsk_tx_pipe_set(GEN_FSK_PIPE0);              // set pipe0 as the TX pipe
    gen_fsk_packet_format_set(GEN_FSK_PACKET_FORMAT_FIXED_PAYLOAD, SYNC_FIX_PAYLOAD_LEN);
    gen_fsk_radio_power_set(GEN_FSK_RADIO_POWER_0DBM);
    gen_fsk_rx_buffer_set((unsigned char *)(rx_buf + rx_ptr * RX_BUF_LEN), RX_BUF_LEN);
    gen_fsk_channel_set(6);
    gen_fsk_radio_state_set(GEN_FSK_STATE_AUTO); // set transceiver to basic TX state
    gen_fsk_tx_settle_set(149);
    gen_fsk_rx_settle_set(89);

    rf_irq_enable(FLD_RF_IRQ_TX | FLD_RF_IRQ_RX | FLD_RF_IRQ_RX_TIMEOUT); // enable rf tx irq
    irq_enable_type(FLD_IRQ_ZB_RT_EN);                                    // enable RF irq
    irq_enable();                                                         // enable general irq
}

_attribute_ram_code_sec_noinline_ void proc_user_task()
{
    if (timer0_expire_flg == 1)
    {
        timer0_expire_flg = 0;
        refresh_chnl_map();
    }
}

void app_sync_init(void);

void app_sync_task(void);

void app_cycle_send_task(void);

_attribute_ram_code_sec_noinline_ int main(void)
{
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    clock_init(SYS_CLK_24M_Crystal);

    user_init();

    app_sync_init();

    while (1)
    {

        app_sync_task();

        app_cycle_send_task();

        gpio_toggle(DBG_EXECUTE_TIME);
    }
}

_attribute_ram_code_sec_noinline_ void app_sync_init(void)
{
    tx_buffer[0] = SYNC_FIX_PAYLOAD_LEN;
    tx_buffer[1] = 0x00;
    tx_buffer[2] = 0x00;
    tx_buffer[3] = 0x00;
    tx_buffer[4] = LL_SYNC_REQ;

    rf_set_channel(broadcast_channel_map[0], 0);
    sync_anchor_point = clock_time() + SYNC_BROADCAST_INTERVAL;
    gen_fsk_stx2rx_start(tx_buffer, sync_anchor_point, 350);
}

_attribute_ram_code_sec_noinline_ static void led_blink(unsigned char cnt)
{
    while (cnt--)
    {
        gpio_toggle(GREEN_LED_PIN | WHITE_LED_PIN | RED_LED_PIN);
        WaitMs(50);
    }
    gpio_write(GREEN_LED_PIN | WHITE_LED_PIN | RED_LED_PIN | BLUE_LED_PIN, 0);
}

_attribute_ram_code_sec_noinline_ void refresh_chnl_map()
{
    for (int i = 0; i < TOTAL_NB_CHNNEL; i++)
    {
        if (ll_ctrl_data.qlty[i] <= AFH_BAD_CHNL_QLTY_THRES)
        {
            if (!BIT_IS_SET(ll_ctrl_data.chnl_map[i / 8], i % 8))
                continue;
            need_afh_flag = 1;

            if ((ll_ctrl_data.abd_tail + 1) % AFH_NB_CHANNEL_ADB == ll_ctrl_data.abd_head)
            {
                printf("chnl_abd fifo is full");
            }
            else
            {
                //只有当channel被放到abd数组才清chnl_map,防止这个channel丢失
                BIT_CLR(ll_ctrl_data.chnl_map[i / 8], i % 8);
                ll_ctrl_data.qlty[i] = 0;
                ll_ctrl_data.chnl_used--;

                ll_ctrl_data.chnl_abd[ll_ctrl_data.abd_tail++] = i;
                ll_ctrl_data.abd_tail %= AFH_NB_CHANNEL_ADB;
            }
        }
    }

    if (need_afh_flag == 2) // slave have channel map changed
        need_afh_flag = 1;

    while (ll_ctrl_data.chnl_used < AFH_NB_CHANNEL_MIN) //最少需要维持AFH_NB_CHANNEL_MIN (8)个可用channel
    {
        if (ll_ctrl_data.abd_head == ll_ctrl_data.abd_tail)
        {
            printf("chnl_abd fifo is empty");
            break;
        }
        else
        {
            unsigned char idex = ll_ctrl_data.chnl_abd[ll_ctrl_data.abd_head++];
            BIT_SET(ll_ctrl_data.chnl_map[idex / 8], idex % 8);
            ll_ctrl_data.abd_head %= AFH_NB_CHANNEL_ADB;
            ll_ctrl_data.chnl_used++;
        }
    }
}

_attribute_ram_code_sec_noinline_ void refresh_chnl_abd()
{
    unsigned char chnl_map_new[CHNL_MAP_LEN] = {0};
    ll_rx_packet_t *prx = (ll_rx_packet_t *)rx_payload;
    memcpy(chnl_map_new, prx->data, CHNL_MAP_LEN);
    for (int i = 0; i < CHNL_MAP_LEN; i++)
    {
        unsigned char v = ll_ctrl_data.chnl_map[i] ^ chnl_map_new[i];
        if (v)
        {
            for (int j = 0; j < 8; j++)
            {
                if (BIT_IS_SET(v, j))
                {
                    slave_chnl_chg_times++;
                    need_afh_flag = 2;                           //立即执行LL_CHG_CHNL_MAP_REQ(1)或等到下一次TIMER0到期(2)
                    ll_ctrl_data.chnl_map[i] &= chnl_map_new[i]; // 清空对应的bit
                    ll_ctrl_data.chnl_used--;

                    if ((ll_ctrl_data.abd_tail + 1) % AFH_NB_CHANNEL_ADB == ll_ctrl_data.abd_head)
                    {
                        printf("chnl_abd fifo is full");
                    }
                    else
                    {
                        ll_ctrl_data.chnl_abd[ll_ctrl_data.abd_tail++] = i * 8 + j;
                        ll_ctrl_data.abd_tail %= AFH_NB_CHANNEL_ADB;
                    }
                }
            }
        }
    }
}

_attribute_ram_code_sec_noinline_ int chn_table_calc(u8 *chn_map, u8 hop)
{
    u8 k = 0, numused = 0;
    u8 tmp_table[TOTAL_NB_CHNNEL];

    for (int i = 0; i < TOTAL_NB_CHNNEL; i++)
    {

        if (chn_map[i >> 3] & BIT(i & 0x07)) //取出chnl map中的可用chnl，chnl_map中一个字节对应8个chnl
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
            ll_ctrl_data.chnl_tbl[ll] = tmp_table[m]; //将channel放入chnl_table
        }
        ll++;
    }
    return numused;
}

_attribute_ram_code_sec_noinline_ void build_chnl_clsf_req()
{
    ll_tx_packet_t *ptx = (ll_tx_packet_t *)tx_buffer;
    ptx->type = ll_ctrl_data.snnesn << 2 & 0xc;
    ptx->len = 3 + 1;
    ptx->cmd = LL_CHNL_CLASSIFICATION_REQ;
    ptx->data[0] = AFH_MODE_ENABLE;
    ptx->data[1] = AFH_INT_MIN_S;
    ptx->data[2] = AFH_INT_MAX_S;
}

_attribute_ram_code_sec_noinline_ void build_chg_chnl_map_req()
{
    ll_tx_packet_t *ptx = (ll_tx_packet_t *)tx_buffer;
    chg_chnl_map_times++;
    ptx->type = ll_ctrl_data.snnesn << 2 & 0xc;
    ptx->len = sizeof(ll_ctrl_data.chnl_map) + 1;
    ptx->cmd = LL_CHG_CHNL_MAP_REQ;
    memcpy(ptx->data, (unsigned char *)&ll_ctrl_data.chnl_map[0], sizeof(ll_ctrl_data.chnl_map));
}

_attribute_ram_code_sec_noinline_ void build_ll_data_packet()
{
    ll_tx_packet_t *ptx = (ll_tx_packet_t *)tx_buffer;
    ptx->type = ll_ctrl_data.snnesn << 2 & 0xc;
    ptx->len = 29;
    ptx->cmd = LL_SYNC_DATA;
    memcpy(ptx->data, sycn_tx_frame, ptx->len);
    ptx->data[0] = sync_chnn_idx;
}

_attribute_ram_code_sec_noinline_ void ll_snnesn_init()
{
    ll_ctrl_data.snnesn = 0; // for master
}

_attribute_ram_code_sec_noinline_ u8 ll_flow_process(u8 peer, u8 local)
{
    local &= ~LL_FLOW_RCVD;
    local &= ~LL_FLOW_SENT;

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
        rx_crc_err_flag = 0;

    if (peer_nesn != local_sn)
    {
        local = (local & ~LL_FLOW_SN) | (peer_nesn << 1);
        local |= LL_FLOW_SENT;
    }
    else
    {
        ll_ctrl_data.con_tx_cnt++;
    }

    return local;
}

_attribute_ram_code_sec_noinline_ void app_sync_task(void)
{
    if (sync_flg != 0) // already sync
        return;

    if (1 == tx_done_flag)
    {
        tx_done_flag = 0;
        sync_tx_cnt++;
        tx_buffer[5] = sync_chnn_idx;
    }

    if (1 == rx_flag)
    {
        rx_flag = 0;
        async_rx_cnt++;
        gpio_toggle(GREEN_LED_PIN);
        rx_payload = gen_fsk_rx_payload_get(rx_packet, &rx_payload_len);
        if (rx_payload[0] == LL_SYNC_RSP)
        {
            gen_fsk_packet_format_set(GEN_FSK_PACKET_FORMAT_FIXED_PAYLOAD, APP_FIX_PAYLOAD_LEN);
            tx_buffer[0] = APP_FIX_PAYLOAD_LEN;

            broadcast_chnn_idx = (broadcast_chnn_idx + 1) % 3;
            rf_set_channel(broadcast_channel_map[broadcast_chnn_idx], 0);
            sync_anchor_point += SYNC_BROADCAST_INTERVAL;

            ll_ctrl_data.chnl_used = AFH_NB_CHANNEL_MAX;
            memcpy(ll_ctrl_data.chnl_map, channel_map, sizeof(channel_map));

            rf_pkt_conn_req_t *conn_req_pkt = (rf_pkt_conn_req_t *)&tx_buffer[4];
            conn_req_pkt->opcode = LL_CONN_REQ;
            memcpy((unsigned char *)&(conn_req_pkt->ac[0]), conn_sync_word, sizeof(conn_sync_word));
            conn_req_pkt->tx_int = SYNC_TX_INT_MS;
            memcpy((unsigned char *)&(conn_req_pkt->chm[0]), channel_map, sizeof(channel_map));
            conn_req_pkt->hop = AFH_HOP;
            chn_table_calc(channel_map, conn_req_pkt->hop); //提前算出channel table,但是并没有启用

            gen_fsk_stx2rx_start(tx_buffer, sync_anchor_point, 350);

            return;
        }
        else if (rx_payload[0] == LL_CONN_RSP)
        {
            sync_flg = 1;
            ll_snnesn_init();

            build_chnl_clsf_req();
            gen_fsk_sync_word_len_set(SYNC_WORD_LEN_4BYTE);
            gen_fsk_sync_word_set(GEN_FSK_PIPE0, conn_sync_word); // set pipe0's sync word
            sync_chnn_idx = 0;
            rf_set_channel(ll_ctrl_data.chnl_tbl[sync_chnn_idx], 0);
            sync_anchor_point += (SYNC_TX_INT_MS * CLOCK_16M_SYS_TIMER_CLK_1MS);

            gen_fsk_stx2rx_start(tx_buffer, sync_anchor_point, 350);
            timer_start(TIMER0); // start channel classfication
            led_blink(5);        // indicate sync success
        }
    }

    if (1 == rx_timeout_flag)
    {
        rx_timeout_flag = 0;
        async_rxtimeout_cnt++;
        if (async_rxtimeout_cnt % 500 == 0)
            gpio_toggle(WHITE_LED_PIN);

        broadcast_chnn_idx = (broadcast_chnn_idx + 1) % 3;
        rf_set_channel(broadcast_channel_map[broadcast_chnn_idx], 0);
        sync_anchor_point += SYNC_BROADCAST_INTERVAL;
        gen_fsk_stx2rx_start(tx_buffer, sync_anchor_point, 350);
    }
}

_attribute_ram_code_sec_noinline_ void proc_rx_data()
{
    ll_rx_packet_t *prx = (ll_rx_packet_t *)rx_payload;
    u8 peer = prx->type >> 2 & 3;
    ll_ctrl_data.snnesn = ll_flow_process(peer, ll_ctrl_data.snnesn);

    if (!(ll_ctrl_data.snnesn & LL_FLOW_RCVD))
    {
        return;
    }

    unsigned char cmd = prx->cmd;
    if (cmd == LL_ACCEPTED)
    {
        unsigned char opcode = prx->data[0];
        if (opcode == LL_CHG_CHNL_MAP_REQ)
        {
            chn_table_calc(ll_ctrl_data.chnl_map, AFH_HOP);
        }
    }
    else if (cmd == LL_CHNL_CLASSIFICATION_IND)
    {
        refresh_chnl_abd();
    }
}

_attribute_ram_code_sec_noinline_ void proc_tx_data()
{
    if (need_afh_flag == 1)
    {
        build_chg_chnl_map_req();
    }
    else
    {
        build_ll_data_packet();
    }
}

_attribute_ram_code_sec_noinline_ void app_cycle_send_task(void)
{
    if (sync_flg == 0)
        return;

    if (1 == tx_done_flag)
    {
        gpio_write(DEBUG_PIN, 0);
        tx_done_flag = 0;
        sync_tx_cnt++;
    }

    if (1 == rx_flag)
    {
        rx_flag = 0;
        sync_rx_cnt++;
#if 1
        gpio_toggle(GREEN_LED_PIN);

        rx_payload = gen_fsk_rx_payload_get(rx_packet, &rx_payload_len);
        proc_rx_data();

        if (ll_ctrl_data.snnesn & LL_FLOW_SENT ||
            ll_ctrl_data.con_tx_cnt >= AUTO_FLUSH_CNT_THRES)
        {
            ll_ctrl_data.con_tx_cnt = 0;
            proc_tx_data();
        }
        ll_tx_packet_t *ptx = (ll_tx_packet_t *)tx_buffer;
        ptx->type = ll_ctrl_data.snnesn << 2 & 0xc;

        sync_chnn_idx = (sync_chnn_idx + 1) % TOTAL_NB_CHNNEL;
        rf_set_channel(ll_ctrl_data.chnl_tbl[sync_chnn_idx], 0);
        sync_anchor_point += (SYNC_TX_INT_MS * CLOCK_16M_SYS_TIMER_CLK_1MS);

        gen_fsk_stx2rx_start(tx_buffer, sync_anchor_point, 350);
        gpio_write(DEBUG_PIN, 1);
#endif
    }
    if (1 == rx_timeout_flag)
    {

#if 1
        sync_rx_timeout_cnt++;
        rx_timeout_flag = 0;
        // gpio_toggle(DEBUG_PIN);
        gpio_toggle(RED_LED_PIN);

        ll_ctrl_data.con_tx_cnt++;

        if (ll_ctrl_data.con_tx_cnt >= AUTO_FLUSH_CNT_THRES) // auto flush occurs
        {
            ll_ctrl_data.con_tx_cnt = 0;
            build_ll_data_packet();
        }

        sync_chnn_idx = (sync_chnn_idx + 1) % TOTAL_NB_CHNNEL;
        if (need_afh_flag != 1)
            tx_buffer[7] = sync_chnn_idx;
        rf_set_channel(ll_ctrl_data.chnl_tbl[sync_chnn_idx], 0);
        sync_anchor_point += (SYNC_TX_INT_MS * CLOCK_16M_SYS_TIMER_CLK_1MS);
        gen_fsk_stx2rx_start(tx_buffer, sync_anchor_point, 350);
        gpio_write(DEBUG_PIN, 1);
#endif
    }
}

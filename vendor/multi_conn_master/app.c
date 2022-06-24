#include "driver.h"
#include "genfsk_ll.h"
#include "common.h"

#define DEBUG_LOG 0
#define DEVICE_LOG_NUM 0

#define BLUE_LED_PIN GPIO_PB3
#define GREEN_LED_PIN GPIO_PB4
#define WHITE_LED_PIN GPIO_PB5
#define RED_LED_PIN GPIO_PB6

#define DEBUG_PA6 GPIO_PA6
#define DEBUG_PA7 GPIO_PA7
#define DEBUG_PB0 GPIO_PB0
#define DEBUG_PB1 GPIO_PB1
#define DEBUG_PB2 GPIO_PB2

#define DEBUG_PB7 GPIO_PB7
#define DEBUG_PC0 GPIO_PC0
#define DEBUG_PC1 GPIO_PC1

#define RX_BUF_LEN 64
#define RX_BUF_NUM 4
#define APP_FIX_PAYLOAD_LEN 32
#define SYNC_FIX_PAYLOAD_LEN 1

#define MAX_CHANNEL_MAP_SIZE 4
#define SYNC_BROADCAST_INTERVAL 20000 //(1250 * 16) // 1.25MS

volatile unsigned char timer0_expire_flg = 0;
unsigned int timer0_irq_cnt = 0;
#define REFRESH_CHNL_MAP_INT_S 5

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
unsigned int sync_tx_cnt, async_tx_cnt = 0, async_rx_cnt = 0, sync_rx_cnt = 0, async_rxtimeout_cnt = 0;

int m_bad_chnl_times = 0;
int slave_chnl_chg_times = 0;
unsigned char afh_bad_chnl = 100;
unsigned int sync_rx_timeout_cnt = 0;

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
    u16 ts_time; // n * 1.25ms
    u16 tx_int;
    u8 chm[10];
    u8 hop;
} __attribute__((packed)) __attribute__((aligned(4))) rf_pkt_conn_req_t;

#define AC_LEN 4
unsigned char conn_sync_word[3][4] = {{0x29, 0x38, 0x47, 0x56},
                                      {0x86, 0xa8, 0xbc, 0x39},
                                      {0x93, 0xbe, 0x69, 0x5a}};
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

#define AFH_HOP 2
#define TOTAL_NB_CHNNEL 79
#define CHNL_MAP_LEN 10
#define AFH_NB_CHANNEL_MAX 16
#define AFH_NB_CHANNEL_MIN 8
#define AFH_NB_CHANNEL_ADB (AFH_NB_CHANNEL_MAX - AFH_NB_CHANNEL_MIN + 8)

#define AFH_BAD_CHNL_QLTY_THRES -3
#define AFH_GOOD_CHNL_QLTY_THRES 3

#define MAX_CONN_NUM 3
#define MAC_LEN 6

typedef struct ll_conn_ctrl_data
{
    /// 10-bytes channel map array
    u8 chnl_map[CHNL_MAP_LEN];
    u8 chnl_tbl[TOTAL_NB_CHNNEL];
    s8 qlty[TOTAL_NB_CHNNEL];
    u8 chnl_abd[AFH_NB_CHANNEL_ADB];
    u8 abd_head; // chnl_abd is FIFO, head and tail is FIFO ptr
    u8 abd_tail;
    u8 chnl_used;
    u8 snnesn;
    u8 con_tx_cnt;
    u8 need_afh_flag;
    u8 rx_crc_err_flag;
    u32 clsf_tick;
    u8 sync_chnn_idx;
    u8 is_first_tx;
    u8 dev_id;
    u8 mac[MAC_LEN];
    u8 rx_to_flag;
    u32 rx_to_cnt;
    u32 fno;
    ll_tx_packet_t ptx_buff; // previous time tx data,access with memcpy(memory align)
    u32 chg_chnl_map_times;
    u32 rx_crc_err_cnt;
} ll_conn_data_t;
ll_conn_data_t ll_conn_data[MAX_CONN_NUM];

#define AUTO_FLUSH_TIMER_MS (SYNC_TX_INT_MS * 3)
#define AUTO_FLUSH_CNT_THRES 10

unsigned char sync_chnn_idx = 0;
unsigned int async_anchor_point = 0;
unsigned int sync_anchor_point = 0;

// unsigned int temp_anchor = 0;
// unsigned int temp_anchor_tick = 0;
// unsigned int temp_anchor_tick_d = 0;
// unsigned int temp_tick_ms = 0;

volatile unsigned char sync_flg = 0;
unsigned char sycn_tx_frame[APP_FIX_PAYLOAD_LEN] = {
    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10,
    0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20,
};

#define B_AC 0X38958D75
u32 systimer_irq_cnt = 0;
char conn_dev_num = 0;
u8 dev_id = 0;

typedef struct ll_comm_ctrl_data
{
    u8 state;
    u32 brct_tick;
} ll_ctrl_data_t;
ll_ctrl_data_t ll_ctrl_data;
enum
{
    LL_STATE_INIT_BRCT,
    LL_STATE_BRCT,
    LL_STATE_CONN,

};

void set_conn_rf_len();
void system_timer_irq_func();
void timer0_irq_func();
void refresh_chnl_map();

_attribute_ram_code_sec_noinline_ __attribute__((optimize("-Os"))) void irq_handler(void)
{
    unsigned int irq_src = irq_get_src();
    unsigned short rf_irq_src = rf_irq_src_get();

    if (irq_src & FLD_IRQ_SYSTEM_TIMER)
    {
        if (conn_dev_num > 1)
        {
            timer_start(TIMER0);
        }
        gpio_toggle(DEBUG_PA6);
        reg_system_tick_irq_level = clock_time() ^ BIT(31);
        reg_irq_src = FLD_IRQ_SYSTEM_TIMER;

        system_timer_irq_func();

        systimer_irq_cnt++;
        reg_system_tick_irq_level = sync_anchor_point;
    }

#if 1
    if (timer_get_interrupt_status(FLD_TMR_STA_TMR0))
    {
        timer_clear_interrupt_status(FLD_TMR_STA_TMR0);
        gpio_toggle(DEBUG_PB7);
        timer0_irq_cnt++;

        timer0_irq_func();
        if (dev_id == conn_dev_num - 1)
            timer_stop(TIMER0);
    }
#endif

#if 1
    if (irq_src & FLD_IRQ_ZB_RT_EN) // rf irq occurs
    {
        if (rf_irq_src & FLD_RF_IRQ_TX) // rf tx irq occurs
        {
            reg_rf_irq_status = FLD_RF_IRQ_TX;

            tx_done_flag = 1;
        }

        if (rf_irq_src & FLD_RF_IRQ_RX) // rf rx irq occurs
        {
            reg_rf_irq_status = FLD_RF_IRQ_RX;

            rx_packet = rx_buf + rx_ptr * RX_BUF_LEN;

            rx_ptr = (rx_ptr + 1) % RX_BUF_NUM;

            gen_fsk_rx_buffer_set(
                (unsigned char *)(rx_buf + rx_ptr * RX_BUF_LEN),
                RX_BUF_LEN);

            if (gen_fsk_is_rx_crc_ok((unsigned char *)rx_packet))
            {
                if (sync_flg == 1 && ll_conn_data[dev_id].qlty[ll_conn_data[dev_id].chnl_tbl[ll_conn_data[dev_id].sync_chnn_idx]] < AFH_GOOD_CHNL_QLTY_THRES)
                {

                    ll_conn_data[dev_id].qlty[ll_conn_data[dev_id].chnl_tbl[ll_conn_data[dev_id].sync_chnn_idx]]++;
                }
            }
            else
            {
                ll_conn_data[dev_id].rx_crc_err_cnt++;

                ll_conn_data[dev_id].rx_crc_err_flag = 1;
                if (sync_flg == 1 && ll_conn_data[dev_id].qlty[ll_conn_data[dev_id].chnl_tbl[ll_conn_data[dev_id].sync_chnn_idx]] > AFH_BAD_CHNL_QLTY_THRES)
                    ll_conn_data[dev_id].qlty[ll_conn_data[dev_id].chnl_tbl[ll_conn_data[dev_id].sync_chnn_idx]]--;
            }

            rx_flag = 1;
        }

        if (rf_irq_src & FLD_RF_IRQ_RX_TIMEOUT) // if rf tx irq occurs
        {
            reg_rf_irq_status = FLD_RF_IRQ_RX_TIMEOUT;

            rx_timeout_flag = 1;
        }
        reg_irq_src = FLD_IRQ_ZB_RT_EN;
    }

#endif
}

void user_init(void)
{
    // io init
    unsigned int pin = WHITE_LED_PIN | GREEN_LED_PIN | RED_LED_PIN | BLUE_LED_PIN;
    gpio_set_func(pin, AS_GPIO);
    gpio_set_input_en(pin, 0);
    gpio_set_output_en(pin, 1);
    gpio_write(pin, 0);

    pin = DEBUG_PA6 | DEBUG_PA7;
    gpio_set_func(pin, AS_GPIO);
    gpio_set_input_en(pin, 0);
    gpio_set_output_en(pin, 1);
    gpio_write(pin, 0);

    pin = DEBUG_PB0 | DEBUG_PB1 | DEBUG_PB2 | DEBUG_PB7;
    gpio_set_func(pin, AS_GPIO);
    gpio_set_input_en(pin, 0);
    gpio_set_output_en(pin, 1);
    gpio_write(pin, 0);

    pin = DEBUG_PC0 | DEBUG_PC1;
    gpio_set_func(pin, AS_GPIO);
    gpio_set_input_en(pin, 0);
    gpio_set_output_en(pin, 1);
    gpio_write(pin, 0);

    timer0_set_mode(TIMER_MODE_SYSCLK, 0, CLOCK_SYS_CLOCK_1US * 1250 * 10);

    memset((void *)&ll_conn_data, 0, sizeof(ll_conn_data));
    memset((void *)&ll_ctrl_data, 0, sizeof(ll_ctrl_data));
    ll_ctrl_data.state = LL_STATE_INIT_BRCT;

    // rf init
    unsigned char sync_word[4] = {B_AC & 0XFF, B_AC >> 8 & 0XFF, B_AC >> 16 & 0XFF, B_AC >> 24 & 0XFF};
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
    gen_fsk_channel_set(60);
    gen_fsk_radio_state_set(GEN_FSK_STATE_AUTO); // set transceiver to basic TX state
    gen_fsk_tx_settle_set(149);
    gen_fsk_rx_settle_set(89);

    rf_irq_enable(FLD_RF_IRQ_TX | FLD_RF_IRQ_RX | FLD_RF_IRQ_RX_TIMEOUT); // enable rf tx irq
    irq_enable_type(FLD_IRQ_ZB_RT_EN);                                    // enable RF irq
    irq_enable();                                                         // enable general irq

    reg_system_irq_mask &= ~BIT(2); // disable stimer
#if DEBUG_LOG
    usb_set_pin_en();
    usb_loginit();
    WaitMs(2000); // delay to ensure USB enumerate done
#endif
}

_attribute_ram_code_sec_noinline_ void proc_user_task()
{
    if (ll_ctrl_data.state == LL_STATE_BRCT &&
        clock_time_exceed(ll_ctrl_data.brct_tick, (SYNC_TX_INT_MS - 100) * 1000))
    {
        u32 to_tick = clock_time();
        while (1)
        {
            if (clock_time_exceed(to_tick, SYNC_BROADCAST_INTERVAL >> 3))
            {
                break;
            }

            if (rx_timeout_flag == 1)
                break;
        }
        rx_timeout_flag = 0;
        tx_done_flag = 0; // clear broadcast status, when the last broadcast packet rsp by slave? fixme???

        gpio_write(DEBUG_PA7, 0);

        set_conn_rf_len();
        ll_ctrl_data.state = LL_STATE_CONN;
    }
#if 1
    if (conn_dev_num > 0 && clock_time_exceed(ll_conn_data[dev_id].clsf_tick, REFRESH_CHNL_MAP_INT_S * 1000 * 1000))
    {
        ll_conn_data[dev_id].clsf_tick = clock_time();
        refresh_chnl_map();
    }
#endif
}

_attribute_ram_code_sec_noinline_ void app_sync_init(void)
{
    tx_buffer[0] = SYNC_FIX_PAYLOAD_LEN;
    tx_buffer[1] = 0x00;
    tx_buffer[2] = 0x00;
    tx_buffer[3] = 0x00;
    tx_buffer[4] = LL_SYNC_REQ;

    rf_set_channel(broadcast_channel_map[0], 0);
    async_anchor_point = clock_time() + SYNC_BROADCAST_INTERVAL;
    gen_fsk_stx2rx_start(tx_buffer, async_anchor_point, 350);
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

_attribute_ram_code_sec_noinline_ void set_pipe0_ac()
{
    gen_fsk_sync_word_len_set(SYNC_WORD_LEN_4BYTE);
    unsigned char conn_ac[4] = {0};
    memcpy(conn_ac, conn_sync_word[dev_id], AC_LEN);
    gen_fsk_sync_word_set(GEN_FSK_PIPE0, conn_ac); // set pipe0's sync word
}

_attribute_ram_code_sec_noinline_ void start_brct()
{
    unsigned char sync_word[4] = {B_AC & 0XFF, B_AC >> 8 & 0XFF, B_AC >> 16 & 0XFF, B_AC >> 24 & 0XFF};
    gen_fsk_sync_word_len_set(SYNC_WORD_LEN_4BYTE);
    gen_fsk_sync_word_set(GEN_FSK_PIPE0, sync_word); // set pipe0's sync word
    gen_fsk_packet_format_set(GEN_FSK_PACKET_FORMAT_FIXED_PAYLOAD, SYNC_FIX_PAYLOAD_LEN);
    app_sync_init();
    ll_ctrl_data.brct_tick = clock_time();
    ll_ctrl_data.state = LL_STATE_BRCT;
    gpio_write(DEBUG_PA7, 1);
}

_attribute_ram_code_sec_noinline_ void set_conn_rf_len()
{
    gen_fsk_packet_format_set(GEN_FSK_PACKET_FORMAT_FIXED_PAYLOAD, APP_FIX_PAYLOAD_LEN);
    tx_buffer[0] = APP_FIX_PAYLOAD_LEN;
}

_attribute_ram_code_sec_noinline_ void refresh_chnl_map()
{
    for (int i = 0; i < TOTAL_NB_CHNNEL; i++)
    {
        if (ll_conn_data[dev_id].qlty[i] <= AFH_BAD_CHNL_QLTY_THRES)
        {
            if (!BIT_IS_SET(ll_conn_data[dev_id].chnl_map[i / 8], i % 8))
                continue;

            ll_conn_data[dev_id].need_afh_flag = 1;

            if ((ll_conn_data[dev_id].abd_tail + 1) % AFH_NB_CHANNEL_ADB == ll_conn_data[dev_id].abd_head)
            {
                printf("chnl_abd fifo is full");
            }
            else
            {
                //只有当channel被放到abd数组才清chnl_map,防止这个channel丢失
                BIT_CLR(ll_conn_data[dev_id].chnl_map[i / 8], i % 8);
                // ll_conn_data[dev_id].qlty[i] = 0;
                ll_conn_data[dev_id].chnl_used--;

                ll_conn_data[dev_id].chnl_abd[ll_conn_data[dev_id].abd_tail++] = i;
                ll_conn_data[dev_id].abd_tail %= AFH_NB_CHANNEL_ADB;
            }
        }
    }

    if (ll_conn_data[dev_id].need_afh_flag == 2) // slave have channel map changed
        ll_conn_data[dev_id].need_afh_flag = 1;

    while (ll_conn_data[dev_id].chnl_used < AFH_NB_CHANNEL_MIN) //最少需要维持AFH_NB_CHANNEL_MIN (8)个可用channel
    {
        if (ll_conn_data[dev_id].abd_head == ll_conn_data[dev_id].abd_tail)
        {
            printf("chnl_abd fifo is empty");
            break;
        }
        else
        {
            unsigned char idex = ll_conn_data[dev_id].chnl_abd[ll_conn_data[dev_id].abd_head++];
            BIT_SET(ll_conn_data[dev_id].chnl_map[idex / 8], idex % 8);
            ll_conn_data[dev_id].abd_head %= AFH_NB_CHANNEL_ADB;
            ll_conn_data[dev_id].chnl_used++;
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
        unsigned char v = ll_conn_data[dev_id].chnl_map[i] ^ chnl_map_new[i];
        if (v)
        {
            for (int j = 0; j < 8; j++)
            {
                if (BIT_IS_SET(v, j))
                {
                    slave_chnl_chg_times++;
                    ll_conn_data[dev_id].need_afh_flag = 2;              //立即执行LL_CHG_CHNL_MAP_REQ(1)或等到下一次TIMER0到期(2)
                    ll_conn_data[dev_id].chnl_map[i] &= chnl_map_new[i]; // 清空对应的bit
                    ll_conn_data[dev_id].chnl_used--;

                    if ((ll_conn_data[dev_id].abd_tail + 1) % AFH_NB_CHANNEL_ADB == ll_conn_data[dev_id].abd_head)
                    {
                        printf("chnl_abd fifo is full");
                    }
                    else
                    {
                        ll_conn_data[dev_id].chnl_abd[ll_conn_data[dev_id].abd_tail++] = i * 8 + j;
                        ll_conn_data[dev_id].abd_tail %= AFH_NB_CHANNEL_ADB;
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
            ll_conn_data[dev_id].chnl_tbl[ll] = k;
        }
        else
        {
            u8 m = k;
            while (m >= numused)
            {
                m -= numused;
            }
            ll_conn_data[dev_id].chnl_tbl[ll] = tmp_table[m]; //将channel放入chnl_table
        }
        ll++;
    }
    return numused;
}

_attribute_ram_code_sec_noinline_ void build_chnl_clsf_req()
{
    ll_tx_packet_t *ptx = (ll_tx_packet_t *)tx_buffer;
    ptx->type = ll_conn_data[dev_id].snnesn << 2 & 0xc;
    ptx->len = 3 + 1;
    ptx->cmd = LL_CHNL_CLASSIFICATION_REQ;
    ptx->data[0] = AFH_MODE_ENABLE;
    ptx->data[1] = AFH_INT_MIN_S;
    ptx->data[2] = AFH_INT_MAX_S;
}

_attribute_ram_code_sec_noinline_ void build_chg_chnl_map_req()
{
    ll_tx_packet_t *ptx = (ll_tx_packet_t *)tx_buffer;
    ll_conn_data[dev_id].chg_chnl_map_times++;

    ptx->len = sizeof(ll_conn_data[dev_id].chnl_map) + 1;
    ptx->cmd = LL_CHG_CHNL_MAP_REQ;
    memcpy(ptx->data, (unsigned char *)&ll_conn_data[dev_id].chnl_map[0], sizeof(ll_conn_data[dev_id].chnl_map));
}

_attribute_ram_code_sec_noinline_ void build_ll_data_packet()
{
    ll_tx_packet_t *ptx = (ll_tx_packet_t *)tx_buffer;
    ptx->len = 29;
    ptx->cmd = LL_SYNC_DATA;
    memcpy(ptx->data, sycn_tx_frame, ptx->len);
    ptx->data[0] = ll_conn_data[dev_id].sync_chnn_idx;
}

_attribute_ram_code_sec_noinline_ void ll_snnesn_init()
{
    ll_conn_data[dev_id].snnesn = 0; // for master
}

_attribute_ram_code_sec_noinline_ u8 ll_flow_process(u8 peer, u8 local)
{
    local &= ~LL_FLOW_RCVD;
    local &= ~LL_FLOW_SENT;

#if DEBUG_LOG
    unsigned char temp_sn[4];
    temp_sn[0] = local;
    temp_sn[1] = peer;
    temp_sn[3] = ll_conn_data[dev_id].rx_crc_err_flag;
#endif

    u8 peer_nesn = peer & LL_FLOW_NESN;
    u8 peer_sn = peer & LL_FLOW_SN ? LL_FLOW_NESN : 0;
    u8 local_nesn = local & LL_FLOW_NESN;
    u8 local_sn = local & LL_FLOW_SN ? LL_FLOW_NESN : 0;

    if (peer_sn == local_nesn && !ll_conn_data[dev_id].rx_crc_err_flag) // 如果此时no RX BUFFER or MCU busy，也可以不toggle nesn,让对方重发此包
    {
        local = (local & ~LL_FLOW_NESN) | (peer_sn ? 0 : LL_FLOW_NESN);
        local |= LL_FLOW_RCVD;
    }

    if (ll_conn_data[dev_id].rx_crc_err_flag)
    {
        ll_conn_data[dev_id].rx_crc_err_flag = 0;
    }

    if (peer_nesn != local_sn)
    {
        local = (local & ~LL_FLOW_SN) | (peer_nesn << 1);
        local |= LL_FLOW_SENT;
    }
    else
    {
        ll_conn_data[dev_id].con_tx_cnt++;
    }

#if DEBUG_LOG
    temp_sn[2] = local;
    if (dev_id == DEVICE_LOG_NUM)
        log_msg("----after ll_flow_process-------  snnesn: \r\n", temp_sn, 4);
#endif
    return local;
}

_attribute_ram_code_sec_noinline_ void app_sync_task(void)
{
    if (ll_ctrl_data.state == LL_STATE_CONN)
        return;

    if (1 == tx_done_flag)
    {
        tx_done_flag = 0;
        async_tx_cnt++;
        gpio_toggle(DEBUG_PB0);
    }

    if (1 == rx_flag)
    {
        rx_flag = 0;
        async_rx_cnt++;

        gpio_toggle(GREEN_LED_PIN);
        rx_payload = gen_fsk_rx_payload_get((unsigned char *)rx_packet, (unsigned char *)&rx_payload_len);
        if (rx_payload[0] == LL_SYNC_RSP)
        {
            conn_dev_num++;
            dev_id = conn_dev_num - 1;
            gen_fsk_packet_format_set(GEN_FSK_PACKET_FORMAT_FIXED_PAYLOAD, APP_FIX_PAYLOAD_LEN);
            tx_buffer[0] = APP_FIX_PAYLOAD_LEN;

            broadcast_chnn_idx = (broadcast_chnn_idx + 1) % 3;
            rf_set_channel(broadcast_channel_map[broadcast_chnn_idx], 0);
            async_anchor_point += SYNC_BROADCAST_INTERVAL;

            ll_conn_data[dev_id].chnl_used = AFH_NB_CHANNEL_MAX;
            memcpy(ll_conn_data[dev_id].chnl_map, channel_map, sizeof(channel_map));

            rf_pkt_conn_req_t *conn_req_pkt = (rf_pkt_conn_req_t *)&tx_buffer[4];
            conn_req_pkt->opcode = LL_CONN_REQ;
            memcpy((unsigned char *)&(conn_req_pkt->ac[0]), conn_sync_word[dev_id], AC_LEN);
            // if(conn_dev_num == 0)
            //    conn_req_pkt->ts_time = SYNC_TX_INT_MS;
            // else
            // conn_req_pkt->ts_time = (sync_anchor_point - clock_time()) / SYNC_BROADCAST_INTERVAL + 10 * conn_dev_num ;

            conn_req_pkt->tx_int = SYNC_TX_INT_MS;
            memcpy((unsigned char *)&(conn_req_pkt->chm[0]), channel_map, sizeof(channel_map));
            conn_req_pkt->hop = AFH_HOP;
            chn_table_calc(channel_map, conn_req_pkt->hop); //提前算出channel table,但是并没有启用

            gen_fsk_stx2rx_start(tx_buffer, async_anchor_point, 350);

            // temp_anchor = async_anchor_point;

            return;
        }
        else if (rx_payload[0] == LL_CONN_RSP)
        {
            ll_conn_data[dev_id].is_first_tx = 1;
            ll_conn_data[dev_id].clsf_tick = clock_time();
#if DEBUG_LOG
            log_msg("----connect_rsp------- dev_id: \r\n", &dev_id, 1);
#endif

            if (dev_id != 0) //不是第一个连接的设备，返回
                return;
            sync_flg = 1;

            // temp_anchor_tick = clock_time();
            gpio_toggle(DEBUG_PA6);
            // temp_anchor_tick_d = temp_anchor_tick - temp_anchor;
            // temp_tick_ms = temp_anchor_tick_d / 16;
            // temp_tick_ms = temp_tick_ms / 1000;
            // sync_anchor_point = temp_anchor + (SYNC_TX_INT_MS * CLOCK_16M_SYS_TIMER_CLK_1MS);
            sync_anchor_point = async_anchor_point + (SYNC_TX_INT_MS * CLOCK_16M_SYS_TIMER_CLK_1MS);

            reg_system_irq_mask |= BIT(2);
            reg_system_tick_irq_level = clock_time() ^ BIT(31);
            irq_enable_type(FLD_IRQ_SYSTEM_TIMER);
            reg_system_tick_irq_level = sync_anchor_point;

            start_brct();
        }
    }

    if (1 == rx_timeout_flag)
    {
        rx_timeout_flag = 0;
        async_rxtimeout_cnt++;
        if (async_rxtimeout_cnt % 200 == 0)
            gpio_toggle(WHITE_LED_PIN);

        broadcast_chnn_idx = (broadcast_chnn_idx + 1) % 3;
        rf_set_channel(broadcast_channel_map[broadcast_chnn_idx], 0);
        async_anchor_point += SYNC_BROADCAST_INTERVAL;
        gen_fsk_stx2rx_start(tx_buffer, async_anchor_point, 350);
    }
}

_attribute_ram_code_sec_noinline_ void proc_rx_data()
{
    ll_rx_packet_t *prx = (ll_rx_packet_t *)rx_payload;
    u8 peer = prx->type >> 2 & 3;
    ll_conn_data[dev_id].snnesn = ll_flow_process(peer, ll_conn_data[dev_id].snnesn);

    if (!(ll_conn_data[dev_id].snnesn & LL_FLOW_RCVD))
    {
        return;
    }

    unsigned char cmd = prx->cmd;
    if (cmd == LL_ACCEPTED)
    {
        unsigned char opcode = prx->data[0];
        if (opcode == LL_CHG_CHNL_MAP_REQ)
        {
            chn_table_calc(ll_conn_data[dev_id].chnl_map, AFH_HOP);
            ll_conn_data[dev_id].need_afh_flag = 0;
        }
    }
    else if (cmd == LL_CHNL_CLASSIFICATION_IND)
    {
        refresh_chnl_abd();
    }
}

_attribute_ram_code_sec_noinline_ void proc_tx_data()
{
    if (ll_conn_data[dev_id].snnesn & LL_FLOW_SENT)
    {
        if (ll_conn_data[dev_id].need_afh_flag == 1)
        {
            build_chg_chnl_map_req();
        }
        else
        {
            build_ll_data_packet();
        }
    }
    else // retrans
    {
#if DEBUG_LOG
        if (dev_id == DEVICE_LOG_NUM)
            log_msg("0000000 tx data:", &ll_conn_data[dev_id].ptx_buff, APP_FIX_PAYLOAD_LEN);
#endif
        u8 *ptxb = (u8 *)&ll_conn_data[dev_id].ptx_buff;
        memcpy(tx_buffer, ptxb, ptxb[0] + 4);
    }
}

_attribute_ram_code_sec_noinline_ void flush_tx_data()
{

    if (ll_conn_data[dev_id].need_afh_flag == 1)
    {
        build_chg_chnl_map_req();
    }
    else
    {
        build_ll_data_packet();
    }

    ll_tx_packet_t *ptx = (ll_tx_packet_t *)tx_buffer;
    ptx->type = ll_conn_data[dev_id].snnesn << 2 & 0xc;
    memcpy(&ll_conn_data[dev_id].ptx_buff, tx_buffer, tx_buffer[0] + 4);
}

_attribute_ram_code_sec_noinline_ void send_chnl_clsf_req()
{

    ll_snnesn_init();

    build_chnl_clsf_req();
    gen_fsk_sync_word_len_set(SYNC_WORD_LEN_4BYTE);
    unsigned char conn_ac[4] = {0};
    memcpy(conn_ac, conn_sync_word[dev_id], AC_LEN);
    gen_fsk_sync_word_set(GEN_FSK_PIPE0, conn_ac); // set pipe0's sync word
    ll_conn_data[dev_id].sync_chnn_idx = 0;
    rf_set_channel(ll_conn_data[dev_id].chnl_tbl[ll_conn_data[dev_id].sync_chnn_idx], 0);
    if (dev_id == 0)
        sync_anchor_point += (SYNC_TX_INT_MS * CLOCK_16M_SYS_TIMER_CLK_1MS);
    memcpy(&ll_conn_data[dev_id].ptx_buff, tx_buffer, tx_buffer[0] + 4);

    gen_fsk_stx2rx_start(tx_buffer, clock_time(), 350);
    // gpio_write(DEBUG_PIN, 0);
}

_attribute_ram_code_sec_noinline_ void proc_rx_to()
{
    gpio_toggle(RED_LED_PIN);
    u8 *ptxb = (u8 *)&ll_conn_data[dev_id].ptx_buff;
    memcpy(tx_buffer, ptxb, ptxb[0] + 4);

    ll_conn_data[dev_id].con_tx_cnt++;
    if (ll_conn_data[dev_id].con_tx_cnt >= AUTO_FLUSH_CNT_THRES)
    {
#if DEBUG_LOG
        if (dev_id == DEVICE_LOG_NUM)
            log_msg("------ auto flush occurs ++++++++++++++++:", 0, 0);
#endif
        flush_tx_data();
        ll_conn_data[dev_id].con_tx_cnt = 0;
    }

#if DEBUG_LOG
    if (dev_id == DEVICE_LOG_NUM)
        log_msg("------ txo data:", &ll_conn_data[dev_id].ptx_buff, APP_FIX_PAYLOAD_LEN);
#endif

    ll_conn_data[dev_id].sync_chnn_idx = (ll_conn_data[dev_id].sync_chnn_idx + 1) % TOTAL_NB_CHNNEL;

    rf_set_channel(ll_conn_data[dev_id].chnl_tbl[ll_conn_data[dev_id].sync_chnn_idx], 0);
    if (dev_id == 0)
        sync_anchor_point += (SYNC_TX_INT_MS * CLOCK_16M_SYS_TIMER_CLK_1MS);
    gen_fsk_stx2rx_start(tx_buffer, clock_time(), 350);
}

_attribute_ram_code_sec_noinline_ void proc_rx_tx_data()
{
    gpio_toggle(DEBUG_PC0);

#if 1
    if (ll_conn_data[dev_id].is_first_tx)
    {
        send_chnl_clsf_req();
        ll_conn_data[dev_id].is_first_tx = 0;

        return;
    }

    set_pipe0_ac();

#if 1
    if (ll_conn_data[dev_id].rx_to_flag == 1)
    {
        proc_rx_to();
        ll_conn_data[dev_id].rx_to_flag = 0;

        return;
    }
#endif

    proc_tx_data();

    ll_tx_packet_t *ptx = (ll_tx_packet_t *)tx_buffer;
    ptx->type = ll_conn_data[dev_id].snnesn << 2 & 0xc;
    memcpy(&ll_conn_data[dev_id].ptx_buff, tx_buffer, tx_buffer[0] + 4);
#if DEBUG_LOG
    if (dev_id == DEVICE_LOG_NUM)
        log_msg("tx data:", &ll_conn_data[dev_id].ptx_buff, APP_FIX_PAYLOAD_LEN);
#endif

    ll_conn_data[dev_id].sync_chnn_idx = (ll_conn_data[dev_id].sync_chnn_idx + 1) % TOTAL_NB_CHNNEL;
    rf_set_channel(ll_conn_data[dev_id].chnl_tbl[ll_conn_data[dev_id].sync_chnn_idx], 0);
    if (dev_id == 0)
        sync_anchor_point += (SYNC_TX_INT_MS * CLOCK_16M_SYS_TIMER_CLK_1MS);

    gen_fsk_stx2rx_start(tx_buffer, clock_time(), 350);

#endif
}

_attribute_ram_code_sec_noinline_ void timer0_irq_func()
{
    dev_id++;
    proc_rx_tx_data();
}

_attribute_ram_code_sec_noinline_ void system_timer_irq_func()
{
    dev_id = 0;
    proc_rx_tx_data();
}

_attribute_ram_code_sec_noinline_ void app_cycle_send_task(void)
{
    if (ll_ctrl_data.state != LL_STATE_CONN)
        return;

    if (1 == tx_done_flag)
    {
        tx_done_flag = 0;
        sync_tx_cnt++;
    }

    if (1 == rx_flag)
    {
#if DEBUG_LOG
        if (dev_id == DEVICE_LOG_NUM)
            log_msg("++++++11 sync_rx_cnt: \r\n", &sync_rx_cnt, 1);
#endif

        rx_flag = 0;
        ll_conn_data[dev_id].con_tx_cnt = 0;

#if 1
        gpio_toggle(GREEN_LED_PIN);
        gpio_toggle(DEBUG_PB1);
        rx_payload = gen_fsk_rx_payload_get((unsigned char *)rx_packet, (unsigned char *)&rx_payload_len);

        proc_rx_data();
#if DEBUG_LOG
        if (dev_id == DEVICE_LOG_NUM)
            log_msg("recv data:", rx_payload, APP_FIX_PAYLOAD_LEN);
#endif
        if (conn_dev_num < MAX_CONN_NUM && dev_id == conn_dev_num - 1)
            start_brct();

#endif
    }

    if (1 == rx_timeout_flag)
    {
        gpio_toggle(DEBUG_PB2);
        rx_timeout_flag = 0;
        ll_conn_data[dev_id].rx_to_cnt++;
        ll_conn_data[dev_id].rx_to_flag = 1;
        sync_rx_timeout_cnt++;
#if DEBUG_LOG
        if (dev_id == DEVICE_LOG_NUM)
            log_msg("rx_timeout****************:", 0, 0);
#endif

        if (conn_dev_num < MAX_CONN_NUM && dev_id == conn_dev_num - 1)
            start_brct();
    }
}

#include "driver.h"
#include "genfsk_ll.h"
#include "common.h"

#define GREEN_LED_PIN           GPIO_PB4
#define WHITE_LED_PIN           GPIO_PB5
#define DEBUG_PIN               GPIO_PB2
#define DBG_EXECUTE_TIME        GPIO_PA6

#define RX_BUF_LEN              64
#define RX_BUF_NUM              4
#define APP_FIX_PAYLOAD_LEN     32

#define MAX_CHANNEL_MAP_SIZE    4
#define SYNC_BROADCAST_INTERVAL (1250 * 16) // 1.25MS


enum {
    CMD_SYNC_REQ             = 0xbb,
    CMD_SYNC_RSP             = 0xba,
    CMD_SYNC_DATA            = 0xbc,
};

static unsigned char __attribute__ ((aligned (4))) tx_buffer[64] = {0};
volatile static unsigned char rx_buf[RX_BUF_LEN * RX_BUF_NUM] __attribute__ ((aligned (4))) = {0};
volatile static unsigned char rx_ptr = 0;
volatile static unsigned char rx_payload_len = 0;
volatile static unsigned char *rx_payload = 0;
volatile static unsigned char *rx_packet = 0;
volatile static unsigned char rx_flag, tx_done_flag, rx_timeout_flag = 0;

unsigned char broadcast_chnn_idx = 0;
unsigned char broadcast_channel_map[3] = {8, 18, 28};
unsigned int sync_tx_cnt, sync_rx_cnt, sync_rxtimeout_cnt = 0;
typedef struct {
    unsigned short next_tx_interval_ms;
    unsigned short sync_tx_interval_ms;
    unsigned char channel_idx;
    unsigned char channel_map_size;
    unsigned char channel_map[MAX_CHANNEL_MAP_SIZE];
} app_sync_pkt_t;
app_sync_pkt_t sync_pkt;

unsigned char sync_chnn_idx = 0;
unsigned int sync_anchor_point = 0;
volatile unsigned char sync_flg = 0;
unsigned char sycn_tx_frame[APP_FIX_PAYLOAD_LEN] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                                                    0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10,
                                                    0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
                                                    0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20,
                                                    };


_attribute_ram_code_sec_noinline_ __attribute__((optimize("-Os"))) void irq_handler(void)
{
    unsigned int irq_src = irq_get_src();
    unsigned short rf_irq_src = rf_irq_src_get();

    if (irq_src & FLD_IRQ_ZB_RT_EN) // rf irq occurs
    {
        if (rf_irq_src & FLD_RF_IRQ_TX) // rf tx irq occurs
        {
            reg_rf_irq_status = FLD_RF_IRQ_TX;

            tx_done_flag = 1;

            gpio_toggle(DEBUG_PIN);

        }

        if (rf_irq_src & FLD_RF_IRQ_RX) // rf tx irq occurs
        {
            reg_rf_irq_status = FLD_RF_IRQ_RX;
            
            rx_packet = rx_buf + rx_ptr * RX_BUF_LEN;
            
            rx_ptr = (rx_ptr + 1) % RX_BUF_NUM;
            
            gen_fsk_rx_buffer_set((unsigned char *)(rx_buf + rx_ptr * RX_BUF_LEN), RX_BUF_LEN);
            
            if (gen_fsk_is_rx_crc_ok((unsigned char *)rx_packet)) 
            {

            }
            rx_flag = 1;
        }


        if (rf_irq_src & FLD_RF_IRQ_RX_TIMEOUT) //if rf tx irq occurs
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
    unsigned int pin = WHITE_LED_PIN | GREEN_LED_PIN;
    gpio_set_func(pin, AS_GPIO);
    gpio_set_input_en(pin, 0); //disable output
    gpio_set_output_en(pin, 1); //enable output
    gpio_write(pin, 0);

    pin = DEBUG_PIN | DBG_EXECUTE_TIME;
    gpio_set_func(pin, AS_GPIO);
    gpio_set_input_en(pin, 0); //disable output
    gpio_set_output_en(pin, 1); //enable output
    gpio_write(pin, 0);

    // rf init
    unsigned char sync_word[4] = {0x53, 0x78, 0x56, 0x52};
    // it needs to notice that this api is different from vulture / kite
    gen_fsk_datarate_set(GEN_FSK_DATARATE_1MBPS); //Note that this API must be invoked first before all other APIs
    gen_fsk_preamble_len_set(4);
    gen_fsk_sync_word_len_set(SYNC_WORD_LEN_4BYTE);
    gen_fsk_sync_word_set(GEN_FSK_PIPE0, sync_word); //set pipe0's sync word
    gen_fsk_pipe_open(GEN_FSK_PIPE0); // enable pipe0's reception
    gen_fsk_tx_pipe_set(GEN_FSK_PIPE0); // set pipe0 as the TX pipe
    gen_fsk_packet_format_set(GEN_FSK_PACKET_FORMAT_FIXED_PAYLOAD, APP_FIX_PAYLOAD_LEN);
    gen_fsk_radio_power_set(GEN_FSK_RADIO_POWER_0DBM);
    gen_fsk_rx_buffer_set((unsigned char *)(rx_buf + rx_ptr * RX_BUF_LEN), RX_BUF_LEN);
    gen_fsk_channel_set(6);
    gen_fsk_radio_state_set(GEN_FSK_STATE_AUTO); // set transceiver to basic TX state
    gen_fsk_tx_settle_set(149);
    gen_fsk_rx_settle_set(89);

    rf_irq_enable(FLD_RF_IRQ_TX | FLD_RF_IRQ_RX | FLD_RF_IRQ_RX_TIMEOUT); // enable rf tx irq
    irq_enable_type(FLD_IRQ_ZB_RT_EN); // enable RF irq
    irq_enable(); // enable general irq
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
    sync_pkt.next_tx_interval_ms = 1000;
    sync_pkt.sync_tx_interval_ms = 500;

    // init sync channel map 6->36->66->46->6...
    unsigned char sync_channel_map[4] = {6, 36, 66, 46};
    sync_pkt.channel_map_size = sizeof(sync_channel_map);
    memcpy((unsigned char *)&(sync_pkt.channel_map[0]), sync_channel_map, sizeof(sync_channel_map));
    sync_pkt.channel_idx = 0; // start index

    tx_buffer[0] = APP_FIX_PAYLOAD_LEN;
    tx_buffer[1] = 0x00;
    tx_buffer[2] = 0x00;
    tx_buffer[3] = 0x00;
    tx_buffer[4] = CMD_SYNC_REQ;
    memcpy(&tx_buffer[6], (unsigned char *)&sync_pkt, sizeof(sync_pkt));

    rf_set_channel(broadcast_channel_map[0], 0);
    sync_anchor_point = clock_time() + SYNC_BROADCAST_INTERVAL;
    gen_fsk_stx2rx_start(tx_buffer, sync_anchor_point, 350);
}

_attribute_ram_code_sec_noinline_ static void led_blink(unsigned char cnt)
{
    while (cnt--)
    {   
        gpio_toggle(GREEN_LED_PIN | WHITE_LED_PIN);
        WaitMs(50);
    }
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
        gpio_toggle(DEBUG_PIN);
    }

    if (1 == rx_flag)
    {
        rx_flag = 0;
        sync_rx_cnt++;
        gpio_toggle(GREEN_LED_PIN);
        rx_payload = gen_fsk_rx_payload_get(rx_packet, &rx_payload_len);
        if (rx_payload[0] == CMD_SYNC_RSP)
        {
            sync_flg = 1;
            memcpy(&tx_buffer[5], sycn_tx_frame, sizeof(sycn_tx_frame));
            
            sync_chnn_idx = 0;
            rf_set_channel(sync_pkt.channel_map[sync_chnn_idx], 0);
            sync_anchor_point += (sync_pkt.next_tx_interval_ms * CLOCK_16M_SYS_TIMER_CLK_1MS);
            gen_fsk_stx_start(tx_buffer, sync_anchor_point);
            led_blink(5); // delay 250ms
            return;
        }
    }
    
    if (1 == rx_timeout_flag)
    {
        rx_timeout_flag = 0;
        sync_rxtimeout_cnt++;
        gpio_toggle(WHITE_LED_PIN);

        broadcast_chnn_idx = (broadcast_chnn_idx + 1) % 3;
        rf_set_channel(broadcast_channel_map[broadcast_chnn_idx], 0);
        sync_anchor_point += SYNC_BROADCAST_INTERVAL;
        gen_fsk_stx2rx_start(tx_buffer, sync_anchor_point, 350);
    }
}

_attribute_ram_code_sec_noinline_ void app_cycle_send_task(void)
{
    if (sync_flg == 0)
        return;

    if (1 == tx_done_flag)
    {
        tx_done_flag = 0;
        sync_tx_cnt++;
        gpio_toggle(DEBUG_PIN);
        gpio_toggle(GREEN_LED_PIN);

        sync_chnn_idx = (sync_chnn_idx + 1) % sync_pkt.channel_map_size;
        tx_buffer[4] = CMD_SYNC_DATA;
        tx_buffer[5] = sync_chnn_idx;

        rf_set_channel(sync_pkt.channel_map[sync_chnn_idx], 0);
        sync_anchor_point += (sync_pkt.sync_tx_interval_ms * CLOCK_16M_SYS_TIMER_CLK_1MS);
        gen_fsk_stx_start(tx_buffer, sync_anchor_point);
        // gen_fsk_stx2rx_start(tx_buffer, sync_anchor_point, 350);
    }

    if (1 == rx_flag)
    {
        rx_flag = 0;
        sync_rx_cnt++;
    } 
}

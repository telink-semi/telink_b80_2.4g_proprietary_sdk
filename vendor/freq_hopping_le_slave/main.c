#include "driver.h"
#include "genfsk_ll.h"

#define GREEN_LED_PIN           GPIO_PB4
#define RED_LED_PIN             GPIO_PB6
#define DBG_RXTIMEOUT_IRQ_PIN   GPIO_PB7
#define DBG_RX_IRQ_PIN          GPIO_PB2
#define DBG_SUSPEND_PIN         GPIO_PA5
#define DBG_EXECUTE_TIME        GPIO_PA6

#define DBG_SUSPEND             1
#define TX_INTERVAL             (500 * 1000 * 16)
#define RX_MARGIN               (10 * 1000 * 16)
#define RX_MARGIN_PM_CALIB      (500 * 16) // for some init after wake up

#define RX_BUF_LEN              64
#define RX_BUF_NUM              4
#define APP_FIX_PAYLOAD_LEN     32
#define MAX_CHANNEL_MAP_SIZE    4

#define SYNC_BROADCAST_INTERVAL (1250 * 16) // 1.25MS

enum {
    RX_SHORT_WINDOW_US       = 625,
    RX_NORMAL_WINDOW_US      = 1250,
    RX_LONG_WINDOW_US        = 2500,
};

enum {
    CMD_SYNC_REQ             = 0xbb,
    CMD_SYNC_RSP             = 0xba,
    CMD_SYNC_DATA            = 0xbc,
};

static unsigned char __attribute__ ((aligned (4))) tx_buffer[64] = {0};

volatile static unsigned char rx_buf[RX_BUF_LEN * RX_BUF_NUM] __attribute__ ((aligned (4))) = {};
volatile static unsigned char rx_ptr = 0;
volatile static unsigned char rx_payload_len = 0;
volatile static unsigned char *rx_payload = 0;
volatile static unsigned char *rx_packet = 0;

volatile static unsigned char tx_done_flag, rx_flag, rx_first_timeout_flag = 0;
volatile static unsigned int sleep_cnt, rx_cnt, rx_first_timeout_cnt = 0;

volatile static unsigned int rx_timestamp = 0;
volatile static unsigned char rx_rssi = 0;

unsigned char sycn_ack_frame[APP_FIX_PAYLOAD_LEN] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                                                    0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10,
                                                    0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18,
                                                    0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20,
                                                    };
volatile unsigned char sync_flg = 0;
volatile unsigned int next_rx_point = 0;
volatile int offset_tick = 0;

volatile unsigned char next_chnn_idx = 0;                                
unsigned short sync_tx_interval_ms = 0;
unsigned char sync_channel_map_size = 0;
unsigned char sync_channel_map[MAX_CHANNEL_MAP_SIZE] = {0};
unsigned int sync_tx_cnt, sync_rx_cnt, sync_first_timeout_cnt = 0;

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
            gpio_toggle(DBG_RX_IRQ_PIN);
            rx_cnt++;
            rx_packet = rx_buf + rx_ptr * RX_BUF_LEN;
            rx_ptr = (rx_ptr + 1) % RX_BUF_NUM;
            gen_fsk_rx_buffer_set((unsigned char *)(rx_buf + rx_ptr * RX_BUF_LEN), RX_BUF_LEN);
            if (gen_fsk_is_rx_crc_ok((unsigned char *)rx_packet))
            {
                // rx_flag = 1;
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
    // gpio config
    unsigned int pin = GREEN_LED_PIN | RED_LED_PIN | DBG_RX_IRQ_PIN | DBG_RXTIMEOUT_IRQ_PIN;
    gpio_set_func(pin, AS_GPIO);
    gpio_set_input_en(pin, 0); //disable output
    gpio_set_output_en(pin, 1); //enable output
    gpio_write(pin, 0);

    pin = DBG_SUSPEND_PIN | DBG_EXECUTE_TIME;
    gpio_set_func(pin, AS_GPIO);
    gpio_set_input_en(pin, 0); //disable output
    gpio_set_output_en(pin, 1); //enable output
    gpio_write(pin, 0);

    // rf config
    unsigned char sync_word[4] = {0x53, 0x78, 0x56, 0x52};
    gen_fsk_datarate_set(GEN_FSK_DATARATE_1MBPS); //Note that this API must be invoked first before all other APIs
    gen_fsk_preamble_len_set(4);
    gen_fsk_sync_word_len_set(SYNC_WORD_LEN_4BYTE);
    gen_fsk_sync_word_set(GEN_FSK_PIPE0, sync_word); //set pipe0's sync word
    gen_fsk_pipe_open(GEN_FSK_PIPE0); //enable pipe0's reception
    gen_fsk_tx_pipe_set(GEN_FSK_PIPE0); //set pipe0 as the TX pipe
    gen_fsk_packet_format_set(GEN_FSK_PACKET_FORMAT_FIXED_PAYLOAD, APP_FIX_PAYLOAD_LEN);
    gen_fsk_radio_power_set(GEN_FSK_RADIO_POWER_0DBM);
    gen_fsk_rx_buffer_set((unsigned char *)(rx_buf + rx_ptr * RX_BUF_LEN), RX_BUF_LEN);
    gen_fsk_channel_set(6); //set rf freq as 2403.5MHz
    gen_fsk_radio_state_set(GEN_FSK_STATE_AUTO); //set transceiver to basic RX state
    gen_fsk_rx_settle_set(89);

    //irq configuration
    rf_irq_disable(FLD_RF_IRQ_ALL);
    rf_irq_enable(FLD_RF_IRQ_RX | FLD_RF_IRQ_FIRST_TIMEOUT); // enable rf rx and rx first timeout irq
    irq_enable_type(FLD_IRQ_ZB_RT_EN); //enable RF irq
    irq_enable(); //enable general irq
}

_attribute_ram_code_sec_noinline_ void rf_recovery_init(void)
{
    unsigned char sync_word[4] = {0x53, 0x78, 0x56, 0x52};
    // it needs to notice that this api is different from vulture / kite
    gen_fsk_datarate_set(GEN_FSK_DATARATE_1MBPS); //Note that this API must be invoked first before all other APIs
    gen_fsk_preamble_len_set(4);
    gen_fsk_sync_word_len_set(SYNC_WORD_LEN_4BYTE);
    gen_fsk_sync_word_set(GEN_FSK_PIPE0, sync_word); //set pipe0's sync word
    gen_fsk_pipe_open(GEN_FSK_PIPE0); //enable pipe0's reception
    gen_fsk_tx_pipe_set(GEN_FSK_PIPE0); //set pipe0 as the TX pipe
    gen_fsk_packet_format_set(GEN_FSK_PACKET_FORMAT_FIXED_PAYLOAD, APP_FIX_PAYLOAD_LEN);
    gen_fsk_radio_power_set(GEN_FSK_RADIO_POWER_0DBM);
    gen_fsk_rx_buffer_set((unsigned char *)(rx_buf + rx_ptr * RX_BUF_LEN), RX_BUF_LEN);
//    gen_fsk_channel_set(6); //set rf freq as 2403.5MHz
    gen_fsk_radio_state_set(GEN_FSK_STATE_AUTO); //set transceiver to basic RX state
    gen_fsk_rx_settle_set(149);
    gen_fsk_rx_settle_set(89);

    //irq configuration
    rf_irq_disable(FLD_RF_IRQ_ALL);
    rf_irq_enable(FLD_RF_IRQ_RX | FLD_RF_IRQ_TX | FLD_RF_IRQ_FIRST_TIMEOUT); // enable rf rx and rx first timeout irq
    irq_enable_type(FLD_IRQ_ZB_RT_EN); //enable RF irq
    irq_enable(); //enable general irq
}

void app_sync_init(void);

void app_sync_task(void);

void app_low_energy_task(void);

_attribute_ram_code_sec_noinline_ int main(void)
{
    blc_pm_select_internal_32k_crystal();

    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    clock_init(SYS_CLK_24M_Crystal);

#if PM_32KRC_CALIBRATION
    pm_32k_rc_offset_init();
    pm_register_tick32kGet_callback(pm_get_32k_rc_calib);
#endif

    user_init();

    app_sync_init();

    while (1)
    {
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

_attribute_ram_code_sec_noinline_ void app_sync_init(void)
{
    tx_buffer[0] = sizeof(sycn_ack_frame);
    tx_buffer[1] = 0x00;
    tx_buffer[2] = 0x00;
    tx_buffer[3] = 0x00;
    memcpy(&tx_buffer[4], (const void *)sycn_ack_frame, sizeof(sycn_ack_frame));
    tx_buffer[4] = CMD_SYNC_RSP;

    broadcast_chnn_idx = 2;
    rf_set_channel(broadcast_channel_map[broadcast_chnn_idx], 0);    
    gen_fsk_srx_start(clock_time(), 3 * SYNC_BROADCAST_INTERVAL);
}


_attribute_ram_code_sec_noinline_ void app_sync_task(void)
{
    if (sync_flg != 0)
        return;

    if (1 == tx_done_flag)
    {
        tx_done_flag = 0;
        sync_tx_cnt++;
    }

    if (1 == rx_flag)
    {
        rx_flag = 0;
        sync_rx_cnt++;
        rx_payload = gen_fsk_rx_payload_get(rx_packet, &rx_payload_len);
        if (rx_payload[0] == CMD_SYNC_REQ)
        {
            sync_tx_interval_ms = (rx_payload[5] << 8) | rx_payload[4];
            next_chnn_idx = rx_payload[6];
            sync_channel_map_size = rx_payload[7];
            memcpy(sync_channel_map, (const void *)&rx_payload[8], sync_channel_map_size);
            tx_buffer[4] = CMD_SYNC_RSP;
            gen_fsk_stx_start(tx_buffer, clock_time());
            sync_flg = 1;
            led_blink(5);

            // sync setup
            rf_set_channel(sync_channel_map[next_chnn_idx], 0);
            gen_fsk_srx_start(clock_time(), 0); // RX first timeout is disabled and the transceiver won't exit the RX state until a packet arrives
            return;
        }        
    }

    if (1 == rx_first_timeout_flag)
    {   
        rx_first_timeout_flag = 0;
        sync_first_timeout_cnt++;

        broadcast_chnn_idx = (broadcast_chnn_idx + 1) % 3;
        rf_set_channel(broadcast_channel_map[broadcast_chnn_idx], 0);    
        gen_fsk_srx_start(clock_time(), 3 * SYNC_BROADCAST_INTERVAL);
    }
}


_attribute_ram_code_sec_noinline_ void app_low_energy_task(void)
{
    if (sync_flg == 0)
        return;

    if (1 == rx_flag)
    {
        gpio_toggle(GREEN_LED_PIN);
        rx_flag = 0;
        rx_timestamp = gen_fsk_rx_timestamp_get((unsigned char *)rx_packet);
        rx_payload = gen_fsk_rx_payload_get((unsigned char *)rx_packet, (unsigned char *)&rx_payload_len);
        rx_rssi = gen_fsk_rx_packet_rssi_get((unsigned char *)rx_packet) + 110;

        if (sleep_cnt != 0)
            offset_tick = next_rx_point - rx_timestamp;

        // next_rx_point = rx_timestamp + TX_INTERVAL - 64 * 16; 
        // 64 us -> 1MBPS 4 byte preamble + 4 byte sync word
        next_rx_point = rx_timestamp + (sync_tx_interval_ms * CLOCK_16M_SYS_TIMER_CLK_1MS)
                - (64 * CLOCK_16M_SYS_TIMER_CLK_1US);
#if PM_32KRC_CALIBRATION
        pm_cal_32k_rc_offset(offset_tick);
#endif
        
#if (DBG_SUSPEND == 1)
        if (offset_tick < 0)
            offset_tick = -offset_tick;
        // it pays about 5 ms for debug printf | rx irq cnt | cmd | channel idx | channel map |
        printf("%d. %1x, %1x, %d %d %d %d, %4x", rx_cnt, rx_payload[0], rx_payload[1], sync_channel_map[0], 
            sync_channel_map[1], sync_channel_map[2], sync_channel_map[3], offset_tick);
#endif

        gpio_toggle(DBG_SUSPEND_PIN);
        cpu_sleep_wakeup(SUSPEND_MODE, PM_WAKEUP_TIMER, next_rx_point - RX_MARGIN_PM_CALIB);
        gpio_toggle(DBG_SUSPEND_PIN);
        sleep_cnt++;

        rf_recovery_init();
        next_chnn_idx = (next_chnn_idx + 1) % sync_channel_map_size;
        rf_set_channel(sync_channel_map[next_chnn_idx], 0);
        gen_fsk_srx_start(clock_time(), RX_NORMAL_WINDOW_US);
    }

    if (1 == rx_first_timeout_flag)
    {
        rx_first_timeout_flag = 0;
        gpio_toggle(RED_LED_PIN);

        next_rx_point = next_rx_point + (sync_tx_interval_ms * CLOCK_16M_SYS_TIMER_CLK_1MS);
        cpu_sleep_wakeup(SUSPEND_MODE, PM_WAKEUP_TIMER, next_rx_point - RX_MARGIN_PM_CALIB);
        
        rf_recovery_init();
        next_chnn_idx = (next_chnn_idx + 1) % sync_channel_map_size;
        rf_set_channel(sync_channel_map[next_chnn_idx], 0);
        gen_fsk_srx_start(clock_time(), RX_LONG_WINDOW_US);
    }
}

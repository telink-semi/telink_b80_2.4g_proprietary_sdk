#include "driver.h"
#include "genfsk_ll.h"

#define GREEN_LED_PIN           GPIO_PB4
#define WHITE_LED_PIN           GPIO_PB5
#define DEBUG_PIN               GPIO_PB2
#define RX_BUF_LEN              64
#define RX_BUF_NUM              4

unsigned char tx_payload[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
static unsigned char __attribute__ ((aligned (4))) tx_buffer[64] = {0};
volatile static unsigned char tx_done_flag = 0;
volatile unsigned int tx_cnt = 0;

volatile static unsigned char rx_buf[RX_BUF_LEN * RX_BUF_NUM] __attribute__ ((aligned (4))) = {};
volatile static unsigned char rx_ptr = 0;
volatile static unsigned char rx_flag, rx_timeout_flag = 0;
volatile static unsigned char rx_payload_len = 0;
volatile static unsigned int rx_timestamp = 0;
volatile static unsigned char rx_rssi = 0;
volatile static unsigned char *rx_payload = 0;
volatile static unsigned char *rx_packet = 0;
volatile static unsigned int rx_cnt = 0;

_attribute_ram_code_sec_noinline_ __attribute__((optimize("-Os"))) void irq_handler(void)
{
    unsigned int irq_src = irq_get_src();
    unsigned short rf_irq_src = rf_irq_src_get();

    if (irq_src & FLD_IRQ_ZB_RT_EN) //if rf irq occurs
    {
        if (rf_irq_src & FLD_RF_IRQ_RX) //if rf rx irq occurs
        {
            rf_irq_clr_src(FLD_RF_IRQ_RX);
            rx_cnt++;
            rx_packet = rx_buf + rx_ptr * RX_BUF_LEN;
            rx_ptr = (rx_ptr + 1) % RX_BUF_NUM;
            gen_fsk_rx_buffer_set((unsigned char *)(rx_buf + rx_ptr * RX_BUF_LEN), RX_BUF_LEN);

            if (gen_fsk_is_rx_crc_ok((unsigned char *)rx_packet))
            {
                gpio_toggle(DEBUG_PIN);
            }
            rx_flag = 1;
        }

        if (rf_irq_src & FLD_RF_IRQ_RX_TIMEOUT) //if rf tx irq occurs
        {
            rf_irq_clr_src(FLD_RF_IRQ_RX_TIMEOUT);
            gpio_toggle(DEBUG_PIN);
            rx_timeout_flag = 1;
        }

        if (rf_irq_src & FLD_RF_IRQ_TX)
        {
            rf_irq_clr_src(FLD_RF_IRQ_TX);
            tx_done_flag = 1;
            gpio_toggle(DEBUG_PIN);
        }
    }
    irq_clr_src2(FLD_IRQ_ALL);
}

void user_init(void)
{
    unsigned char sync_word[4] = {0x53, 0x78, 0x56, 0x52};
    gpio_set_func(GREEN_LED_PIN | WHITE_LED_PIN | DEBUG_PIN, AS_GPIO);
    gpio_set_input_en(GREEN_LED_PIN | WHITE_LED_PIN | DEBUG_PIN, 0); //disable output
    gpio_set_output_en(GREEN_LED_PIN | WHITE_LED_PIN | DEBUG_PIN, 1); //enable output
    gpio_write(GREEN_LED_PIN | WHITE_LED_PIN | DEBUG_PIN, 0);

    // it needs to notice that this api is different from vulture / kite
    gen_fsk_datarate_set(GEN_FSK_DATARATE_1MBPS); //Note that this API must be invoked first before all other APIs

    gen_fsk_preamble_len_set(4);

    gen_fsk_sync_word_len_set(SYNC_WORD_LEN_4BYTE);

    gen_fsk_sync_word_set(GEN_FSK_PIPE0, sync_word); //set pipe0's sync word

    gen_fsk_pipe_open(GEN_FSK_PIPE0); // enable pipe0's reception

    gen_fsk_tx_pipe_set(GEN_FSK_PIPE0); // set pipe0 as the TX pipe

    gen_fsk_packet_format_set(GEN_FSK_PACKET_FORMAT_FIXED_PAYLOAD, sizeof(tx_payload));

    gen_fsk_radio_power_set(GEN_FSK_RADIO_POWER_0DBM);

    gen_fsk_rx_buffer_set((unsigned char *)(rx_buf + rx_ptr * RX_BUF_LEN), RX_BUF_LEN);

    gen_fsk_channel_set(7);

    gen_fsk_radio_state_set(GEN_FSK_STATE_AUTO); // set transceiver to basic TX state

    gen_fsk_tx_settle_set(149);

    rf_irq_enable(FLD_RF_IRQ_TX | FLD_RF_IRQ_RX | FLD_RF_IRQ_RX_TIMEOUT); // enable rf tx irq
    irq_enable_type(FLD_IRQ_ZB_RT_EN); // enable RF irq
    irq_enable(); // enable general irq
}

int main(void)
{
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    clock_init(SYS_CLK_24M_Crystal);

    user_init();

    //fill the dma info in tx buffer
    tx_buffer[0] = sizeof(tx_payload);
    tx_buffer[1] = 0x00;
    tx_buffer[2] = 0x00;
    tx_buffer[3] = 0x00;
    memcpy(&tx_buffer[4], tx_payload, sizeof(tx_payload));

    gen_fsk_stx2rx_start(tx_buffer, clock_time() + 50 * 16, 250);

    while (1)
    {
        if (1 == tx_done_flag)
        {
            tx_done_flag = 0;
            tx_buffer[4]++;
            tx_cnt++;
            gpio_toggle(WHITE_LED_PIN);
        }

        if (1 == rx_timeout_flag)
        {
            rx_timeout_flag = 0;
            WaitMs(100);
            gen_fsk_stx2rx_start(tx_buffer, clock_time() + 50 * 16, 250);
        }

        if (1 == rx_flag)
        {
            rx_flag = 0;
            rx_payload = gen_fsk_rx_payload_get((unsigned char *)rx_packet,
                    (unsigned char *)&rx_payload_len);
            rx_rssi = gen_fsk_rx_packet_rssi_get((unsigned char *)rx_packet) + 110;
            rx_timestamp = gen_fsk_rx_timestamp_get((unsigned char *)rx_packet);

            gpio_toggle(GREEN_LED_PIN);
            WaitMs(100);
            gen_fsk_stx2rx_start(tx_buffer, clock_time() + 50 * 16, 250);
        }
    }
}

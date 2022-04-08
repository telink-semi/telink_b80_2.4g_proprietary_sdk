#include "driver.h"
#include "genfsk_ll.h"

#define GREEN_LED_PIN           GPIO_PB4
#define RX_BUF_LEN              64
#define RX_BUF_NUM              4
volatile static unsigned char rx_buf[RX_BUF_LEN * RX_BUF_NUM] __attribute__ ((aligned (4))) = {};
volatile static unsigned char rx_ptr = 0;
volatile static unsigned char rx_flag = 0;
volatile static unsigned char *rx_packet = 0;
volatile static unsigned char rx_payload_len = 0;
volatile static unsigned char *rx_payload = 0;
volatile static unsigned char rssi = 0;
volatile static unsigned int rx_timestamp = 0;
volatile static unsigned int rx_cnt = 0;

_attribute_ram_code_sec_noinline_ __attribute__((optimize("-Os")))void irq_handler (void)
{
    unsigned int irq_src = irq_get_src();
    unsigned short rf_irq_src = rf_irq_src_get();

    if (irq_src & FLD_IRQ_ZB_RT_EN) //if rf irq occurs
    {
        if (rf_irq_src & FLD_RF_IRQ_RX) //if rf rx irq occurs
        {
            rx_cnt++;
            rx_packet = rx_buf + rx_ptr * RX_BUF_LEN;
            //set to next rx_buf
            rx_ptr = (rx_ptr + 1) % RX_BUF_NUM;
            gen_fsk_rx_buffer_set((unsigned char *)(rx_buf + rx_ptr * RX_BUF_LEN), RX_BUF_LEN);
     
            if (gen_fsk_is_rx_crc_ok(rx_packet))
            {
                rx_flag = 1;
            }
        }
        rf_irq_clr_src(FLD_RF_IRQ_ALL);
    }
    irq_clr_src2(FLD_IRQ_ALL);
}

void user_init(void)
{
    unsigned char sync_word[4] = {0x53, 0x78, 0x56, 0x52};
    gpio_set_func(GREEN_LED_PIN, AS_GPIO);
    gpio_set_input_en(GREEN_LED_PIN, 0); //disable output
    gpio_set_output_en(GREEN_LED_PIN, 1); //enable output
    gpio_write(GREEN_LED_PIN, 0);

    // it needs to notice that this api is different from vulture / kite
    gen_fsk_datarate_set(GEN_FSK_DATARATE_1MBPS); //Note that this API must be invoked first before all other APIs

    gen_fsk_preamble_len_set(4);

    gen_fsk_sync_word_len_set(SYNC_WORD_LEN_4BYTE);

    gen_fsk_sync_word_set(GEN_FSK_PIPE0, sync_word); //set pipe0's sync word

    gen_fsk_pipe_open(GEN_FSK_PIPE0); //enable pipe0's reception

    gen_fsk_tx_pipe_set(GEN_FSK_PIPE0); //set pipe0 as the TX pipe

    gen_fsk_packet_format_set(GEN_FSK_PACKET_FORMAT_FIXED_PAYLOAD, 8);

    gen_fsk_radio_power_set(GEN_FSK_RADIO_POWER_0DBM);

    gen_fsk_rx_buffer_set(rx_buf + rx_ptr * RX_BUF_LEN, RX_BUF_LEN);

    gen_fsk_channel_set(7); //set rf freq as 2403.5MHz

    gen_fsk_radio_state_set(GEN_FSK_STATE_RX); //set transceiver to basic RX state

    gen_fsk_rx_settle_set(89);

    WaitUs(90); //wait for rx settle

    rf_irq_enable(FLD_RF_IRQ_RX);       //enable rf tx irq
    irq_enable_type(FLD_IRQ_ZB_RT_EN);  //enable RF irq
    irq_enable();                       //enable general irq
}

int main(void)
{
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    clock_init(SYS_CLK_24M_Crystal);

    user_init();

    while (1)
    {
        if (rx_flag)
        {
            rx_flag = 0;
            rx_payload = gen_fsk_rx_payload_get(rx_packet, &rx_payload_len);
            rssi = (gen_fsk_rx_packet_rssi_get(rx_packet) + 110);
            rx_timestamp = gen_fsk_rx_timestamp_get(rx_packet);
            gpio_toggle(GREEN_LED_PIN);
        }
    }
}

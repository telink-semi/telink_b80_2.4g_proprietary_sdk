#include "driver.h"

#define WHITE_LED_PIN           GPIO_PB5
#define CLOCK_SYS_CLOCK_HZ      24000000
#define CLOCK_SPI               100000
#define TRANS_DATA_LEN          16

spi_pin_config_t m_spi_master_pin_config = {
        .spi_clk_pin        = GPIO_PA0,
        .spi_csn_pin        = GPIO_PA4,
        .spi_mosi_io0_pin   = GPIO_PB2,
        .spi_miso_io1_pin   = GPIO_PB4,
};

spi_config_t m_spi_master_config = {
        .spi_io_mode        = SPI_SINGLE_MODE,
        .spi_dummy_cnt      = 8,
        .spi_cmd_en         = 1,
        .spi_addr_en        = 0,
        .spi_addr_len       = 0,
        .spi_cmd_fmt_en     = 0,
        .spi_addr_fmt_en    = 0,
};

typedef struct {
    unsigned int dma_len;
    unsigned char address[4];
    unsigned int data_len;
    unsigned char data[TRANS_DATA_LEN];
} __attribute__((aligned(4))) spi_dma_protocol_t;

spi_dma_protocol_t spi_master_tx_buff = {
        .dma_len        = TRANS_DATA_LEN + 8,
        .address        = {0x00,0x04,0x40,0x00},    //distinguish B91m and B80
        .data_len       = TRANS_DATA_LEN,
        .data           = {0x00, 0x11, 0x22, 0x33,
                           0x44, 0x55, 0x66, 0x77,
                           0x88, 0x99, 0xaa, 0xbb,
                           0xcc, 0xdd, 0xee, 0xff},
};
volatile unsigned char spi_master_rx_buff[TRANS_DATA_LEN + 4] __attribute__((aligned(4))) = {0x00};
volatile unsigned char end_irq_flag = 0;

_attribute_ram_code_sec_noinline_ void irq_handler(void)
{
    if(spi_get_irq_status(SPI_END_INT))
    {
        spi_clr_irq_status(SPI_END_INT);//clr
        end_irq_flag = 1;
    }
}

void user_init(void)
{
    WaitMs(1000);
    // led init
    gpio_set_func(WHITE_LED_PIN, AS_GPIO);
    gpio_set_output_en(WHITE_LED_PIN, 1);       //enable output
    gpio_set_input_en(WHITE_LED_PIN, 0);         //disable input

    // spi init
    spi_set_pin(&m_spi_master_pin_config);
    spi_master_init((unsigned char)(CLOCK_SYS_CLOCK_HZ / (2 * CLOCK_SPI) - 1), SPI_MODE0);
    spi_master_config_plus(&m_spi_master_config);

    spi_set_irq_mask(SPI_END_INT_EN);
    irq_enable_type(FLD_IRQ_MIX_CMD_EN);
    irq_enable();
}

static void spi_ready()
{
    while(!end_irq_flag);//Wait for spi transmission end interrupt.
    end_irq_flag = 0;
    while (spi_is_busy());//Spi transmission end interrupt doesn't mean spi is free. Better to check spi busy state again.
}

int main()
{
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    clock_init(SYS_CLK_24M_Crystal);

    gpio_init(0);

    user_init();

    while (1)
    {
        gpio_toggle(WHITE_LED_PIN);
        WaitMs(500);
        spi_master_tx_buff.data[1]++;
        spi_master_write_dma_plus(SPI_WRITE_DATA_SINGLE_CMD, 0x00, (unsigned char*)&spi_master_tx_buff,
                (TRANS_DATA_LEN + 8), SPI_MODE_WR_DUMMY_WRITE);
        spi_ready();
        spi_master_read_dma_plus(SPI_READ_DATA_SINGLE_CMD, 0x00, (unsigned char*)spi_master_rx_buff,
                TRANS_DATA_LEN, SPI_MODE_RD_DUMMY_READ);
        spi_ready();

    }
    return 0;
}

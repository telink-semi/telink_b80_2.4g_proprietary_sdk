#include "driver.h"

#define WHITE_LED_PIN           GPIO_PA6
#define CLOCK_SYS_CLOCK_HZ      24000000
#define CLOCK_SPI               100000

#define SPI_RX_BUFF_LEN         28
#define DATA_BYTE_OFFSET        8 //must be a multiple 4

spi_pin_config_t spi_slave_pin_config = {
        .spi_clk_pin        = GPIO_PC0,
        .spi_csn_pin        = GPIO_PC4,
        .spi_mosi_io0_pin   = GPIO_PB2,
        .spi_miso_io1_pin   = GPIO_PB4,
};

volatile unsigned char spi_slave_rx_buff[SPI_RX_BUFF_LEN] __attribute__((aligned(4))) = {0x00};

_attribute_ram_code_sec_ void irq_handler(void)
{
    unsigned char spi_slave_cmd ;
    if (spi_get_irq_status(SPI_SLV_CMD_INT))
    {
        spi_slave_cmd = spi_slave_get_cmd();
        switch (spi_slave_cmd)
        {
            case SPI_READ_DATA_SINGLE_CMD:
            case SPI_READ_DATA_DUAL_CMD:
            case SPI_READ_DATA_QUAD_CMD:
                spi_tx_dma_en();
            break;

            case SPI_WRITE_DATA_SINGLE_CMD:
            case SPI_WRITE_DATA_DUAL_CMD:
            case SPI_WRITE_DATA_QUAD_CMD:
                spi_rx_dma_en();
            break;
        }
        spi_clr_irq_status( SPI_SLV_CMD_INT);//clr
    }
}

void user_init(void)
{
    WaitMs(1000);

    // led init
    gpio_set_func(WHITE_LED_PIN, AS_GPIO);
    gpio_set_output_en(WHITE_LED_PIN, 1);       // enable output
    gpio_set_input_en(WHITE_LED_PIN, 0);
    // disable input

    // spi init
    spi_set_pin(&spi_slave_pin_config);
    spi_slave_init(SPI_MODE0);
    spi_set_io_mode(SPI_SINGLE_MODE);           // SPI_3LINE_MODE can set by this function
    spi_set_rx_dma((unsigned char*)spi_slave_rx_buff);
    spi_set_tx_dma((unsigned char*)(spi_slave_rx_buff + DATA_BYTE_OFFSET));//rx_buff must have length information
    spi_set_irq_mask(SPI_SLV_CMD_EN);//endint_en txfifoint_en rxfifoint_en
    irq_enable_type(FLD_IRQ_SPI_EN);
    irq_enable();
}

int main()
{
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    wd_32k_stop();

	user_read_flash_value_calib();

    clock_init(SYS_CLK_24M_Crystal);

    gpio_init(0);

    user_init();

    while (1)
    {
        gpio_toggle(WHITE_LED_PIN);
        WaitMs(500);
    }
    return 0;
}

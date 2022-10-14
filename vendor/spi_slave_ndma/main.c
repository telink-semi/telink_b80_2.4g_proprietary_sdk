#include "driver.h"

#define WHITE_LED_PIN           GPIO_PA6
#define CLOCK_SYS_CLOCK_HZ      24000000
#define CLOCK_SPI               500000
#define SPI_RX_BUFF_LEN         24
#define DATA_BYTE_OFFSET        8 //must be a multiple 4

spi_pin_config_t spi_slave_pin_config = {
        .spi_clk_pin        = GPIO_PC0,
        .spi_csn_pin        = GPIO_PC4,
        .spi_mosi_io0_pin   = GPIO_PB2,
        .spi_miso_io1_pin   = GPIO_PB4,
};

volatile unsigned char spi_slave_rx_buff[SPI_RX_BUFF_LEN] __attribute__((aligned(4))) = {0x00};
volatile unsigned int spi_irq_rx_cnt = 0;
volatile unsigned int spi_tx_count = 0;
volatile unsigned int spi_tx_num = 16;

/* handle cmd  interrupt when received read cmd enable txfifo interrupt */
_attribute_ram_code_sec_ void spi_irq_slv_cmd_process(void)
{
    unsigned char spi_slave_cmd = spi_slave_get_cmd();
    switch (spi_slave_cmd)
    {
        case SPI_READ_DATA_SINGLE_CMD:
        case SPI_READ_DATA_DUAL_CMD:
        case SPI_READ_DATA_QUAD_CMD:
            spi_set_irq_mask(SPI_TXFIFO_INT_EN);//enable txfifo_int
        break;
    }
}

/*   handle txfifo  interrupt to return data that received from data master */
_attribute_ram_code_sec_ void spi_irq_tx_fifo_process(void)
{
    if ((spi_tx_num - spi_tx_count) >= 4)
    {
        spi_write_word((unsigned int*)(spi_slave_rx_buff + DATA_BYTE_OFFSET + spi_tx_count));
        spi_clr_irq_status(SPI_TXF_INT);//clr
        spi_tx_count += 4;
    }
    else
    {
        spi_clr_irq_mask(SPI_TXFIFO_INT_EN);//disable txfifo_in
        spi_write((unsigned char*)spi_slave_rx_buff + DATA_BYTE_OFFSET + spi_tx_count, 16 - spi_tx_count);

        // reset
        spi_tx_count = 0;
        spi_tx_num = 16;
    }
}

 /*  handle rxfifo  interrupt to  received data from  master */
_attribute_ram_code_sec_ void spi_irq_rx_fifo_process(void)
{
    if(spi_irq_rx_cnt < (SPI_RX_BUFF_LEN - (SPI_RX_BUFF_LEN % 4)))
    {
        spi_read_word((unsigned int*)(spi_slave_rx_buff + spi_irq_rx_cnt));
        spi_irq_rx_cnt += 4;
    }
    else
    {
        spi_read(spi_slave_rx_buff + spi_irq_rx_cnt, SPI_RX_BUFF_LEN % 4);
        spi_irq_rx_cnt += SPI_RX_BUFF_LEN % 4;
        spi_rx_fifo_clr();
    }
}

 /*  handle end  interrupt to  received remains data  from master */
_attribute_ram_code_sec_ void spi_irq_end_process(void)
{
    unsigned char rx_fifo_num = spi_get_rxfifo_num();

    if((rx_fifo_num != 0) && (spi_irq_rx_cnt < SPI_RX_BUFF_LEN))
    {
        spi_read((unsigned char*)spi_slave_rx_buff + spi_irq_rx_cnt, rx_fifo_num);
        spi_irq_rx_cnt += rx_fifo_num;
    }
    spi_tx_fifo_clr();
    spi_rx_fifo_clr();
    spi_irq_rx_cnt = 0;
}

_attribute_ram_code_sec_ void irq_handler(void)
{
    if (spi_get_irq_status(SPI_SLV_CMD_INT))
    {
        spi_irq_slv_cmd_process();
        spi_clr_irq_status(SPI_SLV_CMD_INT);
    }

    if (spi_get_irq_status(SPI_TXF_INT))
    {
        spi_irq_tx_fifo_process();
        spi_clr_irq_status(SPI_TXF_INT);
    }

    if (spi_get_irq_status(SPI_RXF_INT))
    {
        spi_irq_rx_fifo_process();
        spi_clr_irq_status(SPI_RXF_INT);
    }

    if (spi_get_irq_status(SPI_END_INT))
    {
        spi_irq_end_process();
        spi_clr_irq_status(SPI_RXF_INT);
        spi_clr_irq_status(SPI_END_INT);
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
    spi_set_irq_mask(SPI_SLV_CMD_EN | SPI_RXFIFO_INT_EN | SPI_END_INT_EN);//endint_en txfifoint_en rxfifoint_en
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

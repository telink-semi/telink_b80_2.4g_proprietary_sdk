#include "driver.h"

#define CLOCK_SYS_CLOCK_HZ      24000000
#define UART_TX_PIN_PD0         GPIO_PD0
#define UART_RX_PIN_PD1         GPIO_PD1
/* notes:
 * rec_buff_Len Set up rules:(Both conditions have to be met)
 * 1.The rec_buff_Len value is a multiple of 16     (16 * n)
 * 2.The length of the receive data is less than (16 * n - 4)
 */
#define MAX_DATA_LEN            11
#define UART_DMA_LEN            (((MAX_DATA_LEN + 4 + 16) / 16) * 16)

typedef struct{
    unsigned int dma_len;        // dma len must be 4 byte
    unsigned char data[UART_DMA_LEN];
}uart_data_t;

__attribute__((aligned(4))) uart_data_t recv_buff = {0, {0,}};
__attribute__((aligned(4))) uart_data_t trans_buff = {6, {0x11,0x22,0x33,0x44,0x55,0x66}};

volatile unsigned char uart_dma_rx_flag = 0;
volatile unsigned char uart_dma_tx_flag = 1;

volatile unsigned char uart_dmairq_tx_cnt = 0;
volatile unsigned char uart_dmairq_rx_cnt = 0;

_attribute_ram_code_sec_noinline_ void irq_handler(void)
{
    unsigned char uart_dma_irqsrc;
    uart_dma_irqsrc = dma_chn_irq_status_get();

    if (reg_uart_status1 & FLD_UART_TX_DONE)
    {
        uart_dmairq_tx_cnt++;
        uart_dma_tx_flag = 1;
        uart_clr_tx_done();
    }

    if (uart_dma_irqsrc & FLD_DMA_CHN_UART_RX)
    {
        dma_chn_irq_status_clr(FLD_DMA_CHN_UART_RX);
        uart_dmairq_rx_cnt++;
        uart_dma_rx_flag = 1;
    }
}

void user_init()
{
    WaitMs(2000); //leave enough time for SWS_reset when power on

    // note: dma addr must be set first before any other uart initialization!
    uart_recbuff_init((unsigned char *)&recv_buff, sizeof(recv_buff));

    uart_gpio_set(UART_TX_PIN_PD0, UART_RX_PIN_PD1);

    uart_reset();

    uart_init_baudrate(115200, CLOCK_SYS_CLOCK_HZ, PARITY_NONE, STOP_BIT_ONE);

    uart_dma_enable(1, 1);

    irq_enable_type(FLD_IRQ_DMA_EN);

    dma_chn_irq_enable(FLD_DMA_CHN_UART_RX, 1);

    uart_mask_tx_done_irq_enable();

    irq_enable_type(FLD_IRQ_UART_EN);

    irq_enable();
}

int main(void)
{
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    wd_32k_stop();

	user_read_flash_value_calib();

    clock_init(SYS_CLK_24M_Crystal);

    gpio_init(0);

    user_init();

    while (1)
    {
        if (uart_dma_rx_flag == 1)
        {
            uart_dma_rx_flag = 0;
            while (!uart_dma_tx_flag);
            uart_send_dma((unsigned char *)&recv_buff);
            uart_dma_tx_flag = 0;
        }

        if (uart_dma_tx_flag == 1)
        {
            uart_send_dma((unsigned char *)&trans_buff);
            uart_dma_tx_flag = 0;
        }
        WaitMs(500);
    }
    return 0;
}

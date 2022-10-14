#include "driver.h"

#define CLOCK_SYS_CLOCK_HZ      24000000
#define UART_TX_PIN_PD0         GPIO_PD0
#define UART_RX_PIN_PD1         GPIO_PD1

#define UART_RX_TRIG_LEVEL      1       //B85/B87/B89 can only be 1,B80 can be 1 or 4.
#define RECV_BUF_LEN            16
#define TRANS_BUF_LEN           16

volatile unsigned char uart_ndmairq_index = 0;
volatile unsigned int uart_ndmairq_cnt = 0;
volatile unsigned int uart_tx_cnt = 0;
volatile unsigned char uart_rx_flag = 0;

volatile unsigned int irq_cnt = 0;

__attribute__((aligned(4))) unsigned char recv_buff[RECV_BUF_LEN] = {0};
__attribute__((aligned(4)))
unsigned char trans_buff[TRANS_BUF_LEN] = { 0x11, 0x22, 0x33, 0x44,
                                            0x55, 0x66, 0x77, 0x88,
                                            0x99, 0xaa, 0xbb, 0xcc,
                                            0xdd, 0xee, 0xff, 0x00,
                                          };

_attribute_ram_code_sec_noinline_ void irq_handler(void)
{
    static unsigned char uart_ndma_irqsrc;
    uart_ndma_irqsrc = uart_ndmairq_get();
    //cycle the four registers 0x90 0x91 0x92 0x93, in addition reading will clear the irq.
    if (uart_ndma_irqsrc)
    {
        irq_cnt++;
        if (0 == uart_rx_flag)
        {
            recv_buff[uart_ndmairq_cnt++] = uart_ndma_read_byte();
            if ((uart_ndmairq_cnt % TRANS_BUF_LEN == 0) && (uart_ndmairq_cnt != 0))
            {
                uart_rx_flag = 1;
            }
        }
        else
        {
            uart_ndma_read_byte(); // clear irq
        }
    }
}

void user_init()
{
    WaitMs(1000); //leave enough time for SWS_reset when power on

    uart_gpio_set(UART_TX_PIN_PD0, UART_RX_PIN_PD1);

    uart_reset();

    uart_init_baudrate(115200, CLOCK_SYS_CLOCK_HZ, PARITY_NONE, STOP_BIT_ONE);

    uart_dma_enable(0, 0);

    irq_disable_type(FLD_IRQ_DMA_EN);

    dma_chn_irq_enable(FLD_DMA_CHN_UART_RX | FLD_DMA_CHN_UART_TX, 0);

    uart_irq_enable(1,0);   //uart RX irq enable

    uart_ndma_irq_triglevel(UART_RX_TRIG_LEVEL,0);   //set the trig level. 1 indicate one byte will occur interrupt

    irq_enable();
}

int main()
{
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    wd_32k_stop();

	user_read_flash_value_calib();

    clock_init(SYS_CLK_24M_Crystal);

    gpio_init(1);

    user_init();

    while (1)
    {
        WaitMs(1000);
        if (1 == uart_rx_flag)
        {
            uart_ndmairq_cnt = 0;
            uart_rx_flag = 0;
            for (int i = 0; i < TRANS_BUF_LEN; i++)
            {
                uart_ndma_send_byte(recv_buff[i]);
            }
        }
        for (int i = 0; i < TRANS_BUF_LEN; i++)
        {
            uart_ndma_send_byte(trans_buff[i]);
        }
    }
    return 0;
}

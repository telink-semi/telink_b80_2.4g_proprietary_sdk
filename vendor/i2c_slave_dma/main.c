#include "common.h"
#include "driver.h"

/*
 * Note:
 * 1. slave device id 0x5C(write) 0x5D(read)
 * 2. i2c slave mapping mode, master no need send any address when reading / writing,
 * 3. for i2c master:
 *     - writing data buffer is slave_mapping_buff,
 *     - reading data buffer is (slave_mapping_buff + 64)
 *     (this offset 64 is managed by MCU hardware, user can not change it)
 */

#define GREEN_LED_PIN                   GPIO_PB4
#define BLUE_LED_PIN                    GPIO_PB5
#define I2C_SDA_PIN                     GPIO_PA4
#define I2C_SCL_PIN                     GPIO_PA5

#define IIC_IDLE                        0
#define IIC_READ_SLAVE                  1
#define IIC_WRITE_SLAVE                 2
#define DBG_DATA_LEN                    16
#define DBG_DATA_NUM                    16

#define SLAVE_DMA_MODE_ADDR_WRITE       0x4A000  //i2c master write data to  0x4A000
#define SLAVE_DMA_MODE_ADDR_READ        0x4A000  //i2c master read data from 0x4A000

volatile unsigned int i2c_status_flag = IIC_IDLE;
unsigned int i2c_read_cnt = 0;
unsigned int i2c_write_cnt = 0;
unsigned char slave_rx_buff_debug[DBG_DATA_LEN *DBG_DATA_NUM] = {0};
unsigned char *slave_dma_rx_buff = NULL;
unsigned int slave_rx_index = 0;

_attribute_ram_code_sec_noinline_ void irq_handler(void)
{
    unsigned char  irq_status = reg_i2c_map_host_status;    //i2c slave can distinguish the operation host write or read.
    if (irq_status & FLD_HOST_CMD_IRQ)                      //both host write & read trigger this status
    {
        reg_i2c_map_host_status = irq_status;               //clear all irq status
        if(irq_status & FLD_HOST_READ_IRQ)                  // host read
        {
            i2c_read_cnt++;
            i2c_status_flag = IIC_READ_SLAVE;
        }
        else                                                // host write
        {
            i2c_write_cnt++;
            i2c_status_flag = IIC_WRITE_SLAVE;
        }
    }
}

void user_init()
{
    WaitMs(2000);  //leave enough time for SWS_reset when power on

    // led init
    gpio_set_func(GREEN_LED_PIN | BLUE_LED_PIN, AS_GPIO);
    gpio_set_output_en(GREEN_LED_PIN | BLUE_LED_PIN, 1); // enable ouput
    gpio_set_input_en(GREEN_LED_PIN | BLUE_LED_PIN, 0); // disable input
    gpio_write(GREEN_LED_PIN | BLUE_LED_PIN, 0);

    // i2c slave init
    i2c_gpio_set(I2C_SDA_PIN, I2C_SCL_PIN);// SDA/CK : C0/C1

    i2c_slave_init(0x5C, I2C_SLAVE_DMA, NULL);

    slave_dma_rx_buff = (unsigned char *)(REG_BASE_ADDR + SLAVE_DMA_MODE_ADDR_WRITE);

    irq_enable_type(FLD_IRQ_MIX_CMD_EN);

    irq_enable();
}

int main (void)
{
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    clock_init(SYS_CLK_24M_Crystal);

    gpio_init(1);

    user_init();

    while(1)
    {
        if (i2c_status_flag == IIC_READ_SLAVE)
        {
            i2c_status_flag = IIC_IDLE;

            gpio_toggle(GREEN_LED_PIN);
        }
        else if (i2c_status_flag == IIC_WRITE_SLAVE)
        {
            i2c_status_flag = IIC_IDLE;

            gpio_toggle(BLUE_LED_PIN);

            memcpy((unsigned char*)(slave_rx_buff_debug + slave_rx_index * DBG_DATA_LEN),
                    slave_dma_rx_buff, DBG_DATA_LEN);
            slave_rx_index = (slave_rx_index + 1) % DBG_DATA_NUM;
        }
    }

}

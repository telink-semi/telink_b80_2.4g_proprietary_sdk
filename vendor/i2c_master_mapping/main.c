#include "common.h"
#include "driver.h"

#define CLOCK_SYS_CLOCK_HZ      24000000
#define I2C_SDA_PIN             GPIO_PB5
#define I2C_SCL_PIN             GPIO_PB4
#define DBG_DATA_LEN		    16
#define DBG_DATA_NUM	        16

unsigned int master_rx_index = 0;
unsigned char master_rx_buff_debug[DBG_DATA_LEN * DBG_DATA_NUM];   //store i2c master reading data here to debug
volatile unsigned char i2c_master_rx_buff[DBG_DATA_LEN] = {0};
volatile unsigned char i2c_master_tx_buff[DBG_DATA_LEN] = {0x00, 0x11, 0x22, 0x33,
                                                           0x44, 0x55, 0x66, 0x77,
                                                           0x88, 0x99, 0xaa, 0xbb,
                                                           0xcc, 0xdd, 0xee, 0xff};
/*
 * Note:
 * slave device id 0x5C(write) 0x5D(read), transmission direction is contorlled by LSB
 * i2c clock 200K, only master need set i2c clock
 */
void user_init()
{
	WaitMs(2000);  //leave enough time for SWS_reset when power on

    i2c_gpio_set(I2C_SDA_PIN, I2C_SCL_PIN);

    i2c_master_init(0x5c, (unsigned char)(CLOCK_SYS_CLOCK_HZ / ( 4 * 200000)));
}

int main (void)
{
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    wd_32k_stop();

	user_read_flash_value_calib();

    clock_init(SYS_CLK_24M_Crystal);

    gpio_init(1);

    user_init();

	while(1)
	{
		i2c_master_tx_buff[0] += 1;
		//slave mapping mode, no need send address information, so address length should be 0
		i2c_write_series(0, 0, (unsigned char*)i2c_master_tx_buff, DBG_DATA_LEN);
		i2c_read_series(0, 0, (unsigned char*)i2c_master_rx_buff, DBG_DATA_LEN);

		memcpy((unsigned char*)(master_rx_buff_debug + master_rx_index * DBG_DATA_LEN),
		            (unsigned char*)i2c_master_rx_buff, DBG_DATA_LEN);
		master_rx_index = (master_rx_index + 1) % DBG_DATA_NUM;
	}
	return 0;
}



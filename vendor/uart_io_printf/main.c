#include "driver.h"
#include "common.h"
#include "myprintf.h"

#define TRANS_BUFF_LEN          16
__attribute__((aligned(4))) unsigned char trans_buff[TRANS_BUFF_LEN] = {0x11, 0x22, 0x33, 0x44,
                                                                        0x55, 0x66, 0x77, 0x88,
                                                                        0x99, 0xaa, 0xbb, 0xcc,
                                                                        0xdd, 0xee, 0xff, 0x00};
int main (void)
{
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    wd_32k_stop();

	user_read_flash_value_calib();

    clock_init(SYS_CLK_24M_Crystal);

    gpio_init(0);

    while (1)
    {
        array_printf(trans_buff, TRANS_BUFF_LEN);

        WaitMs(1000);
    }
}

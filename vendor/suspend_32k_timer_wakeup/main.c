#include "driver.h"

#define WHITE_LED_PIN                  GPIO_PA6
#define DEBUG_IO_PIN                   GPIO_PB0
#define SUSPEND_DURATION               500
#define MAX_SUSPEND_TIMES              8
#define SUSPEND_SLEEP_WAKEUP           1
#define SUSPEND_LONG_SLEEP_WAKEUP      2
#define PM_MODE                 SUSPEND_LONG_SLEEP_WAKEUP

volatile unsigned char suspend_times = 0;

_attribute_session_(".ram_code") static void gpio_high_z_config(void)
{
    //output disable
    reg_gpio_pa_oen = 0xff;
    reg_gpio_pb_oen = 0xff;
    reg_gpio_pc_oen = 0xff;
    reg_gpio_pd_oen = 0xff;
    //digital output set as 0
    reg_gpio_pa_out = 0x00;
    reg_gpio_pb_out = 0x00;
    reg_gpio_pc_out = 0x00;
    reg_gpio_pd_out = 0x00;
    //input disable
    reg_gpio_pa_ie = 0x00;
    reg_gpio_pb_ie = 0x00;
    analog_write(areg_gpio_pc_ie, 0);
    reg_gpio_pd_ie = 0x00;
}

static void debug_io_init(void)
{
    gpio_set_func(DEBUG_IO_PIN, AS_GPIO);
    gpio_set_output_en(DEBUG_IO_PIN, 1); //enable output
    gpio_set_input_en(DEBUG_IO_PIN, 0); //disable input

    gpio_set_func(WHITE_LED_PIN, AS_GPIO);
    gpio_set_output_en(WHITE_LED_PIN, 1);
    gpio_set_input_en(WHITE_LED_PIN, 0);
}

int main(void)
{
//    blc_pm_select_external_32k_crystal();
    blc_pm_select_internal_32k_crystal();

    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    wd_32k_stop();

	user_read_flash_value_calib();

    clock_init(SYS_CLK_24M_Crystal);

    while (suspend_times < MAX_SUSPEND_TIMES)
    {
        debug_io_init();
        gpio_write(DEBUG_IO_PIN, 1);
        gpio_write(WHITE_LED_PIN, 1);
        WaitMs(500);

        gpio_high_z_config();
        if(PM_MODE == SUSPEND_SLEEP_WAKEUP){
        cpu_sleep_wakeup(SUSPEND_MODE, PM_WAKEUP_TIMER,
        		ClockTime() + (SUSPEND_DURATION * CLOCK_16M_SYS_TIMER_CLK_1MS));
        }else{
        cpu_long_sleep_wakeup(SUSPEND_MODE, PM_WAKEUP_TIMER,
        		SUSPEND_DURATION*CLOCK_32K_SYS_TIMER_CLK_1MS);
        }
        suspend_times++;
    }

    // resume sws for debug
    gpio_set_input_en(GPIO_SWS, 1);

    while (1)
    {

    }
    return 0;
}

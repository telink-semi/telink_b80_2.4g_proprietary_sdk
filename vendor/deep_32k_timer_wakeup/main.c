#include "driver.h"

#define WHITE_LED_PIN                   GPIO_PA6
#define DEBUG_IO_PIN                    GPIO_PB0
#define DURATION                        500
#define MAX_TIMES                       8
#define DEEP_RET_WAKEUP                 1
#define DEEP_RET_LONG_WAKEUP            2
#define DEEP_WAKEUP                     3
#define DEEP_LONG_WAKEUP                4
#define PM_MODE                         DEEP_RET_LONG_WAKEUP

_attribute_session_(".retention_data") volatile static unsigned char deep_retention_times = 0;
volatile static unsigned char deep_no_retention_times = 0;

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
//    blc_pm_select_external_32k_crystal(); // it has some problems now
    blc_pm_select_internal_32k_crystal();

    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    wd_32k_stop();

    user_read_flash_value_calib();

    clock_init(SYS_CLK_24M_Crystal);

    debug_io_init();

    if (deep_retention_times == MAX_TIMES)
    {
        //resume the SWS for debug
        gpio_set_input_en(GPIO_SWS, 1);
        while (1)
        {
            WaitMs(200);
            gpio_toggle(WHITE_LED_PIN);
        }
    }

    gpio_set_func(DEBUG_IO_PIN, AS_GPIO);
    gpio_set_output_en(DEBUG_IO_PIN, 1);
    gpio_write(DEBUG_IO_PIN, 1);
    WaitMs(10);

    deep_retention_times++;
    deep_no_retention_times++;

    gpio_high_z_config();
    if(PM_MODE == DEEP_RET_WAKEUP){
    	cpu_sleep_wakeup(DEEPSLEEP_MODE_RET_SRAM_LOW16K, PM_WAKEUP_TIMER,
    			ClockTime() + (DURATION * CLOCK_16M_SYS_TIMER_CLK_1MS));
    }else if(PM_MODE == DEEP_RET_LONG_WAKEUP){
    	cpu_long_sleep_wakeup(DEEPSLEEP_MODE_RET_SRAM_LOW16K, PM_WAKEUP_TIMER,
    			DURATION*CLOCK_32K_SYS_TIMER_CLK_1MS);
    }else if(PM_MODE == DEEP_WAKEUP){
    	cpu_sleep_wakeup(DEEPSLEEP_MODE, PM_WAKEUP_TIMER,
    			(clock_time() + DURATION*CLOCK_16M_SYS_TIMER_CLK_1MS));
    }else{
    	cpu_long_sleep_wakeup(DEEPSLEEP_MODE, PM_WAKEUP_TIMER,
    			DURATION*CLOCK_32K_SYS_TIMER_CLK_1MS);
    }


    while (1)
    {

    }
    return 0;
}

#include "driver.h"

#define WAKEUP_PIN          GPIO_PF0    //SW7
#define WHITE_LED_PIN       GPIO_PA6

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

void user_init(void)
{
    gpio_set_output_en(WAKEUP_PIN, 0); //disable output
    gpio_set_input_en(WAKEUP_PIN, 0); //disable input
    gpio_setup_up_down_resistor(WAKEUP_PIN, PM_PIN_PULLUP_1M); //enable internal 1M pull-up
    cpu_set_gpio_wakeup(WAKEUP_PIN, Level_Low, 1); //config low-level wakeup
}

int main(void)
{
	/* Caution: if wake-up source is only gpio, 32K clock source MUST be 32K RC * */
    blc_pm_select_internal_32k_crystal();

    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    wd_32k_stop();

	user_read_flash_value_calib();

    clock_init(SYS_CLK_24M_Crystal);

    gpio_init(0);

    user_init();

    while (1)
    {
        gpio_high_z_config();    //set all GPIO as high_Z state, avoiding current leakage
        cpu_sleep_wakeup(SUSPEND_MODE, PM_WAKEUP_PAD, 0);

        //led pin config
        gpio_set_func(WHITE_LED_PIN, AS_GPIO);
        gpio_set_output_en(WHITE_LED_PIN, 1);
        gpio_set_input_en(WHITE_LED_PIN, 0);
        gpio_write(WHITE_LED_PIN, 1);
        WaitMs(3000);
    }
    return 0;
}

#include "driver.h"

#define WAKEUP_PIN          GPIO_PF0    //SW7
#define WHITE_LED_PIN       GPIO_PB5

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
    // wakeup pad init
    gpio_set_output_en(WAKEUP_PIN, 0); //disable output
    gpio_set_input_en(WAKEUP_PIN, 0); //disable input
    gpio_setup_up_down_resistor(WAKEUP_PIN, PM_PIN_PULLUP_1M); //enable internal 1M pull-up
    cpu_set_gpio_wakeup(WAKEUP_PIN, Level_Low, 1); //config low-level wakeup

    // led init
    gpio_set_func(WHITE_LED_PIN, AS_GPIO);
    gpio_set_output_en(WHITE_LED_PIN, 1);
    gpio_set_input_en(WHITE_LED_PIN, 0);
}

int main(void)
{
//    blc_pm_select_external_32k_crystal();
    blc_pm_select_internal_32k_crystal();

    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    clock_init(SYS_CLK_24M_Crystal);

    gpio_init(0);

    user_init();

    gpio_write(WHITE_LED_PIN, 1);
    WaitMs(3000);

    gpio_high_z_config();
    cpu_sleep_wakeup(DEEPSLEEP_MODE, PM_WAKEUP_PAD, 0);

    while (1)
    {

    }
    return 0;
}

#include "driver.h"


#define GREEN_LED_PIN           GPIO_PA5
#define RED_LED_PIN             GPIO_PA7

#define CLOCK_SYS_CLOCK_HZ      24000000
enum {
    CLOCK_SYS_CLOCK_1S  = CLOCK_SYS_CLOCK_HZ,
    CLOCK_SYS_CLOCK_1MS = (CLOCK_SYS_CLOCK_1S / 1000),
    CLOCK_SYS_CLOCK_1US = (CLOCK_SYS_CLOCK_1S / 1000000),
};

volatile unsigned int t0;

void user_init(void)
{
    WaitMs(1000);  //leave enough time for SWS_reset when power on


    // led init
    gpio_set_func(GREEN_LED_PIN, AS_GPIO);
    gpio_set_output_en(GREEN_LED_PIN, 1);         //enable output
    gpio_set_input_en(GREEN_LED_PIN, 0);          //disable input
    gpio_write(GREEN_LED_PIN, 0);                 //LED On

    gpio_set_func(RED_LED_PIN, AS_GPIO);
    gpio_set_output_en(RED_LED_PIN, 1);         //enable output
    gpio_set_input_en(RED_LED_PIN, 0);          //disable input
    gpio_write(RED_LED_PIN, 0);                 //LED On

    // watchdog init
    wd_set_interval_ms(3000, CLOCK_SYS_CLOCK_1MS);
    wd_start();
}

int main(void)
{
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    wd_32k_stop();

	user_read_flash_value_calib();

    clock_init(SYS_CLK_24M_Crystal);

    gpio_init(1);

    user_init();

    gpio_toggle(RED_LED_PIN);
    WaitMs(1000);
    gpio_toggle(RED_LED_PIN);

    while (1)
    {
    	t0= clock_time();
		while(!clock_time_exceed(t0,500000));//500ms
		wd_clear();
//		wd_stop();
		gpio_toggle(GREEN_LED_PIN);
    }
    return 0;
}


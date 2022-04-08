#include "driver.h"

#define GREEN_LED_PIN           GPIO_PB4
#define CLOCK_SYS_CLOCK_HZ      24000000

enum {
    CLOCK_SYS_CLOCK_1S  = CLOCK_SYS_CLOCK_HZ,
    CLOCK_SYS_CLOCK_1MS = (CLOCK_SYS_CLOCK_1S / 1000),
    CLOCK_SYS_CLOCK_1US = (CLOCK_SYS_CLOCK_1S / 1000000),
};

volatile unsigned int timer2_tick_value = 0;

void user_init(void)
{
    WaitMs(1000);  //leave enough time for SWS_reset when power on
    // led init
    gpio_set_func(GREEN_LED_PIN, AS_GPIO);
    gpio_set_output_en(GREEN_LED_PIN, 1);         //enable output
    gpio_set_input_en(GREEN_LED_PIN, 0);          //disable input
    gpio_write(GREEN_LED_PIN, 0);                 //LED On

    // timer init
    timer2_set_mode(TIMER_MODE_TICK, 0, 0);
    timer_start(TIMER2);
}

int main(void)
{
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    clock_init(SYS_CLK_24M_Crystal);

    gpio_init(1);

    user_init();

    while (1)
    {
        timer2_tick_value = reg_tmr2_tick;
        if (timer2_tick_value >= (500 * CLOCK_SYS_CLOCK_1US * 1000))
        {
            reg_tmr2_tick = 0;
            gpio_toggle(GREEN_LED_PIN);
        }
	}

    return 0;
}


#include "driver.h"

#define DEBUG_PIN_SW7           GPIO_PF0
#define GREEN_LED_PIN           GPIO_PB4
#define RED_LED_PIN             GPIO_PB6

#define CLOCK_SYS_CLOCK_HZ      24000000
enum {
    CLOCK_SYS_CLOCK_1S  = CLOCK_SYS_CLOCK_HZ,
    CLOCK_SYS_CLOCK_1MS = (CLOCK_SYS_CLOCK_1S / 1000),
    CLOCK_SYS_CLOCK_1US = (CLOCK_SYS_CLOCK_1S / 1000000),
};


void user_init(void)
{
    WaitMs(1000);  //leave enough time for SWS_reset when power on

    // debug io init
    gpio_set_func(DEBUG_PIN_SW7, AS_GPIO);
    gpio_set_output_en(DEBUG_PIN_SW7, 0);         //disable output
    gpio_set_input_en(DEBUG_PIN_SW7, 1);          //enable input
    gpio_setup_up_down_resistor(DEBUG_PIN_SW7, PM_PIN_PULLUP_10K);

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

    clock_init(SYS_CLK_24M_Crystal);

    gpio_init(1);

    user_init();

    gpio_toggle(RED_LED_PIN);
    WaitMs(1000);
    gpio_toggle(RED_LED_PIN);

    while (1)
    {
        if (0 == gpio_read(DEBUG_PIN_SW7))
        {
            WaitMs(10);
            if (0 == gpio_read(DEBUG_PIN_SW7))
            {
                while (0 == gpio_read(DEBUG_PIN_SW7));
//                wd_clear(); // feed watchdog
                wd_stop();  // disable watchdog
            }
        }
        WaitMs(100);
        gpio_toggle(GREEN_LED_PIN);
    }
    return 0;
}


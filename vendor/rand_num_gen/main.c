#include "driver.h"

#define GREEN_LED_PIN           GPIO_PB4
#define RANDOM_NUM_COUNT        32
volatile unsigned int randnum_arr[RANDOM_NUM_COUNT];

void user_init(void)
{
    gpio_set_func(GREEN_LED_PIN, AS_GPIO);
    gpio_set_output_en(GREEN_LED_PIN, 1);         //enable output
    gpio_set_input_en(GREEN_LED_PIN, 0);          //disable input
    gpio_write(GREEN_LED_PIN, 0);                 //LED On
}

int main(void)
{
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    clock_init(SYS_CLK_24M_Crystal);

    user_init();

    //RNG module initialize
    random_generator_init();

    for (int i = 0; i < RANDOM_NUM_COUNT; i++)
    {
        randnum_arr[i] = rand();
    }

    while (1)
    {
        gpio_toggle(GREEN_LED_PIN);
        WaitMs(500);
	}

    return 0;
}


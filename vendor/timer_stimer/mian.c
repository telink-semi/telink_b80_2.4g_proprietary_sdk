#include "driver.h"

#define GREEN_LED_PIN           GPIO_PA5
#define CLOCK_SYS_CLOCK_HZ      24000000

int stimer_irq_cnt = 0;

_attribute_ram_code_sec_noinline_ void irq_handler(void)
{
	if(stimer_get_irq_status())
		{
			stimer_clr_irq_status();            			//clear irq status
			stimer_set_capture_tick(clock_time() + CLOCK_16M_SYS_TIMER_CLK_1S);
			stimer_irq_cnt++;
			gpio_toggle(GREEN_LED_PIN);
		}
}

void user_init(void)
{
    WaitMs(1000);  //leave enough time for SWS_reset when power on
    // led init
    gpio_set_func(GREEN_LED_PIN, AS_GPIO);
    gpio_set_output_en(GREEN_LED_PIN, 1);         //enable output
    gpio_set_input_en(GREEN_LED_PIN, 0);          //disable input
    gpio_write(GREEN_LED_PIN, 0);                 //LED On

    // timer init
	stimer_set_irq_mask();
	stimer_set_capture_tick(clock_time() + CLOCK_16M_SYS_TIMER_CLK_1S);
	stimer_enable();
	irq_enable();
}

int main(void)
{
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    wd_32k_stop();

	user_read_flash_value_calib();

    clock_init(SYS_CLK_24M_Crystal);

    gpio_init(1);

    user_init();

    while (1)
    {

	}

    return 0;
}

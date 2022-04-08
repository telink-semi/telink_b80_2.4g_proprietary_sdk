#include "driver.h"
#define DEBUG_IO_PIN            GPIO_PB0
volatile unsigned int systimer_irq_cnt = 0;


_attribute_ram_code_sec_noinline_ void irq_handler(void)
{
    if (reg_irq_src & FLD_IRQ_SYSTEM_TIMER)
    {
        reg_irq_src = FLD_IRQ_SYSTEM_TIMER;
        gpio_toggle(DEBUG_IO_PIN);
        systimer_irq_cnt++;
        reg_system_tick_irq_level = clock_time() + (500 * CLOCK_16M_SYS_TIMER_CLK_1US);
    }
}

void user_init(void)
{
    gpio_set_func(DEBUG_IO_PIN, AS_GPIO);
    gpio_set_output_en(DEBUG_IO_PIN, 1); //enable output
    gpio_set_input_en(DEBUG_IO_PIN, 0); //disable input
    gpio_write(DEBUG_IO_PIN, 1);

    irq_enable_type(FLD_IRQ_SYSTEM_TIMER);
    reg_system_tick_irq_level = clock_time() + (500 * CLOCK_16M_SYS_TIMER_CLK_1US);
    irq_enable();
}

int main(void)
{
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    clock_init(SYS_CLK_24M_Crystal);

    user_init();

    while (1)
    {

	}

    return 0;
}


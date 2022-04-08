#include "driver.h"

#define GREEN_LED_PIN     		GPIO_PB4
#define GPIO_IRQ_PIN			GPIO_PF0	//SW7

volatile unsigned char gpio_irq_cnt = 0;
volatile unsigned char gpio_irq_flag = 0;

_attribute_ram_code_sec_noinline_ void irq_handler(void)
{
    unsigned int irq_src = irq_get_src();
	if (irq_src & FLD_IRQ_GPIO_EN)
	{
	    if (0 == gpio_read(GPIO_IRQ_PIN))
	    {
	        WaitMs(10);
	        if (0 == gpio_read(GPIO_IRQ_PIN))
	        {
	            while (0 == gpio_read(GPIO_IRQ_PIN));
	            gpio_irq_flag = 1;
	            gpio_irq_cnt++;
	        }
	    }
	}
	irq_clr_src2(FLD_IRQ_GPIO_EN);
}


void user_init(void)
{
    gpio_set_func(GREEN_LED_PIN, AS_GPIO);
    gpio_set_output_en(GREEN_LED_PIN, 1);         //enable output
    gpio_set_input_en(GREEN_LED_PIN, 0);          //disable input
    gpio_write(GREEN_LED_PIN, 0);                 //LED On

    gpio_set_func(GPIO_IRQ_PIN, AS_GPIO);
    gpio_set_output_en(GPIO_IRQ_PIN, 0); 			// disable output
    gpio_set_input_en(GPIO_IRQ_PIN, 1);				// enable input
    gpio_setup_up_down_resistor(GPIO_IRQ_PIN, PM_PIN_PULLUP_10K);
    gpio_set_interrupt(GPIO_IRQ_PIN, POL_FALLING);

    irq_enable_type(FLD_IRQ_GPIO_EN);
    irq_enable();
}

int main(void)
{
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    clock_init(SYS_CLK_24M_Crystal);

    user_init();

    while (1)
    {
        if (gpio_irq_flag == 1)
        {
            gpio_irq_flag = 0;
            gpio_toggle(GREEN_LED_PIN);
        }
    }
	return 0;
}


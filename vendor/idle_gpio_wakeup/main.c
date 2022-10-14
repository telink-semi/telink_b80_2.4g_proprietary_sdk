#include "driver.h"

#define WAKEUP_PAD				GPIO_PB1
#define BLUE_LED_PIN     		GPIO_PA4
#define GREEN_LED_PIN     		GPIO_PA5

_attribute_ram_code_sec_noinline_ void irq_handler(void)
{
	if((reg_irq_src & FLD_IRQ_GPIO_EN) == FLD_IRQ_GPIO_EN)
	{
		reg_irq_src |= FLD_IRQ_GPIO_EN;
		gpio_toggle(GREEN_LED_PIN);
	}
	gpio_clr_irq_status(GPIO_IRQ_MASK_GPIO);//Clear the interrupt status.
}

void user_init()
{
	sleep_ms(1000);

	gpio_set_func(BLUE_LED_PIN ,AS_GPIO);
	gpio_set_output_en(BLUE_LED_PIN, 1); 		//enable output
	gpio_set_input_en(BLUE_LED_PIN ,0);			//disable input
	gpio_write(BLUE_LED_PIN, 0);              	//LED On
	gpio_set_func(GREEN_LED_PIN,AS_GPIO);
	gpio_set_output_en(GREEN_LED_PIN, 1); 		//enable output
	gpio_set_input_en(GREEN_LED_PIN,0);			//disable input
	gpio_write(GREEN_LED_PIN, 0);              	//LED On

	//After stall wakes up, it does not enter the interrupt, but only continues to execute, and the interrupt mask needs to be cleared.
	gpio_set_func(WAKEUP_PAD ,AS_GPIO);
	gpio_set_output_en(WAKEUP_PAD, 0);
	gpio_set_input_en(WAKEUP_PAD ,1);
	gpio_setup_up_down_resistor(WAKEUP_PAD, PM_PIN_PULLUP_10K);
	gpio_set_interrupt(WAKEUP_PAD, POL_FALLING);
	irq_enable();		//Turn on the total interrupt.
}

int main(void)
{
	cpu_wakeup_init(EXTERNAL_XTAL_24M);

    wd_32k_stop();

	user_read_flash_value_calib();

	clock_init(SYS_CLK_24M_Crystal);

	gpio_init(0);

	user_init();

	while(1)
	{
		cpu_stall_wakeup(FLD_IRQ_GPIO_EN);

		gpio_write(BLUE_LED_PIN, 1);
		sleep_ms(100);
		gpio_write(BLUE_LED_PIN, 0);
		sleep_ms(100);
	}
	return 0;

}

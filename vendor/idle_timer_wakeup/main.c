#include "driver.h"

#define BLUE_LED_PIN     		        GPIO_PA4
#define IDLE_TIMER_WAKEUP				1
#define IDLE_STIMER_WAKEUP				2
#define PM_MODE			     			IDLE_STIMER_WAKEUP
#define CLOCK_SYS_CLOCK_HZ  	24000000

enum{
	CLOCK_SYS_CLOCK_1S = CLOCK_SYS_CLOCK_HZ,				///< system tick per 1 second
	CLOCK_SYS_CLOCK_1MS = (CLOCK_SYS_CLOCK_1S / 1000),		///< system tick per 1 millisecond
	CLOCK_SYS_CLOCK_1US = (CLOCK_SYS_CLOCK_1S / 1000000),   ///< system tick per 1 microsecond
};

void user_init()
{
	sleep_ms(1000);

	gpio_set_func(BLUE_LED_PIN ,AS_GPIO);
	gpio_set_output_en(BLUE_LED_PIN, 1); 		//enable output
	gpio_set_input_en(BLUE_LED_PIN ,0);			//disable input
	gpio_write(BLUE_LED_PIN, 0);              	//LED On

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
	//After stall wakes up, it does not enter the interrupt, but only continues to execute, and the interrupt mask needs to be cleared.
		if(PM_MODE == IDLE_TIMER_WAKEUP){
			timer0_set_mode(TIMER_MODE_SYSCLK, 0, 2*CLOCK_SYS_CLOCK_1S);
			timer_clr_irq_mask(TMR_STA_TMR0);		//Clear the interrupt mask.
			timer_start(TIMER0);
			cpu_stall_wakeup(FLD_IRQ_TMR0_EN);
			timer_clear_interrupt_status(FLD_TMR_STA_TMR0);	//Clear the interrupt status.
			timer_stop(TIMER0);
		}else if(PM_MODE == IDLE_STIMER_WAKEUP){
			stimer_set_capture_tick(clock_time() + CLOCK_16M_SYS_TIMER_CLK_1S);
			stimer_enable();
			cpu_stall_wakeup(FLD_IRQ_SYSTEM_TIMER);
			stimer_clr_irq_status();				//Clear the interrupt status.
			stimer_disable();
		}

		gpio_write(BLUE_LED_PIN, 1);
		sleep_ms(100);
		gpio_write(BLUE_LED_PIN, 0);
		sleep_ms(100);
	}
	return 0;

}

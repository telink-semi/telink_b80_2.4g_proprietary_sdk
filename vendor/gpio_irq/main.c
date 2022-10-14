#include "driver.h"

#define GREEN_LED_PIN     		GPIO_PA5
#define IRQ_PIN           		GPIO_PF0
#define GPIO_IRQ				1
#define GPIO_IRQ_RSIC0			2
#define GPIO_IRQ_RSIC1			3
#define GPIO_HIGH_RESISTOR		4
#define GPIO_SEL_IRQ_SRC        5
#define SET_GROUP        		GPIO_GROUP_F
#define SET_GROUP_GPIO    		IRQ_PIN

#define GPIO_MODE 				GPIO_SEL_IRQ_SRC

volatile unsigned int gpio_irq_cnt;
volatile unsigned int gpio_set_irq_cnt;
volatile unsigned char gpio_irq_flag = 0;

_attribute_ram_code_sec_noinline_ void irq_handler(void)
{
#if (GPIO_MODE == GPIO_IRQ )
    unsigned int irq_src = irq_get_src();
	if (irq_src & FLD_IRQ_GPIO_EN)
	{
		reg_irq_src |= FLD_IRQ_GPIO_EN; // clear the relevant irq
	    if (0 == gpio_read(IRQ_PIN))
	    {
	        WaitMs(10);
	        if (0 == gpio_read(IRQ_PIN))
	        {
	            while (0 == gpio_read(IRQ_PIN));
	            gpio_irq_cnt++;
	            gpio_toggle(GREEN_LED_PIN);
	        }
	    }
	}
#elif(GPIO_MODE == GPIO_IRQ_RSIC0)

	if((reg_irq_src & FLD_IRQ_GPIO_RISC0_EN)==FLD_IRQ_GPIO_RISC0_EN){
		reg_irq_src |= FLD_IRQ_GPIO_RISC0_EN; // clear the relevant irq
		gpio_irq_cnt++;
		gpio_toggle(GREEN_LED_PIN);
	}
#elif(GPIO_MODE == GPIO_IRQ_RSIC1)

	if((reg_irq_src & FLD_IRQ_GPIO_RISC1_EN)==FLD_IRQ_GPIO_RISC1_EN){
		reg_irq_src |= FLD_IRQ_GPIO_RISC1_EN; // clear the relevant irq
		gpio_irq_cnt++;
		gpio_toggle(GREEN_LED_PIN);

	}
#elif(GPIO_MODE == GPIO_SEL_IRQ_SRC)
	static unsigned char gpio_irqsrc;
	gpio_irqsrc = (reg_gpio_irq_from_pad & IRQ_PIN);
	if(gpio_irqsrc)
	{
		reg_gpio_irq_from_pad |= IRQ_PIN;
		gpio_irq_cnt++;
		gpio_toggle(GREEN_LED_PIN);
	}
#endif
}


void user_init(void)
{
#if(GPIO_MODE == GPIO_HIGH_RESISTOR)
	gpio_shutdown(GPIO_ALL);				//set all gpio as high resistor except sws and mspi
#else
    gpio_set_func(GREEN_LED_PIN, AS_GPIO);
    gpio_set_output_en(GREEN_LED_PIN, 1);         //enable output
    gpio_set_input_en(GREEN_LED_PIN, 0);          //disable input
    gpio_write(GREEN_LED_PIN, 0);                 //LED On

    /****  IRQ pin link PA0, PA0 produces a square wave.  **/
	gpio_set_func(GPIO_PA0 ,AS_GPIO);
	gpio_set_output_en(GPIO_PA0, 1); 		//enable output
	gpio_set_input_en(GPIO_PA0 ,0);			//disable input
	gpio_write(GPIO_PA0, 0);

	//2.init the IRQ pin, for trigger interrupt
	gpio_set_func(IRQ_PIN ,AS_GPIO);
	gpio_set_output_en(IRQ_PIN, 0); 			// disable output
	gpio_set_input_en(IRQ_PIN ,1);				// enable input


#if (GPIO_MODE == GPIO_IRQ )
	gpio_setup_up_down_resistor(IRQ_PIN, PM_PIN_PULLUP_10K);
	/****GPIO_IRQ POL_FALLING   Press SW2 to connect KEY1 and KEY3 to trigger an interrupt. **/
	gpio_set_interrupt(IRQ_PIN, POL_FALLING);	//When SW2 is pressed, the falling edge triggers the interrupt.
	irq_enable();
#elif(GPIO_MODE == GPIO_IRQ_RSIC0)
	gpio_setup_up_down_resistor(IRQ_PIN, PM_PIN_PULLDOWN_100K);
	/****GPIO_IRQ_RSIC0  POL_RISING   toggle PA0 to trigger an interrupt. **/
	gpio_set_interrupt_risc0(IRQ_PIN, POL_RISING);	//When switching PA0, the rising edge triggers an interrupt.
	irq_enable();

#elif(GPIO_MODE == GPIO_IRQ_RSIC1)
	gpio_setup_up_down_resistor(IRQ_PIN, PM_PIN_PULLUP_10K);
	/****GPIO_IRQ_RSIC1  POL_FALLING   toggle PA0 to trigger an interrupt. **/
	gpio_set_interrupt_risc1(IRQ_PIN, POL_FALLING);	//When switching PA0, the falling edge triggers an interrupt.
	irq_enable();

#elif(GPIO_MODE == GPIO_SEL_IRQ_SRC)
	/*Note : the use method of 8 new GPIO irq source :
	 * First ,you can choose a gpio group,like gpio GPIO_GROUP_A,8 GPIO in all.
	 * Second,the 8 gpio you choose corresponding to the 8 gpio irq source respectively.for example,irq source0-GPIO_PA0,irq source1-GPIO-PA1......
	 * Attention:Once you choose a gpio-group,you can only use the gpio as irq_source in this group.And you should obey the rule of correspondence.
	 */
	gpio_setup_up_down_resistor(SET_GROUP_GPIO, PM_PIN_PULLUP_10K);
	gpio_set_src_irq_group(SET_GROUP);
	gpio_set_src_irq(SET_GROUP_GPIO, SRC_IRQ_FALLING_EDGE);	//When switching PA0, the falling edge triggers an interrupt.
	irq_enable();
#endif
#endif
}

int main(void)
{
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    wd_32k_stop();

	user_read_flash_value_calib();

    clock_init(SYS_CLK_24M_Crystal);

    user_init();

    while (1)
    {
    	   sleep_ms(10);
    	   gpio_toggle(GPIO_PA0);
    }
	return 0;
}


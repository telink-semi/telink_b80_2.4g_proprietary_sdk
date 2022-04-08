#include "driver.h"

#define LED1                    GPIO_PB3
#define LED2                    GPIO_PB4
#define LED3                    GPIO_PB5
#define LED4                    GPIO_PB6

void user_init(void)
{
    gpio_set_func(LED1|LED2|LED3|LED4 ,AS_GPIO);
    gpio_set_output_en(LED1|LED2|LED3|LED4, 1); 		//enable output
    gpio_set_input_en(LED1|LED2|LED3|LED4 ,0);			//disable input
    gpio_write(LED1|LED2|LED3|LED4, 0);              	//LED On
}

int main(void)
{
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    clock_init(SYS_CLK_24M_Crystal);

    user_init();

    while (1)
    {
        gpio_toggle(LED1|LED2|LED3|LED4);
        WaitMs(1000);
	}

    return 0;
}


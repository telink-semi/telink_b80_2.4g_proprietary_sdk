#include "driver.h"

#define GREEN_LED_PIN           GPIO_PB4
#define WHITE_LED_PIN           GPIO_PB5
#define RED_LED_PIN             GPIO_PB6
#define CLOCK_SYS_CLOCK_HZ      24000000
enum {
    CLOCK_SYS_CLOCK_1S  = CLOCK_SYS_CLOCK_HZ,
    CLOCK_SYS_CLOCK_1MS = (CLOCK_SYS_CLOCK_1S / 1000),
    CLOCK_SYS_CLOCK_1US = (CLOCK_SYS_CLOCK_1S / 1000000),
};

volatile unsigned char timer0_expire_flg = 0;
volatile unsigned char timer1_expire_flg = 0;
volatile unsigned char timer2_expire_flg = 0;

unsigned int timer0_irq_cnt = 0;
unsigned int timer1_irq_cnt = 0;
unsigned int timer2_irq_cnt = 0;

_attribute_ram_code_sec_noinline_ void irq_handler(void)
{
    if (timer_get_interrupt_status(FLD_TMR_STA_TMR0))
    {
        timer_clear_interrupt_status(FLD_TMR_STA_TMR0);
        timer0_irq_cnt++;
        timer0_expire_flg = 1;
    }

    if (timer_get_interrupt_status(FLD_TMR_STA_TMR1))
    {
        timer_clear_interrupt_status(FLD_TMR_STA_TMR1);
        timer1_irq_cnt++;
        timer1_expire_flg = 1;
    }

    if (timer_get_interrupt_status(FLD_TMR_STA_TMR2))
    {
        timer_clear_interrupt_status(FLD_TMR_STA_TMR2);
        timer2_irq_cnt++;
        timer2_expire_flg = 1;
    }
}

void user_init(void)
{
    WaitMs(1000);  //leave enough time for SWS_reset when power on
    // led init
    gpio_set_output_en(GREEN_LED_PIN|WHITE_LED_PIN|RED_LED_PIN, 1); //enable output
    gpio_set_input_en(GREEN_LED_PIN|WHITE_LED_PIN|RED_LED_PIN, 0); //disable input
    gpio_write(GREEN_LED_PIN|WHITE_LED_PIN|RED_LED_PIN, 0);

    // timer init
    timer0_set_mode(TIMER_MODE_SYSCLK, 0, 500 * CLOCK_SYS_CLOCK_1MS);
    timer_start(TIMER0);

    timer1_set_mode(TIMER_MODE_SYSCLK, 0, 1000 * CLOCK_SYS_CLOCK_1MS);
    timer_start(TIMER1);

    timer2_set_mode(TIMER_MODE_SYSCLK, 0, 2000 * CLOCK_SYS_CLOCK_1MS);
    timer_start(TIMER2);

    irq_enable();
}

int main(void)
{
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    clock_init(SYS_CLK_24M_Crystal);

    gpio_init(1);

    user_init();

    while (1)
    {
        if (timer0_expire_flg)
        {
            timer0_expire_flg = 0;
            gpio_toggle(GREEN_LED_PIN);
        }

        if (timer1_expire_flg)
        {
            timer1_expire_flg = 0;
            gpio_toggle(WHITE_LED_PIN);
        }

        if (timer2_expire_flg)
        {
            timer2_expire_flg = 0;
            gpio_toggle(RED_LED_PIN);
        }
	}
    return 0;
}


#include "driver.h"

#define GREEN_LED_PIN           GPIO_PB4
#define WHITE_LED_PIN           GPIO_PB5
#define RED_LED_PIN             GPIO_PB6
#define SAMPLE_INTPUT_PIN       GPIO_PC7
#define PULSE_OUTPUT_PIN        GPIO_PB2

unsigned int timer0_irq_cnt = 0;
unsigned int timer1_irq_cnt = 0;
unsigned int timer2_irq_cnt = 0;

volatile unsigned char timer0_expire_flg = 0;
volatile unsigned char timer1_expire_flg = 0;
volatile unsigned char timer2_expire_flg = 0;
volatile unsigned int timer_pulse_width_tick_value = 0;

_attribute_ram_code_sec_noinline_ void irq_handler(void)
{
    if (reg_tmr_sta & FLD_TMR_STA_TMR0)
    {
        reg_tmr_sta = FLD_TMR_STA_TMR0; //clear irq status
        timer0_irq_cnt++;
        timer0_expire_flg = 1;
    }

    if (reg_tmr_sta & FLD_TMR_STA_TMR1)
    {
        reg_tmr_sta = FLD_TMR_STA_TMR1; //clear irq status
        timer1_irq_cnt++;
        timer1_expire_flg = 1;
    }

    if (reg_tmr_sta & FLD_TMR_STA_TMR2)
    {
        reg_tmr_sta = FLD_TMR_STA_TMR2; //clear irq status
        timer2_irq_cnt++;
        timer2_expire_flg = 1;
        timer_pulse_width_tick_value = reg_tmr2_tick;
        reg_tmr2_tick = 0;
    }
}

void user_init(void)
{
    WaitMs(1000);

    // led ouput pin init
    gpio_set_func(GREEN_LED_PIN|WHITE_LED_PIN|RED_LED_PIN|PULSE_OUTPUT_PIN, AS_GPIO);
    gpio_set_output_en(GREEN_LED_PIN|WHITE_LED_PIN|RED_LED_PIN|PULSE_OUTPUT_PIN, 1); //enable output
    gpio_set_input_en(GREEN_LED_PIN|WHITE_LED_PIN|RED_LED_PIN|PULSE_OUTPUT_PIN, 0); //disable input
    gpio_write(GREEN_LED_PIN|WHITE_LED_PIN|RED_LED_PIN|PULSE_OUTPUT_PIN, 0);

    // timer init
    timer2_gpio_init(SAMPLE_INTPUT_PIN, POL_FALLING);
    timer2_set_mode(TIMER_MODE_GPIO_WIDTH, 0, 0);
    timer_start(TIMER2);
    irq_enable();
}

_attribute_ram_code_sec_noinline_ int main(void)
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

        WaitMs(50);
        gpio_toggle(PULSE_OUTPUT_PIN);
    }
    return 0;
}


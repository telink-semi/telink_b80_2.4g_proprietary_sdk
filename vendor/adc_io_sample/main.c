#include "driver.h"
#include "common.h"

#define GREEN_LED_PIN           GPIO_PB4
#define ADC_INPUT_PIN           GPIO_PB0
volatile unsigned short sample_result[16] = {0};

static void  user_init(void)
{
    // led init
    gpio_set_output_en(GREEN_LED_PIN, 1); //enable output
    gpio_set_input_en(GREEN_LED_PIN, 0); //disable input
    gpio_write(GREEN_LED_PIN, 0); //LED Off

    // adc init
    adc_init();
    adc_set_ain_pre_scaler(ADC_PRESCALER_1F8);
    adc_base_init(ADC_INPUT_PIN);
    adc_power_on_sar_adc(1);
}

int main(void)
{
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    clock_init(SYS_CLK_24M_Crystal);

    user_init();

    unsigned char i = 0;
    while (1)
    {
        sample_result[i] = adc_sample_and_get_result();
        i = (i + 1) % 16;
        WaitMs(50);
        gpio_toggle(GREEN_LED_PIN);
    }
}

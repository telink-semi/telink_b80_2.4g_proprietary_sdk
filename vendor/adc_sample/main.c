#include "driver.h"
#include "common.h"

#define GREEN_LED_PIN           GPIO_PA5
#define ADC_INPUT_PIN           ADC_GPIO_PB0

#define ADC_BASE_MODE			    1	//GPIO voltage
#define ADC_VBAT_CHANNEL_MODE		2	//Vbat channel Battery Voltage
#define ADC_TEMP_MODE_EE			3	//Temp test
#define ADC_RNG_MODE			    4	//Random number Gernerator
#define MANNUAL_MODE_GET_ADC_SAMPLE_RESULT		1	//Vbat channel Battery Voltage mannual mode

#define ADC_MODE				    ADC_BASE_MODE

volatile unsigned short sample_result[16] = {0};
volatile signed short temp_new_val;
volatile unsigned int rns_val;
volatile unsigned short adc_manual_val;

static void  user_init(void)
{
    // led init
    gpio_set_output_en(GREEN_LED_PIN, 1); //enable output
    gpio_set_input_en(GREEN_LED_PIN, 0); //disable input
    gpio_write(GREEN_LED_PIN, 0); //LED Off

    // adc init
#if(ADC_MODE==ADC_RNG_MODE)
    random_generator_init();
#else
    adc_init();
	#if(ADC_MODE==ADC_BASE_MODE)
    adc_base_init(ADC_INPUT_PIN);
	adc_set_ain_pre_scaler(ADC_PRESCALER_1F8);//ADC pre_scaling default value is ADC_PRESCALER_1F8, it can change after adc_base_init().
    #elif (ADC_MODE==ADC_VBAT_CHANNEL_MODE)
	adc_vbat_channel_init();
    #elif (ADC_MODE==ADC_TEMP_MODE_EE)
	adc_temp_init();
	#endif

	adc_power_on_sar_adc(1);		//After setting the ADC parameters, turn on the ADC power supply control bit

#endif

}

int main(void)
{
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    wd_32k_stop();

	user_read_flash_value_calib();

    clock_init(SYS_CLK_24M_Crystal);

    user_init();

    unsigned char i = 0;
    while (1)
    {

#if(ADC_MODE==ADC_RNG_MODE)
	rns_val = rand();

#else

	#if(ADC_MODE==ADC_BASE_MODE)
		sample_result[i] = adc_sample_and_get_result();
		i = (i + 1) % 16;
		WaitMs(50);
		gpio_toggle(GREEN_LED_PIN);
    #elif (ADC_MODE==ADC_VBAT_CHANNEL_MODE)
		sample_result[i] = adc_sample_and_get_result();
		i = (i + 1) % 16;
		WaitMs(50);
		gpio_toggle(GREEN_LED_PIN);
    #elif (ADC_MODE==ADC_TEMP_MODE_EE)
		temp_new_val = adc_temp_result();
	#endif

    #if(MANNUAL_MODE_GET_ADC_SAMPLE_RESULT==1)
		adc_manual_val = adc_sample_and_get_result_manual_mode();
    #endif

#endif

    }
}



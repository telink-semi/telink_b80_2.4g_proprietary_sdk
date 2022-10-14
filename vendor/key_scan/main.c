#include "driver.h"


#define	SOURCE_32K_RC 				0
#define	SOURCE_32K_XTAL 			1
#define KEYSCAN_32K_SOURCE			SOURCE_32K_RC


#define BLUE_LED_PIN     		        GPIO_PA4
#define GREEN_LED_PIN     		        GPIO_PA5

#define ROW_CNT		2
#define COL_CNT		2
unsigned char g_ks_row[ROW_CNT] = {	KS_PA0, KS_PD4};
unsigned char g_ks_col[COL_CNT] = { KS_PF0,	KS_PF1};
volatile unsigned char g_key_value[256] = {0};
volatile unsigned char g_key_value_rptr = 0;
volatile unsigned char g_key_value_wptr = 0;
volatile unsigned char g_keyscan_error_flag = 0;//1 indicates that the data stored in the interrupt is abnormal.

_attribute_ram_code_sec_noinline_ void irq_handler(void)
{
	if(reg_comb_irq & FLD_IRQ_KS){
			unsigned char rptr = 0;
			unsigned char wptr = 0;
			unsigned char key_val = 0;
			keyscan_clr_irq_status();
			while(1){
			rptr = keyscan_get_rptr();
			wptr = keyscan_get_wptr();
			key_val = keyscan_get_ks_value();
			g_key_value[g_key_value_wptr] = key_val;
			g_key_value_wptr = (g_key_value_wptr + 1)&0xff;
			if(key_val == KESYCAN_END_FLAG){
				break;
			}else if(rptr == wptr){
				g_keyscan_error_flag = 1;
				break;
			}
			}
		}
}

void user_init()
{
	gpio_set_func(BLUE_LED_PIN|GREEN_LED_PIN ,AS_GPIO);
	gpio_set_output_en(BLUE_LED_PIN|GREEN_LED_PIN , 1); 		//enable output
	gpio_set_input_en(BLUE_LED_PIN|GREEN_LED_PIN ,0);			//disable input


	//set gpio as to keyscan.
	keyscan_set_martix((unsigned char*)g_ks_row, ROW_CNT, (unsigned char*)g_ks_col, COL_CNT, KS_INT_PIN_PULLDOWN);

	keyscan_init(DEBOUNCE_PERIOD_8MS, 1, TRIPLE_SCAN_TIMES);

	keyscan_enable();

	irq_enable_type(FLD_IRQ_KS_CMD_EN);
	irq_enable();
	printf("\n Keyscan Test Start! Please press button. \n");
}


int main (void)
{

#if(KEYSCAN_32K_SOURCE==SOURCE_32K_XTAL)
	blc_pm_select_external_32k_crystal();
#else
	blc_pm_select_internal_32k_crystal();
#endif

	cpu_wakeup_init(EXTERNAL_XTAL_24M);

    wd_32k_stop();

	user_read_flash_value_calib();

	gpio_init(1);

	clock_init(SYS_CLK_24M_Crystal);

	user_init();
	while(1)
	{
		sleep_ms(500);
			while(1){
				if(g_key_value_rptr == g_key_value_wptr){
					break;
				}
				if(g_key_value[g_key_value_rptr] != KESYCAN_END_FLAG){
					printf("%d row %d column\n",g_key_value[g_key_value_rptr]>>5, g_key_value[g_key_value_rptr]&0x1f);
				}
				g_key_value_rptr = (g_key_value_rptr + 1)&0xff;
			}
			gpio_toggle(BLUE_LED_PIN|GREEN_LED_PIN);
			sleep_ms(200);
	}
	return 0;
}

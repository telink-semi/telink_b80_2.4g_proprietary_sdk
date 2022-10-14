#include "driver.h"
#define GREEN_LED_PIN           GPIO_PA5

unsigned char debug_logo[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};

int main(void)
{
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    wd_32k_stop();

	user_read_flash_value_calib();

    clock_init(SYS_CLK_24M_Crystal);

    gpio_set_output_en(GREEN_LED_PIN, 1); //enable output
    gpio_set_input_en(GREEN_LED_PIN, 0); //disable input
    gpio_write(GREEN_LED_PIN, 0);

    usb_set_pin_en();
    usb_loginit();
    WaitMs(3000); //delay to ensure USB enumerate done

    while (1)
    {
        gpio_toggle(GREEN_LED_PIN);
        log_msg("logo", debug_logo, sizeof(debug_logo));
        WaitMs(500);
    }
    return 0;
}

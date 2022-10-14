#include "driver.h"

#define CLOCK_SYS_CLOCK_HZ      24000000

/* List tick per second/millisecond/microsecond */
enum {
    CLOCK_SYS_CLOCK_1S = CLOCK_SYS_CLOCK_HZ,
    CLOCK_SYS_CLOCK_1MS = (CLOCK_SYS_CLOCK_1S / 1000),
    CLOCK_SYS_CLOCK_1US = (CLOCK_SYS_CLOCK_1S / 1000000),
};

#define TEST_PWM_NORMAL_MODE_1  1
#define TEST_PWM_NORMAL_MODE_2  2
#define TEST_PWM_NORMAL_MODE_3  3
#define TEST_PWM_SELECT         TEST_PWM_NORMAL_MODE_1

/*********************************************************************************

GPIO PB[4,5,6,7] | PC[0,1,2,3,4,5,6,7] support all pwm chnn
if you want use pwm by other pins, read <b80 gpio lookup table> for more msgs

*********************************************************************************/

void app_pwm_test(void)
{
    pwm_set_clk(CLOCK_SYS_CLOCK_HZ, CLOCK_SYS_CLOCK_HZ);
#if (TEST_PWM_SELECT == TEST_PWM_NORMAL_MODE_1)  //test PWMx (0~5)   normal mode
    //PB4 PWM0  1ms cycle  1/2 duty
    gpio_set_func(GPIO_PB4, PWM0);
    pwm_set_mode(PWM0_ID, PWM_NORMAL_MODE);
    pwm_set_cycle_and_duty(PWM0_ID, (unsigned short) (1000 * CLOCK_SYS_CLOCK_1US),  (unsigned short) (500 * CLOCK_SYS_CLOCK_1US));
    pwm_start(PWM0_ID);

    //PB5 PWM1  1ms cycle  1/3 duty
    gpio_set_func(GPIO_PB5, PWM1); // b80 do not export PA3\PC3
    pwm_set_mode(PWM1_ID, PWM_NORMAL_MODE);
    pwm_set_cycle_and_duty(PWM1_ID, (unsigned short) (1000 * CLOCK_SYS_CLOCK_1US),  (unsigned short) (333 * CLOCK_SYS_CLOCK_1US) );
    pwm_start(PWM1_ID);

    //PPB6 PWM2   1ms cycle  1/4 duty
    gpio_set_func(GPIO_PB6, PWM2);
    pwm_set_mode(PWM2_ID, PWM_NORMAL_MODE);
    pwm_set_cycle_and_duty(PWM2_ID, (unsigned short) (1000 * CLOCK_SYS_CLOCK_1US),  (unsigned short) (250 * CLOCK_SYS_CLOCK_1US) );
    pwm_start(PWM2_ID);

    //PB0 PWM3  1ms cycle  1/5 duty
    gpio_set_func(GPIO_PB7, PWM3);
    pwm_set_mode(PWM3_ID, PWM_NORMAL_MODE);
    pwm_set_cycle_and_duty(PWM3_ID, (unsigned short) (1000 * CLOCK_SYS_CLOCK_1US),  (unsigned short) (200 * CLOCK_SYS_CLOCK_1US) );
    pwm_start(PWM3_ID);

    //PC0 PWM4  1ms cycle  2/3 duty
    gpio_set_func(GPIO_PC0, PWM4);
    pwm_set_mode(PWM4_ID, PWM_NORMAL_MODE);
    pwm_set_cycle_and_duty(PWM4_ID, (unsigned short) (1000 * CLOCK_SYS_CLOCK_1US),  (unsigned short) (667 * CLOCK_SYS_CLOCK_1US) );
    pwm_start(PWM4_ID);

    //PC1 PWM5  1ms cycle  3/4 duty
    gpio_set_func(GPIO_PC1, PWM5);
    pwm_set_mode(PWM5_ID, PWM_NORMAL_MODE);
    pwm_set_cycle_and_duty(PWM5_ID, (unsigned short) (1000 * CLOCK_SYS_CLOCK_1US),  (unsigned short) (750 * CLOCK_SYS_CLOCK_1US) );
    pwm_start(PWM5_ID);

#elif (TEST_PWM_SELECT == TEST_PWM_NORMAL_MODE_2)  //test PWMx and PWMx_N(0~2)   normal mode

    //GPIO_PB4 PWM0     1ms cycle  1/3 duty
    //GPIO_PB5 PWM0_N   1ms cycle  2/3 duty
    gpio_set_func(GPIO_PB4, PWM0);
    gpio_set_func(GPIO_PB5, PWM0_N);
    pwm_set_mode(PWM0_ID, PWM_NORMAL_MODE);
    pwm_set_cycle_and_duty(PWM0_ID, (unsigned short) (1000 * CLOCK_SYS_CLOCK_1US),  (unsigned short) (333 * CLOCK_SYS_CLOCK_1US) );

    //PB6 PWM1     1ms cycle  1/4 duty
    //PB7 PWM1_N   1ms cycle  3/4 duty
    gpio_set_func(GPIO_PB6, PWM1);
    gpio_set_func(GPIO_PB7, PWM1_N);
    pwm_set_mode(PWM1_ID, PWM_NORMAL_MODE);
    pwm_set_cycle_and_duty(PWM1_ID, (unsigned short) (1000 * CLOCK_SYS_CLOCK_1US),  (unsigned short) (250 * CLOCK_SYS_CLOCK_1US) );

    //PC0 PWM2     1ms cycle  1/5 duty
    //PC1 PWM2_N   1ms cycle  4/5 duty
    gpio_set_func(GPIO_PC0, PWM2);
    gpio_set_func(GPIO_PC1, PWM2_N);
    pwm_set_mode(PWM2_ID, PWM_NORMAL_MODE);
    pwm_set_cycle_and_duty(PWM2_ID, (unsigned short) (1000 * CLOCK_SYS_CLOCK_1US),  (unsigned short) (200 * CLOCK_SYS_CLOCK_1US) );

    pwm_start(PWM0_ID);
    pwm_start(PWM1_ID);
    pwm_start(PWM2_ID);

#elif (TEST_PWM_SELECT == TEST_PWM_NORMAL_MODE_3)  //test PWMx and PWMx_N(3~5)   normal mode

    //PC0 PWM3     1ms cycle  1/3 duty
    //PC1 PWM3_N   1ms cycle  2/3 duty
    gpio_set_func(GPIO_PC0, PWM3);
    gpio_set_func(GPIO_PC1, PWM3_N);
    pwm_set_mode(PWM3_ID, PWM_NORMAL_MODE);
    pwm_set_phase(PWM3_ID, 0);   //no phase at pwm beginning
    pwm_set_cycle_and_duty(PWM3_ID, (unsigned short) (1000 * CLOCK_SYS_CLOCK_1US),  (unsigned short) (333 * CLOCK_SYS_CLOCK_1US) );

    //PC4 PWM4     1ms cycle  1/4 duty
    //PC5 PWM4_N   1ms cycle  3/4 duty
    gpio_set_func(GPIO_PC4, PWM4);
    gpio_set_func(GPIO_PC5, PWM4_N);
    pwm_set_mode(PWM4_ID, PWM_NORMAL_MODE);
    pwm_set_phase(PWM4_ID, 0);   //no phase at pwm beginning
    pwm_set_cycle_and_duty(PWM4_ID, (unsigned short) (1000 * CLOCK_SYS_CLOCK_1US),  (unsigned short) (250 * CLOCK_SYS_CLOCK_1US) );

    //PC6 PWM5     1ms cycle  1/5 duty
    //PC7 PWM5_N   1ms cycle  4/5 duty
    gpio_set_func(GPIO_PC6, PWM5);
    gpio_set_func(GPIO_PC7, PWM5_N);
    pwm_set_mode(PWM5_ID, PWM_NORMAL_MODE);
    pwm_set_phase(PWM5_ID, 0);   //no phase at pwm beginning
    pwm_set_cycle_and_duty(PWM5_ID, (unsigned short) (1000 * CLOCK_SYS_CLOCK_1US),  (unsigned short) (200 * CLOCK_SYS_CLOCK_1US) );

    pwm_start(PWM3_ID);
    pwm_start(PWM4_ID);
    pwm_start(PWM5_ID);

#else

#endif
}


int main(void)
{
    cpu_wakeup_init(EXTERNAL_XTAL_24M);

    wd_32k_stop();

	user_read_flash_value_calib();

    clock_init(SYS_CLK_24M_Crystal);

    app_pwm_test();

    while (1)
    {

    }
    return 0;
}


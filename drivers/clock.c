/********************************************************************************************************
 * @file	clock.c
 *
 * @brief	This is the source file for B80
 *
 * @author	Driver Group
 * @date	2021
 *
 * @par     Copyright (c) 2021, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *          All rights reserved.
 *
 *          Licensed under the Apache License, Version 2.0 (the "License");
 *          you may not use this file except in compliance with the License.
 *          You may obtain a copy of the License at
 *
 *              http://www.apache.org/licenses/LICENSE-2.0
 *
 *          Unless required by applicable law or agreed to in writing, software
 *          distributed under the License is distributed on an "AS IS" BASIS,
 *          WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *          See the License for the specific language governing permissions and
 *          limitations under the License.
 *
 *******************************************************************************************************/
#include "register.h"
#include "clock.h"
#include "irq.h"
#include "analog.h"
#include "timer.h"
#include "pm.h"
#include "./otp/otp.h"
#include "compiler.h"



extern _attribute_data_retention_ unsigned char tl_24mrc_cal;

_attribute_data_retention_	unsigned char system_clk_type;

#if 0
/**
 * @brief       This function to set RC for the system clock.
 * @param[in]   SYS_CLK - the clock source of the system clock.
 * @return      none
 */
void clock_rc_set(SYS_CLK_TypeDef SYS_CLK)
{
	unsigned char temp = analog_read(0x04)&0xfc;
	if(SYS_CLK==SYS_CLK_24M_RC)
	{
		analog_write(0x04, temp|0x00);
		rc_24m_cal();
	}
	else if(SYS_CLK==SYS_CLK_32M_RC)
	{
		analog_write(0x04, temp|0x01);
		//rc_32m_cal();
	}
	else if(SYS_CLK==SYS_CLK_48M_RC)
	{
		analog_write(0x04, temp|0x03);
		rc_48m_cal();
	}
}
#endif
/**
 * @brief       This function to select the system clock source.
 * @param[in]   SYS_CLK - the clock source of the system clock.
 * @return      none
 * @note		Do not switch the clock during the DMA sending and receiving process;
 * 			    because during the clock switching process, the system clock will be
 * 			    suspended for a period of time, which may cause data loss.
 */
#if (BLC_PM_DEEP_RETENTION_MODE_EN)
_attribute_ram_code_sec_noinline_
#endif
void clock_init(SYS_CLK_TypeDef SYS_CLK)
{
	reg_clk_sel = (unsigned char)SYS_CLK;
	system_clk_type = (unsigned char)SYS_CLK;
	otp_set_clk(SYS_CLK);
	otp_set_auto_pce_tcs(SYS_CLK);
#if (SYSCLK_RC_CLOCK_EN)
	if(SYS_CLK<SYS_CLK_RC_THRES)
	{
		clock_rc_set(SYS_CLK);
	}
#endif
	if(!pm_is_MCU_deepRetentionWakeup()){
		rc_24m_cal ();
	}

}

/**
 * @brief   This function serves to set 32k clock source.
 * @param[in]   variable of 32k type.
 * @return  none.
 */
void clock_32k_init (CLK_32K_TypeDef src)
{
	unsigned char sel_32k   = analog_read(0x2d)&0x7f;
	unsigned char power_32k = analog_read(0x05)&0xfc;
	analog_write(0x2d, sel_32k|(src<<7));
	if(src)
	{
		analog_write(0x05, power_32k|0x1);//32k xtal
		//2.set pc3 as pwm output
		unsigned char sys_clk = read_reg8(0x66);
		write_reg8(0x66,0x43);
		unsigned char reg_596 = read_reg8(0x596);
		write_reg8(0x596,reg_596&0xf7);
		unsigned short reg_798 = read_reg16(0x798);
		write_reg16(0x798,0x01);
		unsigned short reg_79a = read_reg16(0x79a);
		write_reg16(0x79a,0x02);
		unsigned char reg_780 = read_reg8(0x780);
		write_reg8(0x780,0x02);
		write_reg8(0x782,0xf3);

		//3.wait for PWM wake up Xtal
		sleep_ms(5);

		//4.Xtal 32k output
		analog_write(0x03,0x4f); //<7:6>current select

		//5.Recover PC3 as Xtal pin
		write_reg8(0x66,sys_clk);
		write_reg8(0x596,reg_596);
		write_reg16(0x798,reg_798);
		write_reg16(0x79a,reg_79a);
		write_reg8(0x780,reg_780);
	}
	else
	{
		analog_write(0x05, power_32k|0x2);//32k rc
	}
}

/**
 * @brief     This function performs to select 48M RC as the system clock source.
 * @param[in] none.
 * @return    none.
 */
void rc_48m_cal (void)
{
	analog_write(0x33, 0x80);
	analog_write(0x30, 0x20);
    analog_write(0xc7, 0x0e);
    sleep_us(1000);
    analog_write(0xc7, 0x0f);
    while((analog_read(0xcf) & 0x80) == 0);

    volatile unsigned int cal_cnt = analog_read(0xcf)&0x07 ;
    cal_cnt = (cal_cnt<<8) + analog_read(0xce);
    unsigned int f = 64;
    unsigned int temp_v = 0;
    unsigned int temp_d = 0;
    unsigned int temp_d2 = 100;
    unsigned char temp_cap = 0;
    unsigned int i=0;
    while(f>=1)
    {
		temp_v = analog_read(0x33);
    	if(cal_cnt>250)
    	{
    		temp_d =  cal_cnt - 250;
    	}
    	else
    	{
    		temp_d =  250 - cal_cnt;
    	}
    	if(cal_cnt>250)
    	{
    		analog_write(0x33, temp_v-f);
    	}
    	else
    	{
    		analog_write(0x33, temp_v+f);
    	}
    	f = f/2;
        analog_write(0xc7, 0x0e);
        analog_write(0xc7, 0x0f);
        while((analog_read(0xcf) & 0x80) == 0);
        cal_cnt = analog_read(0xcf)&0x07 ;
		cal_cnt = (cal_cnt<<8) + analog_read(0xce);
		i++;
    	if(temp_d2>temp_d)
    	{
    		temp_d2 = temp_d;
    		temp_cap = temp_v;
    	}
    }
    analog_write(0x33, temp_cap);
}

/**
 * @brief     This function performs to select 24M as the system clock source.
 * @param[in] none.
 * @return    none.
 */
void rc_24m_cal (void)
{
    analog_write(0xc8, 0x80);

//    sub_wr_ana(0x30, 1, 7, 7);
    analog_write(0x30, analog_read(0x30) | BIT(7) );

    analog_write(0xc7, 0x0e);
    analog_write(0xc7, 0x0f);
    while((analog_read(0xcf) & 0x80) == 0);
    unsigned char cap = analog_read(0xcb);
    analog_write(0x33, cap);		//write 24m cap into manual register

//	sub_wr_ana(0x30, 0, 7, 7);	//manual on
    analog_write(0x30, analog_read(0x30) & (~BIT(7)) );

	analog_write(0xc7, 0x0e);
	tl_24mrc_cal = analog_read(0x33);
}

/**
 * @brief     This function performs to select 32K as the system clock source.
 * @param[in] none.
 * @return    none.
 */
void rc_32k_cal (void)
{
    analog_write(0x30, 0x60);
    analog_write(0xc6, 0xf6);
    analog_write(0xc6, 0xf7);
    while((analog_read(0xcf) & BIT(6)) == 0);
	unsigned char res1 = analog_read(0xc9);	//read 32k res[13:6]
	analog_write(0x32, res1);		//write 32k res[13:6] into manual register
	unsigned char res2 = analog_read(0xca);	//read 32k res[5:0]
	analog_write(0x31, res2);		//write 32k res[5:0] into manual register
	analog_write(0xc6, 0xf6);
	analog_write(0x30, 0x20);//manual on
}


/**
 * @brief     This function performs to probe clock to IO.
 * @param[in] src - the clock source which you want to probe.
 * @param[in] pin - the pin to probe clock.exclude PA[1]/PA[2]/PA[3]/B[0]/B[1]/B[3]/PD[4]/PE[3:0]/PF[1:0]
 * @return    none.
 */
void clock_prob(prob_clock_src_e src, GPIO_PinTypeDef pin)
{
	if(PROB_CLK_32K == src)
	{
		analog_write(0x2d, (analog_read(0x2d) & 0x7f) | (blt_miscParam.pad32k_en << 7));
	}

	write_reg8(0x75, ((read_reg8(0x75) & 0xf8) | src));	//0:clk_7816,  1:clk32k,   2:clk_sys,      3:rc24m
												  	    //4:xtl24m,    5:clkpll,   6:clk_stimer,   7:clk_usbphy
	BM_CLR(reg_gpio_func(pin), (pin&0xff));
	reg_gpio_func_mux(pin) = ((reg_gpio_func_mux(pin) & 0xc0) | 0x15);
}


/**
 * @brief     This function performs to select 32K as source of DMIC.
 * @param[in] source clock to provide DMIC.
 * @return    none.
 */
void dmic_prob_32k(unsigned char src)
{
	analog_write(0x2d, (analog_read(0x2d) & 0x7f));	  		//32k clk select

	write_reg8(0x75, read_reg8(0x75) | BIT(0));				//probe_clk_sel,
											   	   	   	    //0:clk_7816,  1:clk32k,   2:clk_sys,      3:rc24m
											   	   	   	   	//4:xtl24m,    5:clkpll,   6:clk_stimer,   7:clk_usbphy
	write_reg8(0x506, (read_reg8(0x506) & ~(BIT(0))));  	//pa_io[0]=0
	write_reg8(0x548, (read_reg8(0x548) & 0xc0) | 0x15);	//mux sel clk7816
}

/**
 * @brief     This function performs to select 24M/2 RC as source of DMIC.
 * @param[in] source clock to provide DMIC.
 * @return    none.
 */
void dmic_prob_24M_rc()
{

	write_reg8(0x75, read_reg8(0x75) | BIT_RNG(0,1));   //probe_clk_sel,
												  	    //0:clk_7816,  1:clk32k,   2:clk_sys,      3:rc24m
												  	    //4:xtl24m,    5:clkpll,   6:clk_stimer,   7:clk_usbphy
	write_reg8(0x506, (read_reg8(0x506) & ~(BIT(0))));  	//pa_io[0]=0
	write_reg8(0x548, (read_reg8(0x548) & 0xc0) | 0x15);	//mux sel clk7816

}



/********************************************************************************************************
 * @file	otp_base.c
 *
 * @brief	This is the source file for B80
 *
 * @author	Driver Group
 * @date	2022
 *
 * @par		Copyright (c) 2022, Telink Semiconductor (Shanghai) Co., Ltd.
 *			All rights reserved.
 *
 *          The information contained herein is confidential property of Telink
 *          Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *          of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *          Co., Ltd. and the licensee or the terms described here-in. This heading
 *          MUST NOT be removed from this file.
 *
 *          Licensee shall not delete, modify or alter (or permit any third party to delete, modify, or
 *          alter) any information contained herein in whole or in part except as expressly authorized
 *          by Telink semiconductor (shanghai) Co., Ltd. Otherwise, licensee shall be solely responsible
 *          for any claim to the extent arising out of or relating to such deletion(s), modification(s)
 *          or alteration(s).
 *
 *          Licensees are granted free, non-transferable use of the information in this
 *          file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided.
 *
 *******************************************************************************************************/
#include "otp_base.h"
extern unsigned char otp_program_flag;

/**
 * @brief     This function servers to enable pce auto mode,after enable pce auto mode,
 *            If the time from the last instruction fetching to the next instruction fetching exceeds the set timeout time,
 *            the pce will be pulled down. If the pce is pulled down, the hardware will automatically pull up when the instruction fetching,
 *            and clk will not work until the tsc time. This mechanism can save power consumption but reduce efficiency.
 * @param[in] none
 * @return	  none.
 */
_attribute_ram_code_sec_noinline_ void otp_auto_pce_enable(){
	reg_otp_ctrl2 |= FLD_OTP_AUTO_PCE_MODE;
}

/**
 * @brief      This function serves to otp set deep standby mode,if code run in flash,otp is idle,can enter deep to save current,
 *             if code run in otp,otp_set_deep_standby_mode and otp_set_active_mode are restricted to use in cpu_sleep_wakeup.
 * @param[in]  none
 * @return     none
 */
_attribute_ram_code_sec_noinline_ void otp_set_deep_standby_mode(void)
{
	/*
	 * 1.Tash has no maximum limit and does not require interrupt protection;
	 * 2.this function is limited to the cpu_sleep_wakeup function, earlyWakeup_us=90us, and the maximum pce_timeout is about 22us,
	 * so there is no need to wait for a pce timeout;
	 */
	reg_otp_ctrl0 &= ~(FLD_OTP_PCE);//pce=0
	//Tash >10(ns)
	reg_otp_ctrl1 &=~(FLD_OTP_PLDO|FLD_OTP_PDSTD);    //pdstb=0  pldo=0  Tplh >= 0(ns)
}
/**
 * @brief      This function serves to otp set active mode,if otp is in deep mode,need to operate on the otp,set active mode,
 *             if code run in otp,otp_set_deep_standby_mode and otp_set_active_mode are restricted to use in cpu_sleep_wakeup.
 * @param[in]  none
 * @return     none
 */
_attribute_ram_code_sec_noinline_ void otp_set_active_mode(void)
{
	/*
	 *Tpls/Tsas has no maximum limit and does not require interrupt protection;
	 */
	reg_otp_ctrl1 |= FLD_OTP_PLDO; //pldo=1
	for(unsigned char i = 0; i < 100; i++){//Tpls:the requirement is greater than 10us, set to 14us according to 48M,if is 24RC,the corresponding time is 29us.
		asm("tnop");
	}
	reg_otp_ctrl1 |= FLD_OTP_PDSTD;//pdstb=1
    for(unsigned char i = 0; i < 35; i++){//Tsas:the requirement is greater than 2us, set to 5us according to 48M,if is 24RC,the corresponding time is 10us.
		asm("tnop");
	}
    /*
	 * If bin is downloaded to ram or flash, can disable pce to save power consumption.
	 * if bin is downloaded to otp:
	 * 1.if pce_auto_mode is enabled, the pce and ready signals are pulled down after the fetch command ends and the timeout period is reached,
	 *   when the pce is pulled up, the ready signal is pulled up after the tcs time,when the timeout mechanism is not triggered or the pce_auto_mode is turned off,
	 *   manually pull down the pce, and the ready signal will not be pulled down,if the pce is pulled up again, then the ready signal has been high,
	 *   and hardware tcs time no longer counts,however, the otp itself reading time sequence requires at least 10us time to work after the pce is raised, so manual delay is required,
	 * 2.both read and write timing and whether pce_auto_mode is enabled or not need to be restored，during read and write, the pce needs to be manually raised,
	 *   triggering the ready signal to be raised,then, when the interface is out, ptm needs to be configured, so the pce needs to be lowered,
	 *   but the ready signal does not,therefore, after the outbound interface, when the hardware raises the pce, the ready is already raised,
	 *   but the otp itself reading time sequence requires at least 10us time to work after the pce is raised, so need manually delay,otherwise the ack will fetch the instruction, causing an error.
	 */
#if !ONLY_SUPPORT_OTP_PROGRAM
    if(otp_program_flag==1)
#endif
    {
       reg_otp_ctrl0 |= FLD_OTP_PCE;
	   for(unsigned char i = 0; i < 100; i++){//Tcs:the requirement is greater than 10us, set to 14us according to 48M,if is 24RC,the corresponding time is 29us.
		  asm("tnop");
	   }
    }
}

/**
 * @brief     This function servers to set tcs time,the minimum value of Tcs_time is 10us,
 *            to improve efficiency, the Tcs_time is set to the minimum value corresponding to the system clock.
 * @param[in] SYS_CLK - system clock,Tcs_time=24M/SYS_CLK*((tcs_config(bit5-7) + 1)*2.688 us).
 * | :-------------- | :--------------- | :------------ |
 * |    SYS_CLK      |    tcs_config    |   tcs_time    |
 * |   12M_Crystal   |     0x01         |   10.752us    |
 * |   16M_Crystal   |     0x02         |   12.096us    |
 * |   24M_Crystal   |     0x03         |   10.752us    |
 * |   32M_Crystal   |     0x05         |   12.096us    |
 * |   48M_Crystal   |     0x07         |   10.752us    |
 * |   24M_RC        |     0x03         |   10.752us    |
 * @return	  none.
 */
_attribute_ram_code_sec_noinline_ void otp_set_auto_pce_tcs(SYS_CLK_TypeDef SYS_CLK){
    unsigned char tcs_config= (SYS_CLK>>8) &0x07;
	reg_otp_ctrl2 =(reg_otp_ctrl2 &(~FLD_OTP_TCS_CONFIG))|(tcs_config<<5);
}


/**
 * @brief     This function servers to set otp clk,hardware settings otp maximum clk is 12M,
 *            in this function, otp_clk is set to the highest based on the sys_clk passed in.
 * @param[in] SYS_CLK - system clock,reg_otp_paio_bit6: 0 - two frequency,1 - four frequency.
 * | :-------------- | :---------------  | :------------ |
 * |    SYS_CLK      |    tcs_config     |    otp_clk    |
 * |   12M_Crystal   |     0x00          |      6M       |
 * |   16M_Crystal   |     0x00          |      8M       |
 * |   24M_Crystal   |     0x00          |      12M      |
 * |   32M_Crystal   |     0x01          |      8M       |
 * |   48M_Crystal   |     0x01          |      12M      |
 * |   24M_RC        |     0x00          |      12M      |
 * @return	  none.
 */
_attribute_ram_code_sec_noinline_ void otp_set_clk(SYS_CLK_TypeDef SYS_CLK)
{
	unsigned char div_flag = (SYS_CLK>>11)&0x01;
	if(div_flag){
		reg_otp_paio |= FLD_OTP_CLK_DIV;
	}else{
		reg_otp_paio &=~FLD_OTP_CLK_DIV;
	}
}

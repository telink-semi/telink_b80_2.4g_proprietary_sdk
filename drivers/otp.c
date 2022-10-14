/********************************************************************************************************
 * @file	otp.c
 *
 * @brief	This is the source file for B80
 *
 * @author	Driver Group
 * @date	2021
 *
 * @par		Copyright (c) 2021, Telink Semiconductor (Shanghai) Co., Ltd.
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
#include "otp.h"
#include "timer.h"
#include "irq.h"
unsigned char otp_program_flag;
unsigned char g_pce_timeout=6;
unsigned char g_irq_en=0;


/**
 * @brief     This function servers to waiting for pce timeout.
 * @param[in] none
 * @return	  none.
 */
_attribute_ram_code_sec_noinline_ void otp_pce_timeout_exceed(void){
	/*
	 * Interrupt protection needs to be added because the read and write function interfaces require the use of delay functions and time on the sequence;
	 */
	 g_irq_en = irq_disable();

#if OTP_PCE_AUTO_MODE_EN
#if !ONLY_SUPPORT_OTP_PROGRAM
	 if(otp_program_flag==1)
#endif
	{
	 unsigned long t = clock_time();
	 /*
	 * 1.If the pce auto mode is enabled, there will be a timeout mechanism. when the software manually operates the pce within the timeout time,
	 *   if the pce is manually lowered, timeout will not be invalid, however, if the timeout is not over, pull the pce up manually. when the timeout is over,
	 *   pull the pce down.therefore, in order to prevent the interaction between software and hardware and disrupt the timing sequence,
	 *   it is necessary to wait for the pce hardware to pull down automatically before performing software processing;
	 * 2.when pce_auto_mode is enabled, the pce hardware will automatically pull down, which will be triggered only after mcu fetches the instruction from otp.
	 *   If the previous function (to pull up the pce) moves to the next function (to wait for the pce hardware to pull down automatically),
	 *   the prefetch instruction (cache:4 word) causes no fetch from otp,then it gets stuck, so add timeout_exceed to handle this exception,
	 */
	 while((reg_otp_ctrl0&FLD_OTP_PCE)&&(!clock_time_exceed(t, g_pce_timeout)));
	}
#endif
}
/**
 * @brief      This function serves to enable the otp test area,only for internal testing,there are two operating units, each of which is 16 words,one start at address 0x00,
 *             the other the otp start at address 0x4000, test area address is independent of the real otp address,if use the otp test area,need to use this function.
 * @param[in]  none
 * @return     none
 */
void otp_test_mode_en(void){
	 reg_otp_ctrl0 |= FLD_OTP_PTR;
}
/**
 * @brief      This function serves to disable the otp test area,only for internal testing,if do not use the otp test area,use this function,
 *             will transition from test area to operating on real the otp.
 * @param[in]  none
 * @return     none
 */
void otp_test_mode_dis(void){
	reg_otp_ctrl0 &= ~(FLD_OTP_PTR);
}

/**
 * @brief      This function serves to wait until the operation of OTP is done.
 * @param[in]  none
 * @return     none
 */
_attribute_ram_code_sec_ static inline void otp_wait_done(void)
{
	while(reg_otp_dat & FLD_OTP_BUSY);
}

/*
 * @brief     This function is a common sequence used by these interfaces:otp_write32/otp_read_cycle/otp_set_active_mode.
 * @param[in] ptm_mode - ptm type.
 * @return    none
 */
_attribute_ram_code_sec_ void otp_start(OTP_PtmTypeDef ptm_mode)
{
	if(otp_program_flag==0||(!OTP_PCE_AUTO_MODE_EN))
	{
	   //clk_en does not need to be turned off because the clk is controlled internally by the ready signal.
	   reg_otp_ctrl0 &= ~(FLD_OTP_PCE);
	}
	//ptm
	reg_otp_ctrl1 &= (~FLD_OTP_PTM);
	reg_otp_ctrl1 |= ptm_mode;
	//Tms >= 1(ns)
	//pce
	reg_otp_ctrl0 |= (FLD_OTP_PCE);
	//clk_en needs to be opened manually if the program is in flash;
#if !ONLY_SUPPORT_OTP_PROGRAM
	if(otp_program_flag==0){
		reg_otp_ctrl0 |= (FLD_OTP_PCLK_EN);
	}
#endif
}
/**
 * @brief      This function serves to read data from OTP memory.
 * @param[in]  ptm_mode - read mode.
 * @param[in]  addr - the address of the data,the otp memory that can access is from 0x0000-0x7ffc,can't access other address.
 * @param[in]  len  - the length of the data,the unit is word(4 bytes).
 * @param[in]  buff - data buff.
 * @return     none
 */
_attribute_ram_code_sec_ void otp_read_cycle(OTP_PtmTypeDef ptm_mode,unsigned int addr, unsigned int word_len, unsigned int *buff)
{
	//if code run in otp,when switching to ram_code,pce/pclk may be pulled up,when read to otp,ptm needs to be configured,prog/pce need to pull down.
	otp_start(ptm_mode);
	/*
	 * software tcs counting: when the pce is pulled up and the ready signal is pulled down, the software tcs will start counting,when the count reaches the register configuration time,
	 * the ready signal is up and the clk starts working. if the pce is raised and the ready signal is in the raised state, the clk will work and the software tcs will not time;
	 * hardware tcs: otp read operation, pce pull, must wait tcs time (> 10us) before otp can be read;
	 * description:
	 * 1. If pce_auto_mode timeout is triggered, then the ready signal will also be lowered when the pce is lowered, so you need to wait for the software register configuration time,
	 *    using the ready signal is ok. there is no need to consider whether to call otp_set_auto_pce_tcs;
	 * 2. If enter this function,no timeout mechanism is triggered, ready has triggered the pull state, then need to delay wait for the tcs of otp hardware;
	 */
	sleep_us(Tcs);
	while(!(reg_otp_dat|FLD_OTP_READY));
    /*
     * change the address increment mode to manual mode:1.In the address increment mode, due to the address increment function,
     * when the for loop ends, an extra word will be added to the address, which may be increased to the address 0x3ffc that is not wanted to be read,
     * triggering the encryption and decryption module by mistake;2:if the encryption and decryption module is enabled, the auto increment function is unavailable;
     */

	for(unsigned int i = 0; i < word_len; i++){
		reg_otp_pa = addr+i*4;
		reg_otp_rd_dat;
		otp_wait_done();
		buff[i] = reg_otp_rd_dat;
	/*
	 * after executing reg_otp_rd_dat, two actions are triggered, one is to read the data, and the other is  trig hardware to read the current address,
	 * add a wait to prevent another read from 0x3ffc and an exception may occur during otp operations,so there is no limit to the address of the OTP.
	 */
		otp_wait_done();
	}
      reg_otp_ctrl0 &= ~(FLD_OTP_PCE);
}
/**
 * @brief      This function serves to preparations after otp software operation.
 * @param[in]  pce_flag - pce auto mode flag,from the return value of otp_auto_pce_disable function.
 * @return     none
 */
_attribute_ram_code_sec_ void otp_auto_pce_restore(){
	/*
	 * If bin is downloaded to ram or flash, can disable pce to save power consumption.
	 * if bin is downloaded to otp:
	 * 1.if pce_auto_mode is enabled, the pce and ready signals are pulled down after the fetch command ends and the timeout period is reached,
	 *   when the pce is pulled up, the ready signal is pulled up after the tcs time,when the timeout mechanism is not triggered or the pce_auto_mode is turned off,
	 *   manually pull down the pce, and the ready signal will not be pulled down,if the pce is pulled up again, then the ready signal has been high,
	 *   and hardware tcs time no longer counts,however, the otp itself reading time sequence requires at least 10us time to work after the pce is raised, so manual delay is required,
	 * 2.both read and write timing and whether pce_auto_mode is enabled or not need to be restored£¬during read and write, the pce needs to be manually raised,
	 *   triggering the ready signal to be raised,then, when the interface is out, ptm needs to be configured, so the pce needs to be lowered,
	 *   but the ready signal does not,therefore, after the outbound interface, when the hardware raises the pce, the ready is already raised,
	 *   but the otp itself reading time sequence requires at least 10us time to work after the pce is raised, so need manually delay,otherwise the ack will fetch the instruction, causing an error.
	 */
#if !ONLY_SUPPORT_OTP_PROGRAM
	if(otp_program_flag==1)
#endif
	{
		//ptm
	  reg_otp_ctrl1 &= (~FLD_OTP_PTM);
	  reg_otp_ctrl1 |= OTP_PTM_READ;
	  reg_otp_ctrl0 |= FLD_OTP_PCE;
	  sleep_us(Tcs);

	}

	irq_restore(g_irq_en);
}



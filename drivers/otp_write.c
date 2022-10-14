/********************************************************************************************************
 * @file	otp_write.c
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
#include "otp.h"

/**
 * @brief      This function serves to write data to OTP memory,4 bytes one time.
 * @param[in]  ptm_mode - write mode.
 * @param[in]  addr  - the address of the data,the otp memory that can access is from 0x0000-0x7ffc,can't access other address.
 * @param[in]  data  - the data need to be write,4 bytes.
 * @return     none
 */
_attribute_ram_code_sec_ static void otp_write32(OTP_PtmTypeDef ptm_mode,unsigned int addr, unsigned int data)
{
	//if code run in otp,when switching to ram_code,prog/pwe must be pulled down,pce/pclk may be pulled up.
	//when write to otp, ptm needs to be configured,prog/pwe/pce need to pull down.
	otp_start(ptm_mode);
	//tcsp
	sleep_us(Tcsp);
	//prog pas addr data
	reg_otp_ctrl0 |= FLD_OTP_PPROG ;
	reg_otp_ctrl0 |=FLD_OTP_PAS;
	reg_otp_pa = addr;
	reg_otp_wr_dat = data;
	reg_otp_paio = 0;
	//Tpps
	sleep_us(Tpps);
	//redundancy programming  38*2
	for(unsigned char i = 1; i <= 76; i++){
		reg_otp_ctrl0 |= FLD_OTP_PWE;
		sleep_us(Tpw);
		reg_otp_ctrl0 &= ~(FLD_OTP_PWE);
		if(i<38){
			 reg_otp_paio = i;
		}
		else if(i==38){
			reg_otp_ctrl0&=(~FLD_OTP_PAS);
			reg_otp_pa = addr;
			reg_otp_wr_dat = data;
			reg_otp_paio = 0;
		}
		else if((i>38)&&(i<76))
		{
			reg_otp_paio = i-38;
		}
		else if(i==76)
		{
			break;
		}
		//because the for loop and the if judge the time,choose to use Tpwi/2.
		 sleep_us(Tpwi/2);
	}
	sleep_us(Tpph);
	//prog =0
	reg_otp_ctrl0 &= ~(FLD_OTP_PPROG);
	sleep_us(5);
	//pce=0
	reg_otp_ctrl0 &= ~(FLD_OTP_PCE);
	//Tms >= 1(ns)
	reg_otp_ctrl1 = ~(FLD_OTP_PTM);
}
/**
 * @brief      This function serves to write data to OTP memory.
 *             the minimum unit of otp read-write operation is 4 bytes, that is a word. meanwhile, the otp cannot be burned repeatedly,
 *             this function is limited to writing only once,this function will determine if the otp is 0xffffffff, and if it is 0xffffffff,
 *             it will write the otp.
 * @param[in]  addr - the address of the data,it has to be a multiple of 4,the OTP memory that can access is from 0x0000-0x3ffc,can't access other address.
 * @param[in]  word_len  - the length of the data,the unit is word(4 bytes).
 * @param[in]  buff - data buff.
 * @return     0 :it means that the otp operation area is 0xffffffff or the write data,
 *                return 0 not mean that the burning was successful,need to use three kinds of read mode to check whether the writing was successful.
 *             1 :it means that there is an operation value in the operation area,it is not 0xffffffff or the write data,no burning action is performed.
 *
 * Attention: When the vbat voltage is greater than 3.3V, otp supply is 3.3V, if the vabt voltage is lower than 3.3V,
 * then the otp supply voltage will follow the vbat voltage value, write to otp, according to the datasheet,
 * the voltage value is at least 2.25V, if below the voltage value, you can not operate,and prompt adc sampling voltage has certain error,add by shuaixing.zhai, confirmed by baoyi 20211015.
 */
_attribute_ram_code_sec_noinline_ unsigned char otp_write(unsigned int addr,unsigned int word_len,unsigned int *buff)
{
	otp_pce_timeout_exceed();
	for(unsigned int i=0;i<word_len;i++)
	{
		unsigned int temp = 0;
		otp_read_cycle(OTP_PTM_READ,addr + i*4, 1, (unsigned int *)&temp);
		if(temp == 0xffffffff){
			otp_write32(OTP_PTM_PROG,addr + i*4,buff[i]);
	    }else if(temp != buff[i]){
	    	otp_auto_pce_restore();
		    return 1;
	    }

	}
	otp_auto_pce_restore();
	return 0;
}

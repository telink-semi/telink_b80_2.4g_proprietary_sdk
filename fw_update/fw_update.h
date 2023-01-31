/********************************************************************************************************
 * @file	fw_update.h
 *
 * @brief	This is the header file for b80
 *
 * @author	2.4G Group
 * @date	2021
 *
 * @par     Copyright (c) 2021, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
 *          All rights reserved.
 *
 *          Redistribution and use in source and binary forms, with or without
 *          modification, are permitted provided that the following conditions are met:
 *
 *              1. Redistributions of source code must retain the above copyright
 *              notice, this list of conditions and the following disclaimer.
 *
 *              2. Unless for usage inside a TELINK integrated circuit, redistributions
 *              in binary form must reproduce the above copyright notice, this list of
 *              conditions and the following disclaimer in the documentation and/or other
 *              materials provided with the distribution.
 *
 *              3. Neither the name of TELINK, nor the names of its contributors may be
 *              used to endorse or promote products derived from this software without
 *              specific prior written permission.
 *
 *              4. This software, with or without modification, must only be used with a
 *              TELINK integrated circuit. All other usages are subject to written permission
 *              from TELINK and different commercial license may apply.
 *
 *              5. Licensee shall be solely responsible for any claim to the extent arising out of or
 *              relating to such deletion(s), modification(s) or alteration(s).
 *
 *          THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *          ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *          WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *          DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
 *          DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *          (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *          LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *          ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *          (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *          SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************************************/
#ifndef _FW_UPDATE_H_
#define _FW_UPDATE_H_

#define FW_UPDATE_FRAME_TYPE_CMD        0x01
#define FW_UPDATE_FRAME_TYPE_DATA       0x02
#define FW_UPDATE_FRAME_TYPE_ACK        0x03

#define FW_UPDATE_CMD_ID_START_REQ      0x01
#define FW_UPDATE_CMD_ID_START_RSP      0x02
#define FW_UPDATE_CMD_ID_END_REQ        0x03
#define FW_UPDATE_CMD_ID_END_RSP        0x04
#define FW_UPDATE_CMD_ID_VERSION_REQ    0x05
#define FW_UPDATE_CMD_ID_VERSION_RSP    0x06

#define FW_UPDATE_FRAME_PAYLOAD_MAX     (2+64)
#define FW_UPDATE_RETRY_MAX             3
#define FW_APPEND_INFO_LEN              2 // FW_CRC 2 BYTE
#define FW_BOOT_ADDR                    0x7f000

#define FW_UPDATE_SLAVE_BIN_ADDR_20000  0x20000
#define FW_UPDATE_SLAVE_BIN_ADDR_40000  0x40000
#define FW_UPDATE_SLAVE_BIN_ADDR        FW_UPDATE_SLAVE_BIN_ADDR_20000

typedef struct {
    unsigned int FlashAddr;
    unsigned int TotalBinSize;
    unsigned short MaxBlockNum;
    unsigned short BlockNum;
    unsigned short FwVersion;
    unsigned short FwCRC;
    unsigned short PktCRC;
    unsigned short TargetFwCRC;
    unsigned char State;
    unsigned char RetryTimes;
    unsigned char FinishFlag;
} FW_UPDATE_CtrlTypeDef;

typedef struct {
    unsigned char CheckSum;
    unsigned char Type;
    unsigned short Len;
    unsigned char Payload[FW_UPDATE_FRAME_PAYLOAD_MAX];
} FW_UPDATE_FrameTypeDef;

enum {
    FW_UPDATE_MASTER_STATE_IDLE = 0,
    FW_UPDATE_MASTER_STATE_FW_VER_WAIT,
    FW_UPDATE_MASTER_STATE_START_RSP_WAIT,
    FW_UPDATE_MASTER_STATE_DATA_ACK_WAIT,
    FW_UPDATE_MASTER_STATE_END_RSP_WAIT,
    FW_UPDATE_MASTER_STATE_END,
    FW_UPDATE_MASTER_STATE_ERROR,
};

enum {
    FW_UPDATE_SLAVE_STATE_IDLE = 0,
    FW_UPDATE_SLAVE_STATE_FW_VERSION_READY,
    FW_UPDATE_SLAVE_STATE_START_READY,
    FW_UPDATE_SLAVE_STATE_DATA_READY,
    FW_UPDATE_SLAVE_STATE_END_READY,
    FW_UPDATE_SLAVE_STATE_END,
    FW_UPDATE_SLAVE_STATE_ERROR
};

enum {
    FW_UPDATE_MSG_TYPE_INVALID_DATA = 0,
    FW_UPDATE_MSG_TYPE_DATA,
    FW_UPDATE_MSG_TYPE_TIMEOUT,
};

extern void FW_UPDATE_MasterInit(unsigned int FWBinAddr, unsigned short FwVer);
extern void FW_UPDATE_MasterStart(void);

extern void FW_UPDATE_SlaveInit(unsigned int FWBinAddr, unsigned short FwVer);
extern void FW_UPDATE_SlaveStart(void);

extern void FW_UPDATE_RxIrq(unsigned char *Data);

#endif /*_FW_UPDATE_H_*/

/********************************************************************************************************
 * @file    tl_esb_ll_util.h
 *
 * @brief   This is the header file for b80
 *
 * @author  2.4G Group
 * @date    2022
 *
 * @par     Copyright (c) 2022, Telink Semiconductor (Shanghai) Co., Ltd. ("TELINK")
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

#ifndef TL_ESB_LL_UTIL_H_
#define TL_ESB_LL_UTIL_H_
#include "static_assert.h"

#define TRF_RETVAL_CHECK(expression)    \
do {if (!(expression)) return 1;} while (0)

#define TRF_LEN_VA_ARGS_CAL(                                    \
    _0,  _1, _2, _3, _4, _5, _6,  _7,  _8,  _9,  _10,           \
    _11, _12, _13, _14, _15, _16, _17, _18, _19, _20,           \
    _21, _22, _23, _24, _25, _26, _27, _28, _29, _30,           \
    _31, _32, _33, _34, _35, _36, _37, _38, _39, _40,           \
    _41, _42, _43, _44, _45, _46, _47, _48, _49, _50,           \
    _51, _52, _53, _54, _55, _56, _57, _58, _59, _60,           \
    _61, _62, _63, _64, _65, _66, _67, _68, _69, _70,           \
    _71, _72, _73, _74, _75, _76, _77, _78, _79, _80,           \
    _81, _82, _83, _84, _85, _86, _87, _88, _89, _90,           \
    _91, _92, _93, _94, _95, _96, _97, _98, _99, _100,          \
    _101,_102,_103,_104,_105,_106,_107,_108,_109,_110,          \
    _111,_112,_113,_114,_115,_116,_117,_118,_119,_120,          \
    _121,_122,_123,_124,_125,_126,_127, N, ...) N

#define TRF_NUM_VA_ARGS(...) TRF_LEN_VA_ARGS_CAL(__VA_ARGS__,   \
    128, 127, 126, 125, 124, 123, 122, 121, 120, 119,           \
    118, 117, 116, 115, 114, 113, 112, 111, 110, 109,           \
    108, 107, 106, 105, 104, 103, 102, 101, 100, 99,            \
    98,  97,  96,  95,  94,  93,  92,  91,  90,  89,            \
    88,  87,  86,  85,  84,  83,  82,  81,  80,  79,            \
    78,  77,  76,  75,  74,  73,  72,  71,  70,  69,            \
    68,  67,  66,  65,  64,  63,  62,  61,  60,  59,            \
    58,  57,  56,  55,  54,  53,  52,  51,  50,  49,            \
    48,  47,  46,  45,  44,  43,  42,  41,  40,  39,            \
    38,  37,  36,  35,  34,  33,  32,  31,  30,  29,            \
    28,  27,  26,  25,  24,  23,  22,  21,  20,  19,            \
    18,  17,  16,  15,  14,  13,  12,  11,  10,                 \
    9,   8,   7,   6,   5,   4,   3,   2,   1, 0)


#endif /* TL_ESB_LL_UTIL_H_ */

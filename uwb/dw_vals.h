///*! ------------------------------------------------------------------------------------------------------------------
// * @file    deca_vals.h
// * @brief   DW3000 Register Definitions
// *          This file supports assembler and C development for DW3000 enabled devices
// *
// * @attention
// *
// * Copyright 2013-2020 (c) Decawave Ltd, Dublin, Ireland.
// *
// * All rights reserved.
// *
// */
//

#ifndef _DW3000_VALS_H_
#define _DW3000_VALS_H_

#define IP_CONFIG_LO_SCP        0x0306
#define IP_CONFIG_HI_SCP        0x00000000
#define STS_CONFIG_LO_SCP       0x000C5A0A
#define STS_CONFIG_HI_SCP       0x7D

#define PD_THRESH_NO_DATA       0xAF5F35CC      /* PD threshold for no data STS mode*/
#define PD_THRESH_DEFAULT       0xAF5F584C

#define RF_TXCTRL_CH5           0x1C071134UL
#define RF_RXCTRL_CH9           0x08B5A833UL
#define RF_TXCTRL_CH9           0x1C010034UL
#define RF_PLL_CFG_CH9          0x0F3C
#define RF_PLL_CFG_CH5          0x1F3C
#define LDO_RLOAD_VAL_B1        0x14
#define RF_TXCTRL_LO_B2         0x0E

#define DWT_DGC_CFG             0x32
#define RF_PLL_CFG_LD           0x81

#endif//_DW3000_VALS_H_

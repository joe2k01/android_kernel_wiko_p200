/*
 * Copyright (c) 2015-2017 MICROTRUST Incorporated
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __FP_VENDOR_H__
#define __FP_VENDOR_H__
#include <linux/types.h>

#define MAX_TA_NAME 64

// if add a new id , sould sort by alphabetic
 enum {
    FP_VENDOR_BETTERLIFE,
    FP_VENDOR_BIOSEC,
    FP_VENDOR_CDFINGER,
    FP_VENDOR_CHIPONE,
    FP_VENDOR_CHIPSAILING,
    FP_VENDOR_EGIS,
    FP_VENDOR_ELAN,
    FP_VENDOR_FINCHOS,
    FP_VENDOR_FOCALTECH,
    FP_VENDOR_FPC,
    FP_VENDOR_GOODIX,
    FP_VENDOR_HENGZHI,
    FP_VENDOR_HOLITECH,
    FP_VENDOR_IMAGING,
    FP_VENDOR_LEADCORETECH,
    FP_VENDOR_MICROARRAY,
    FP_VENDOR_METRICS,
    FP_VENDOR_NOVATECH,
    FP_VENDOR_MSTAR,
    FP_VENDOR_RISKSTORM,
    FP_VENDOR_SILEAD,
    FP_VENDOR_SYNAPTICS,
    FP_VENDOR_SUNWAVE,
    FP_VENDOR_SUNRISE,
    FP_VENDOR_MAX
};

void set_fp_vendor(uint8_t fp_vendor_id);
uint8_t get_fp_vendor(void);
void get_fp_ta_load_path(char* fp_ta_load_path);
/*
 * return 1 if fp driver can call spi in ree, else return 0
*/
int get_fp_spi_enable(void);

extern int fp_spi_enable;

#endif  /*__FP_VENDOR_H__*/

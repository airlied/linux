/* SPDX-License-Identifier: MIT */

/* Copyright 2024 Advanced Micro Devices, Inc. */

#ifndef __SPL_FIXED31_32_H__
#define __SPL_FIXED31_32_H__

#include "dc/os_types.h"
#include "include/fixed31_32.h"

#define spl_fixed31_32 fixed31_32
#define spl_fixpt_mul dc_fixpt_mul
#define spl_fixpt_from_fraction dc_fixpt_from_fraction
#define spl_fixpt_u0d19 dc_fixpt_u0d19
#define spl_fixpt_u3d19 dc_fixpt_u3d19
#define spl_fixpt_le dc_fixpt_le

#define spl_fixpt_add dc_fixpt_add
#define spl_fixpt_mul dc_fixpt_mul
#define spl_fixpt_div dc_fixpt_div

#define spl_fixpt_from_int dc_fixpt_from_int
#define spl_fixpt_add_int dc_fixpt_add_int
#define spl_fixpt_mul_int dc_fixpt_mul_int
#define spl_fixpt_div_int dc_fixpt_div_int

#define spl_fixpt_min dc_fixpt_min
#define spl_fixpt_one dc_fixpt_one
#define spl_fixpt_zero dc_fixpt_zero
#define spl_fixpt_ceil dc_fixpt_ceil
#define spl_fixpt_floor dc_fixpt_floor
#define spl_fixpt_recip dc_fixpt_recip
#define spl_fixpt_truncate dc_fixpt_truncate
#define spl_fixpt_round dc_fixpt_round

#endif

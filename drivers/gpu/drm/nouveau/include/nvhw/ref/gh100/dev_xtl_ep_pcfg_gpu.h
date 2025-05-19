/*
 * SPDX-FileCopyrightText: Copyright (c) 2003-2023 NVIDIA CORPORATION & AFFILIATES
 * SPDX-License-Identifier: MIT
 */

#ifndef __gh100_dev_xtl_ep_pcfg_gpu_h__
#define __gh100_dev_xtl_ep_pcfg_gpu_h__
#define NV_EP_PCFG_GPU_DEVICE_CONTROL_STATUS                                                                   0x00000068           /* RW-4R */
#define NV_EP_PCFG_GPU_DEVICE_CONTROL_STATUS_ENABLE_RELAXED_ORDERING                                           4:4                  /* RWIVF */
#define NV_EP_PCFG_GPU_DEVICE_CONTROL_STATUS_ENABLE_RELAXED_ORDERING_INIT                                      0x00000001           /* RWI-V */
#define NV_EP_PCFG_GPU_DEVICE_CONTROL_STATUS_EXTENDED_TAG_FIELD_ENABLE                                         8:8                  /* RWIVF */
#define NV_EP_PCFG_GPU_DEVICE_CONTROL_STATUS_EXTENDED_TAG_FIELD_ENABLE_INIT                                    0x00000001           /* RWI-V */
#define NV_EP_PCFG_GPU_DEVICE_CONTROL_STATUS_ENABLE_NO_SNOOP                                                   11:11                /* RWIVF */
#define NV_EP_PCFG_GPU_DEVICE_CONTROL_STATUS_INITIATE_FN_LVL_RST                                               15:15                /* RWIVF */
#define NV_EP_PCFG_GPU_DEVICE_CONTROL_STATUS_CORR_ERROR_DETECTED                                               16:16                /* RWIVF */
#define NV_EP_PCFG_GPU_DEVICE_CONTROL_STATUS_NON_FATAL_ERROR_DETECTED                                          17:17                /* RWIVF */
#define NV_EP_PCFG_GPU_DEVICE_CONTROL_STATUS_FATAL_ERROR_DETECTED                                              18:18                /* RWIVF */
#define NV_EP_PCFG_GPU_DEVICE_CONTROL_STATUS_UNSUPP_REQUEST_DETECTED                                           19:19                /* RWIVF */
#define NV_EP_PCFG_GPU_DEVICE_CONTROL_STATUS_TRANSACTIONS_PENDING                                              21:21                /* R-IVF */

#endif // __gh100_dev_xtl_ep_pcfg_gpu_h__

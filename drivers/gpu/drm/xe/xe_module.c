/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#include <linux/init.h>
#include <linux/module.h>

#include "xe_drv.h"

static int __init xe_init(void)
{
	return 0;
}

static void __exit xe_exit(void)
{
}

module_init(xe_init);
module_exit(xe_exit);

MODULE_AUTHOR("Intel Corporation");

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL and additional rights");

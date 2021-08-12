/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2021 Intel Corporation
 */

#include <linux/init.h>
#include <linux/module.h>

#include "xe_drv.h"
#include "xe_pci.h"

static int __init xe_init(void)
{
	return i915_register_pci_driver();
}

static void __exit xe_exit(void)
{
	i915_unregister_pci_driver();
}

module_init(xe_init);
module_exit(xe_exit);

MODULE_AUTHOR("Intel Corporation");

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL and additional rights");

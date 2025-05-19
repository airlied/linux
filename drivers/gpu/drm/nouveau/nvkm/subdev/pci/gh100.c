/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2025, NVIDIA CORPORATION. All rights reserved.
 */
#include "priv.h"

#include <nvhw/drf.h>
#include <nvhw/ref/gh100/dev_xtl_ep_pri.h>
#include <nvhw/ref/gh100/dev_xtl_ep_pcfg_gpu.h>

static void
gh100_pci_disable_snoop(struct nvkm_pci *pci)
{
	u32 reg_val = nvkm_pci_rd32(pci, NV_EP_PCFG_GPU_DEVICE_CONTROL_STATUS);

	reg_val |= NVVAL_X(NV_EP_PCFG_GPU_DEVICE_CONTROL_STATUS_ENABLE_NO_SNOOP, 1);
	nvkm_pci_wr32(pci, NV_EP_PCFG_GPU_DEVICE_CONTROL_STATUS, reg_val);
}

static void
gh100_pci_msi_rearm(struct nvkm_pci *pci)
{
	/* Handled by top-level intr ACK. */
}

static void
gh100_pci_init(struct nvkm_pci *pci)
{
	g84_pci_init(pci);
	gh100_pci_disable_snoop(pci);
}

static const struct nvkm_pci_func
gh100_pci = {
	.cfg = {
		.addr = DRF_LO(NV_EP_PCFGM),
		.size = DRF_HI(NV_EP_PCFGM) - DRF_LO(NV_EP_PCFGM) + 1,
	},
	.msi_rearm = gh100_pci_msi_rearm,
	.init = gh100_pci_init,
};

int
gh100_pci_new(struct nvkm_device *device, enum nvkm_subdev_type type, int inst,
	      struct nvkm_pci **ppci)
{
	/* gh100 always disables snoop */
	return nvkm_pci_new_(&gh100_pci, device, type, inst, ppci);
}

/*
 * Copyright 2022 Red Hat Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */
#include "gf100.h"

#include <core/mm.h>
#include <subdev/gsp.h>
#include <subdev/instmem.h>
#include <subdev/mmu/vmm.h>

static void
r515_bar_bar2_wait(struct nvkm_bar *base)
{
}

static void
r515_bar_bar2_fini(struct nvkm_bar *bar)
{
	struct nvkm_gsp *gsp = bar->subdev.device->gsp;

	gsp->subdev.device->mmu->ready = false;
	WARN_ON(gsp->rpc->update_bar_pde(gsp, 2, 0));
}

static void
r515_bar_bar2_init(struct nvkm_bar *bar)
{
	struct nvkm_device *device = bar->subdev.device;
	struct nvkm_vmm *vmm = gf100_bar(bar)->bar[0].vmm;
	struct nvkm_gsp *gsp = device->gsp;

	WARN_ON(gsp->rpc->update_bar_pde(gsp, 2, vmm->pd->pde[0]->pt[0]->addr));
	vmm->rm_bar2_pdb = gsp->bar.rm_bar2_pdb;
	device->mmu->ready = true;
}

static void
r515_bar_bar1_wait(struct nvkm_bar *base)
{
}

static void
r515_bar_bar1_fini(struct nvkm_bar *base)
{
}

#define wrap(p) container_of((p), struct wrap, memory)

struct wrap {
	struct nvkm_memory memory;
	struct nvkm_device *device;
	u64 addr;
	u64 size;
};

static int
wrap_kmap(struct nvkm_memory *memory, struct nvkm_memory **pmemory)
{
	return nvkm_instobj_wrap(wrap(memory)->device, memory, pmemory);
}

static int
wrap_map(struct nvkm_memory *memory, u64 offset, struct nvkm_vmm *vmm,
	      struct nvkm_vma *vma, void *argv, u32 argc)
{
	struct wrap *wrap = wrap(memory);
	struct nvkm_mm_node mn = { .offset = wrap->addr >> 12, .length = wrap->size >> 12 };
	struct nvkm_vmm_map map = {
		.memory = &wrap->memory,
		.offset = offset,
		.mem = &mn,
	};

	return nvkm_vmm_map(vmm, vma, argv, argc, &map);
}

static u64
wrap_size(struct nvkm_memory *memory)
{
	return wrap(memory)->size;
}

static u64
wrap_addr(struct nvkm_memory *memory)
{
	return wrap(memory)->addr;
}

static u8
wrap_page(struct nvkm_memory *memory)
{
	return 12;
}

static enum nvkm_memory_target
wrap_target(struct nvkm_memory *memory)
{
	return NVKM_MEM_TARGET_VRAM;
}

static void *
wrap_dtor(struct nvkm_memory *memory)
{
	return wrap(memory);
}

static const struct nvkm_memory_func
wrap_func = {
	.dtor = wrap_dtor,
	.target = wrap_target,
	.page = wrap_page,
	.addr = wrap_addr,
	.size = wrap_size,
	.map = wrap_map,
	.kmap = wrap_kmap,
};

static int
wrap_vram(struct nvkm_device *device, u64 addr, u64 size, struct nvkm_memory **pmemory)
{
	struct wrap *wrap;

	if (!(wrap = kzalloc(sizeof(*wrap), GFP_KERNEL)))
		return -ENOMEM;

	nvkm_memory_ctor(&wrap_func, &wrap->memory);
	wrap->device = device;
	wrap->addr = addr;
	wrap->size = size;

	*pmemory = &wrap->memory;
	return 0;
}

static void
r515_bar_bar1_init(struct nvkm_bar *bar)
{
	struct nvkm_device *device = bar->subdev.device;
	struct nvkm_gsp *gsp = device->gsp;
	struct nvkm_vmm *vmm = gf100_bar(bar)->bar[1].vmm;
	struct nvkm_memory *pd3;
	int ret;

	ret = wrap_vram(device, gsp->bar.rm_bar1_pdb, 0x1000, &pd3);
	if (WARN_ON(ret))
		return;

	nvkm_memory_unref(&vmm->pd->pt[0]->memory);

	ret = nvkm_memory_kmap(pd3, &vmm->pd->pt[0]->memory);
	nvkm_memory_unref(&pd3);
	if (WARN_ON(ret))
		return;

	vmm->pd->pt[0]->addr = nvkm_memory_addr(vmm->pd->pt[0]->memory);
}

static void *
r515_bar_dtor(struct nvkm_bar *bar)
{
	void *data = gf100_bar_dtor(bar);

	kfree(bar->func);
	return data;
}

int
r515_bar_new_(const struct nvkm_bar_func *hw, struct nvkm_device *device,
	      enum nvkm_subdev_type type, int inst, struct nvkm_bar **pbar)
{
	struct nvkm_bar_func *rm;
	int ret;

	if (!(rm = kzalloc(sizeof(*rm), GFP_KERNEL)))
		return -ENOMEM;

	rm->dtor = r515_bar_dtor;
	rm->oneinit = hw->oneinit;
	rm->bar1.init = r515_bar_bar1_init;
	rm->bar1.fini = r515_bar_bar1_fini;
	rm->bar1.wait = r515_bar_bar1_wait;
	rm->bar1.vmm = hw->bar1.vmm;
	rm->bar2.init = r515_bar_bar2_init;
	rm->bar2.fini = r515_bar_bar2_fini;
	rm->bar2.wait = r515_bar_bar2_wait;
	rm->bar2.vmm = hw->bar2.vmm;
	rm->flush = NULL;

	ret = gf100_bar_new_(rm, device, type, inst, pbar);
	if (ret)
		kfree(rm);

	return ret;
}

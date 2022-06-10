/*
 * Copyright 2021 Red Hat Inc.
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
#include "priv.h"

#include "fw/booter/load/ga102.h"
#include "fw/booter/unload/ga102.h"
#include "fw/gsp-rm/boot/ga102.h"

static int
ga102_gsp_reset(struct nvkm_gsp *gsp)
{
	int ret;

	ret = gsp->falcon.func->reset_eng(&gsp->falcon);
	if (ret)
		return ret;

	nvkm_falcon_mask(&gsp->falcon, 0x1668, 0x00000111, 0x00000111);
	return 0;
}

static int
ga102_gsp_booter_ctor(struct nvkm_gsp *gsp, const char *name,
		      const struct nvkm_gsp_booter_fw *booter, struct nvkm_falcon *falcon,
		      struct nvkm_falcon_fw *fw)
{
	struct nvkm_device *device = gsp->subdev.device;
	const struct nvkm_gsp_booter_fw_hdr *hdr = (void *)booter->hdr.data;
	int ret;

	ret = nvkm_falcon_fw_ctor(&ga102_flcn_fw, name, device, true,
				  booter->img.data, booter->img.size, falcon, fw);
	if (WARN_ON(ret))
		return ret;

	fw->imem_base_img = hdr->app_code_offset;
	fw->imem_base = 0;
	fw->imem_size = hdr->app_code_size;
	fw->dmem_base_img = hdr->os_data_offset;
	fw->dmem_base = 0;
	fw->dmem_size = hdr->os_data_size;
	fw->dmem_sign = booter->patch_loc - fw->dmem_base_img;
	fw->boot_addr = hdr->app_code_offset;
	fw->fuse_ver = booter->fuse_ver;
	fw->ucode_id = booter->ucode_id;
	fw->engine_id = booter->engine_id;
	return 0;
}

static const struct nvkm_falcon_func
ga102_gsp_flcn = {
	.disable = gm200_flcn_disable,
	.enable = gm200_flcn_enable,
	.select = ga102_flcn_select,
	.addr2 = 0x1000,
	.reset_eng = gp102_flcn_reset_eng,
	.reset_prep = ga102_flcn_reset_prep,
	.reset_wait_mem_scrubbing = ga102_flcn_reset_wait_mem_scrubbing,
	.imem_dma = &ga102_flcn_dma,
	.dmem_dma = &ga102_flcn_dma,
	.riscv_active = ga102_flcn_riscv_active,
};

static const struct nvkm_gsp_func
ga102_gsp_r515_48_07 = {
	.flcn = &ga102_gsp_flcn,
	.fwsec = &ga102_flcn_fw,

	.booter.ctor = ga102_gsp_booter_ctor,
	.booter.load = &ga102_gsp_booter_load_fw,
	.booter.unload = &ga102_gsp_booter_unload_fw,

	.boot = &ga102_gsp_gsp_rm_boot_fw,

	.dtor = r515_gsp_dtor,
	.oneinit = tu102_gsp_oneinit,
	.init = r515_gsp_init,
	.fini = r515_gsp_fini,
	.reset = ga102_gsp_reset,
};

static const struct nvkm_gsp_func
ga102_gsp = {
	.flcn = &ga102_gsp_flcn,
};

struct nvkm_gsp_fwif
ga102_gsps[] = {
	{ 5154807,  r515_gsp_load, &ga102_gsp_r515_48_07, ".fwsignature_ga10x" },
	{      -1, gv100_gsp_nofw, &ga102_gsp },
	{}
};

int
ga102_gsp_new(struct nvkm_device *device, enum nvkm_subdev_type type, int inst,
	      struct nvkm_gsp **pgsp)
{
	return nvkm_gsp_new_(ga102_gsps, device, type, inst, pgsp);
}

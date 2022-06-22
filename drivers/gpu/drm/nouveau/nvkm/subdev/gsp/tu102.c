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
#include "priv.h"

#include <subdev/fb.h>

#include <nvfw/flcn.h>

#include "fw/booter/load/tu102.h"
#include "fw/booter/unload/tu102.h"
#include "fw/gsp-rm/boot/tu102.h"

int
tu102_gsp_booter_ctor(struct nvkm_gsp *gsp, const char *name,
		      const struct nvkm_gsp_booter_fw *booter, struct nvkm_falcon *falcon,
		      struct nvkm_falcon_fw *fw)
{
	struct nvkm_device *device = gsp->subdev.device;
	const struct nvkm_gsp_booter_fw_hdr *hdr = (void *)booter->hdr.data;
	int ret;

	ret = nvkm_falcon_fw_ctor(&gm200_flcn_fw, name, device, false,
				  booter->img.data, booter->img.size, falcon, fw);
	if (WARN_ON(ret))
		return ret;

	fw->nmem_base_img = 0;
	fw->nmem_base = hdr->os_code_offset;
	fw->nmem_size = hdr->os_code_size;
	fw->imem_base_img = fw->nmem_size;
	fw->imem_base = hdr->app_code_offset;
	fw->imem_size = hdr->app_code_size;
	fw->dmem_base_img = hdr->os_data_offset;
	fw->dmem_base = 0;
	fw->dmem_size = hdr->os_data_size;
	fw->dmem_sign = booter->patch_loc - fw->dmem_base_img;
	fw->boot_addr = hdr->os_code_offset;
	return 0;
}

static int
tu102_gsp_fwsec_load_bld(struct nvkm_falcon_fw *fw)
{
	struct flcn_bl_dmem_desc_v2 desc = {
		.ctx_dma = FALCON_DMAIDX_PHYS_SYS_NCOH,
		.code_dma_base = fw->fw.phys,
		.non_sec_code_off = fw->nmem_base,
		.non_sec_code_size = fw->nmem_size,
		.sec_code_off = fw->imem_base,
		.sec_code_size = fw->imem_size,
		.code_entry_point = 0,
		.data_dma_base = fw->fw.phys + fw->dmem_base_img,
		.data_size = fw->dmem_size,
		.argc = 0,
		.argv = 0,
	};

	flcn_bl_dmem_desc_v2_dump(fw->falcon->user, &desc);

	nvkm_falcon_mask(fw->falcon, 0x600 + desc.ctx_dma * 4, 0x00000007, 0x00000005);

	return nvkm_falcon_pio_wr(fw->falcon, (u8 *)&desc, 0, 0, DMEM, 0, sizeof(desc), 0, 0);
}

const struct nvkm_falcon_fw_func
tu102_gsp_fwsec = {
	.reset = gm200_flcn_fw_reset,
	.load = gm200_flcn_fw_load,
	.load_bld = tu102_gsp_fwsec_load_bld,
	.boot = gm200_flcn_fw_boot,
};

int
tu102_gsp_reset(struct nvkm_gsp *gsp)
{
	return gsp->falcon.func->reset_eng(&gsp->falcon);
}

static u64
tu102_gsp_vga_workspace_addr(struct nvkm_gsp *gsp, u64 fb_size)
{
	struct nvkm_device *device = gsp->subdev.device;
	const u64 base = fb_size - 0x100000;
	u64 addr = 0;

	if (device->disp)
		addr = nvkm_rd32(gsp->subdev.device, 0x625f04);
	if (!(addr & 0x00000008))
		return base;

	addr = (addr & 0xffffff00) << 16;
	if (addr < base)
		return fb_size - 0x20000;

	return addr;
}

int
tu102_gsp_oneinit(struct nvkm_gsp *gsp)
{
	gsp->fb.size = nvkm_fb_vidmem_size(gsp->subdev.device);

	gsp->fb.bios.vga_workspace.addr = tu102_gsp_vga_workspace_addr(gsp, gsp->fb.size);
	gsp->fb.bios.vga_workspace.size = gsp->fb.size - gsp->fb.bios.vga_workspace.addr;
	gsp->fb.bios.addr = gsp->fb.bios.vga_workspace.addr;
	gsp->fb.bios.size = gsp->fb.bios.vga_workspace.size;

	return r515_gsp_oneinit(gsp);
}

const struct nvkm_falcon_func
tu102_gsp_flcn = {
	.disable = gm200_flcn_disable,
	.enable = gm200_flcn_enable,
	.addr2 = 0x1000,
	.reset_eng = gp102_flcn_reset_eng,
	.reset_wait_mem_scrubbing = gm200_flcn_reset_wait_mem_scrubbing,
	.bind_inst = gm200_flcn_bind_inst,
	.bind_stat = gm200_flcn_bind_stat,
	.bind_intr = true,
	.imem_pio = &gm200_flcn_imem_pio,
	.dmem_pio = &gm200_flcn_dmem_pio,
	.riscv_active = tu102_flcn_riscv_active,
};

static const struct nvkm_gsp_func
tu102_gsp_r515_48_07 = {
	.flcn = &tu102_gsp_flcn,
	.fwsec = &tu102_gsp_fwsec,

	.booter.ctor = tu102_gsp_booter_ctor,
	.booter.load = &tu102_gsp_booter_load_fw,
	.booter.unload = &tu102_gsp_booter_unload_fw,

	.boot = &tu102_gsp_gsp_rm_boot_fw,

	.dtor = r515_gsp_dtor,
	.oneinit = tu102_gsp_oneinit,
	.init = r515_gsp_init,
	.fini = r515_gsp_fini,
	.reset = tu102_gsp_reset,

	.rpc = &r515_gsp_rpc,
	.client = &r515_gsp_client,
};

static struct nvkm_gsp_fwif
tu102_gsps[] = {
	{ 5154807,  r515_gsp_load, &tu102_gsp_r515_48_07, ".fwsignature_tu10x" },
	{      -1, gv100_gsp_nofw, &gv100_gsp },
	{}
};

int
tu102_gsp_new(struct nvkm_device *device, enum nvkm_subdev_type type, int inst,
	      struct nvkm_gsp **pgsp)
{
	return nvkm_gsp_new_(tu102_gsps, device, type, inst, pgsp);
}

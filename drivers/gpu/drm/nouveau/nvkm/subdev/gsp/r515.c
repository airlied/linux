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

#include <core/pci.h> //XXX
#include <subdev/timer.h>
#include <engine/sec2.h>

#include "fw/msgq.h"
#include "fw/static.h"
#include "fw/rmgspseq.h"

#define GSP_PAGE_SHIFT 12
#define GSP_PAGE_SIZE  BIT(GSP_PAGE_SHIFT)

struct r515_gsp_msg {
	u32 checksum;
	u32 sequence;
	u8  data[];
};

static void *
r515_gsp_msgq_wait(struct nvkm_gsp *gsp, u32 repc, u32 *prepc, int *ptime)
{
	struct r515_gsp_msg *mqe;
	u32 size, rptr = *gsp->msgq.rptr;
	int used;
	u8 *msg, *src;
	u32 off = 0, len;

	size = DIV_ROUND_UP(sizeof(*mqe) + repc, GSP_PAGE_SIZE);
	if (WARN_ON(!size || size >= gsp->msgq.cnt))
		return ERR_PTR(-EINVAL);

	do {
		u32 wptr = *gsp->msgq.wptr;

		used = wptr + gsp->msgq.cnt - rptr;
		if (used >= gsp->msgq.cnt)
			used -= gsp->msgq.cnt;
		if (used >= size)
			break;

		usleep_range(1, 2);
	} while (--(*ptime));

	if (WARN_ON(!*ptime))
		return ERR_PTR(-ETIMEDOUT);

	mqe = (void *)((u8 *)gsp->shm.msgq.ptr + 0x1000 + rptr * 0x1000);
	src = mqe->data;

	if (prepc) {
		*prepc = (used * GSP_PAGE_SIZE) - sizeof(*mqe);
		return src;
	}

	msg = kvmalloc(repc, GFP_KERNEL);
	if (!msg)
		return ERR_PTR(-ENOMEM);

	do {
		len = min_t(u32, repc, (gsp->msgq.cnt - rptr) * GSP_PAGE_SIZE);

		memcpy(msg + off, src, len);
		src += len;
		off += len;

		rptr += DIV_ROUND_UP(len, GSP_PAGE_SIZE);
		if (rptr == gsp->msgq.cnt) {
			rptr = 0;
			src = (void *)((u8 *)gsp->shm.msgq.ptr + 0x1000 + rptr * 0x1000);
		}

		repc -= len;
	} while(repc);

	mb();
	(*gsp->msgq.rptr) = rptr;
	return msg;
}

static void *
r515_gsp_msgq_recv(struct nvkm_gsp *gsp, u32 repc, int *ptime)
{
	return r515_gsp_msgq_wait(gsp, repc, NULL, ptime);
}

static int
r515_gsp_cmdq_push(struct nvkm_gsp *gsp, void *argv)
{
	struct r515_gsp_msg *cmd = container_of(argv, typeof(*cmd), data);
	struct r515_gsp_msg *cqe;
	u32 argc = cmd->checksum;
	u64 *ptr = (void *)cmd;
	u64 *end = (void *)cmd->data + argc;
	u64 csum = 0;
	int free, time = 1000000;
	u32 wptr, size;
	u32 off = 0;

	cmd->checksum = 0;
	cmd->sequence = gsp->cmdq.seq++;
	while (ptr < end)
		csum ^= *ptr++;

	cmd->checksum = upper_32_bits(csum) ^ lower_32_bits(csum);

	argc = sizeof(*cmd) + argc;
	wptr = *gsp->cmdq.wptr;
	do {
		do {
			free = *gsp->cmdq.rptr + gsp->cmdq.cnt - wptr - 1;
			if (free >= gsp->cmdq.cnt)
				free -= gsp->cmdq.cnt;
			if (free >= 1)
				break;

			usleep_range(1, 2);
		} while(--time);

		if (WARN_ON(!time)) {
			kvfree(cmd);
			return -ETIMEDOUT;
		}

		cqe = (void *)((u8 *)gsp->shm.cmdq.ptr + 0x1000 + wptr * 0x1000);
		size = min_t(u32, argc, (gsp->cmdq.cnt - wptr) * GSP_PAGE_SIZE);
		memcpy(cqe, (u8 *)cmd + off, size);

		if (++wptr == gsp->cmdq.cnt)
			wptr = 0;

		off  += size;
		argc -= size;
	} while(argc);

	nvkm_trace(&gsp->subdev, "cmdq: wptr %d\n", wptr);
	wmb();
	(*gsp->cmdq.wptr) = wptr;
	mb();

	nvkm_falcon_wr32(&gsp->falcon, 0xc00, 0x00000000);

	kvfree(cmd);
	return 0;
}

static void *
r515_gsp_cmdq_get(struct nvkm_gsp *gsp, u32 argc)
{
	struct r515_gsp_msg *cmd;

	cmd = kvzalloc(ALIGN(sizeof(*cmd) + argc, sizeof(u64)), GFP_KERNEL);
	if (!cmd)
		return ERR_PTR(-ENOMEM);

	cmd->checksum = argc;
	return cmd->data;
}

struct nvfw_gsp_rpc {
	u32 header_version;
	u32 signature;
	u32 length;
	u32 function;
	u32 rpc_result;
	u32 rpc_result_private;
	u32 sequence;
	union {
		u32 spare;
		u32 cpuRmGfid;
	};
	u8  data[];
};

static void
r515_gsp_msg_done(struct nvkm_gsp *gsp, struct nvfw_gsp_rpc *msg)
{
	kvfree(msg);
}

static void
r515_gsp_msg_dump(struct nvkm_gsp *gsp, struct nvfw_gsp_rpc *msg, int lvl)
{
	if (gsp->subdev.debug >= lvl) {
		nvkm_printk__(&gsp->subdev, lvl, info,
			      "msg fn:%d len:0x%x/0x%zx res:0x%x resp:0x%x\n",
			      msg->function, msg->length, msg->length - sizeof(*msg),
			      msg->rpc_result, msg->rpc_result_private);
		print_hex_dump(KERN_INFO, "msg: ", DUMP_PREFIX_OFFSET, 16, 1,
			       msg->data, msg->length - sizeof(*msg), true);
	}
}

static struct nvfw_gsp_rpc *
r515_gsp_msg_recv(struct nvkm_gsp *gsp, int fn, u32 repc)
{
	struct nvkm_subdev *subdev = &gsp->subdev;
	struct nvfw_gsp_rpc *msg;
	int time = 4000000, i;
	u32 size;

retry:
	msg = r515_gsp_msgq_wait(gsp, sizeof(*msg), &size, &time);
	if (IS_ERR_OR_NULL(msg))
		return msg;

	msg = r515_gsp_msgq_recv(gsp, msg->length, &time);
	if (IS_ERR_OR_NULL(msg))
		return msg;

	r515_gsp_msg_dump(gsp, msg, NV_DBG_TRACE);

	if (msg->rpc_result) {
		r515_gsp_msg_dump(gsp, msg, NV_DBG_ERROR);
		r515_gsp_msg_done(gsp, msg);
		return ERR_PTR(-EINVAL);
	}

	if (fn && msg->function == fn) {
		if (repc) {
			if (msg->length != sizeof(*msg) + repc) {
				nvkm_error(subdev, "msg len %d != %zd\n",
					   msg->length, sizeof(*msg) + repc);
				r515_gsp_msg_dump(gsp, msg, NV_DBG_ERROR);
				r515_gsp_msg_done(gsp, msg);
				return ERR_PTR(-EIO);
			}

			return msg;
		}

		r515_gsp_msg_done(gsp, msg);
		return NULL;
	}

	for (i = 0; i < gsp->msgq.ntfy_nr; i++) {
		struct nvkm_gsp_msgq_ntfy *ntfy = &gsp->msgq.ntfy[i];

		if (ntfy->fn == msg->function) {
			ntfy->func(ntfy->priv, ntfy->fn, msg->data, msg->length - sizeof(*msg));
			break;
		}
	}

	if (i == gsp->msgq.ntfy_nr)
		r515_gsp_msg_dump(gsp, msg, NV_DBG_WARN);

	r515_gsp_msg_done(gsp, msg);
	if (fn)
		goto retry;

	return NULL;
}

static int
r515_gsp_msg_ntfy_add(struct nvkm_gsp *gsp, u32 fn, nvkm_gsp_msg_ntfy_func func, void *priv)
{
	int ret = 0;

	mutex_lock(&gsp->msgq.mutex);
	if (WARN_ON(gsp->msgq.ntfy_nr >= ARRAY_SIZE(gsp->msgq.ntfy))) {
		ret = -ENOSPC;
	} else {
		gsp->msgq.ntfy[gsp->msgq.ntfy_nr].fn = fn;
		gsp->msgq.ntfy[gsp->msgq.ntfy_nr].func = func;
		gsp->msgq.ntfy[gsp->msgq.ntfy_nr].priv = priv;
		gsp->msgq.ntfy_nr++;
	}
	mutex_unlock(&gsp->msgq.mutex);
	return ret;
}

static void
r515_gsp_rpc_done(struct nvkm_gsp *gsp, void *repv)
{
	struct nvfw_gsp_rpc *rpc = container_of(repv, typeof(*rpc), data);

	r515_gsp_msg_done(gsp, rpc);
}

static int
r515_gsp_rpc_poll(struct nvkm_gsp *gsp, u32 fn)
{
	void *repv;

	mutex_lock(&gsp->cmdq.mutex);
	repv = r515_gsp_msg_recv(gsp, fn, 0);
	mutex_unlock(&gsp->cmdq.mutex);
	if (IS_ERR(repv))
		return PTR_ERR(repv);

	return 0;
}

static void *
r515_gsp_rpc_push(struct nvkm_gsp *gsp, void *argv, bool wait, u32 repc)
{
	struct nvfw_gsp_rpc *rpc = container_of(argv, typeof(*rpc), data);
	struct nvfw_gsp_rpc *msg;
	u32 fn = rpc->function;
	void *repv = NULL;
	int ret;

	mutex_lock(&gsp->cmdq.mutex);
	if (gsp->subdev.debug >= NV_DBG_TRACE) {
		nvkm_trace(&gsp->subdev, "rpc fn:%d len:0x%x/0x%zx\n", rpc->function,
			   rpc->length, rpc->length - sizeof(*rpc));
		print_hex_dump(KERN_INFO, "rpc: ", DUMP_PREFIX_OFFSET, 16, 1,
			       rpc->data, rpc->length - sizeof(*rpc), true);
	}

	ret = r515_gsp_cmdq_push(gsp, rpc);
	if (ret) {
		mutex_unlock(&gsp->cmdq.mutex);
		return ERR_PTR(ret);
	}

	if (wait) {
		msg = r515_gsp_msg_recv(gsp, fn, repc);
		if (!IS_ERR_OR_NULL(msg))
			repv = msg->data;
		else
			repv = msg;
	}

	mutex_unlock(&gsp->cmdq.mutex);
	return repv;
}

static int
r515_gsp_rpc_wr(struct nvkm_gsp *gsp, void *argv, bool wait)
{
	void *repv = r515_gsp_rpc_push(gsp, argv, wait, 0);

	if (IS_ERR(repv))
		return PTR_ERR(repv);

	return 0;
}

static void *
r515_gsp_rpc_get(struct nvkm_gsp *gsp, u32 fn, u32 argc)
{
	struct nvfw_gsp_rpc *rpc;

	rpc = r515_gsp_cmdq_get(gsp, sizeof(*rpc) + argc);
	if (!rpc)
		return NULL;

	rpc->header_version = 0x03000000;
	rpc->signature = ('V' << 24) | ('R' << 16) | ('P' << 8) | 'C';
	rpc->function = fn;
	rpc->rpc_result = 0xffffffff;
	rpc->rpc_result_private = 0xffffffff;
	rpc->length = sizeof(*rpc) + argc;
	return rpc->data;
}

static void *
r515_gsp_rpc_rd(struct nvkm_gsp *gsp, u32 fn, u32 argc)
{
	void *argv = r515_gsp_rpc_get(gsp, fn, argc);

	if (IS_ERR_OR_NULL(argv))
		return argv;

	return r515_gsp_rpc_push(gsp, argv, true, argc);
}

static int
r515_gsp_rpc_update_bar_pde(struct nvkm_gsp *gsp, int id, u64 addr)
{
	struct {
		u32 bar;
		u64 addr;
		u64 shift;
	} *rpc;

	rpc = r515_gsp_rpc_get(gsp, 70, sizeof(*rpc));
	if (WARN_ON(IS_ERR_OR_NULL(rpc)))
		return -EIO;

	rpc->bar = id - 1;
	rpc->addr = addr ? ((addr >> 4) | 2) : 0; /* PD3 entry format! */
	rpc->shift = 47; //XXX: probably fetch this from mmu!

	return r515_gsp_rpc_wr(gsp, rpc, true);
}

const struct nvkm_gsp_rpc
r515_gsp_rpc = {
	.update_bar_pde = r515_gsp_rpc_update_bar_pde,
};

static int
r515_gsp_rpc_unloading_guest_driver(struct nvkm_gsp *gsp)
{
	struct nvfw_registry {
		u8  bSuspend;
		u8  bGc6Entering;
		u32 newLevel;
	} *rpc;

	rpc = r515_gsp_rpc_get(gsp, 47, sizeof(*rpc));
	if (IS_ERR(rpc))
		return PTR_ERR(rpc);

	rpc->bSuspend = 0;
	rpc->bGc6Entering = 0;
	rpc->newLevel = 0;

	return r515_gsp_rpc_wr(gsp, rpc, true);
}

static int
r515_gsp_rpc_get_gsp_static_info(struct nvkm_gsp *gsp)
{
	GspStaticConfigInfo *rpc;

	rpc = r515_gsp_rpc_rd(gsp, 65, sizeof(*rpc));
	if (IS_ERR(rpc))
		return PTR_ERR(rpc);

	gsp->client = rpc->hInternalClient;
	gsp->device = rpc->hInternalDevice;
	gsp->subdevice = rpc->hInternalSubdevice;
	gsp->bar.rm_bar1_pdb = rpc->bar1PdeBase;
	gsp->bar.rm_bar2_pdb = rpc->bar2PdeBase;

	r515_gsp_rpc_done(gsp, rpc);
	return 0;
}

static int
r515_gsp_rpc_set_registry(struct nvkm_gsp *gsp)
{
	struct nvfw_registry {
		u32	size;
		u32	numEntries;
		struct {
			u32	nameOffset;
			u8 	type;
			u32	data;
			u32	length;
		} entries[1];
		char strings[1];
	} *rpc;

	rpc = r515_gsp_rpc_get(gsp, 73, sizeof(*rpc));
	if (IS_ERR(rpc))
		return PTR_ERR(rpc);

	rpc->size = sizeof(*rpc);
	rpc->numEntries = ARRAY_SIZE(rpc->entries);
	rpc->entries[0].nameOffset = offsetof(typeof(*rpc), strings);
	rpc->entries[0].type = 1;
	rpc->entries[0].data = 0;
	rpc->entries[0].length = 4;
	rpc->strings[0] = '\0';

	return r515_gsp_rpc_wr(gsp, rpc, false);
}

static int
r515_gsp_rpc_set_system_info(struct nvkm_gsp *gsp)
{
	struct nvkm_device *device = gsp->subdev.device;
	struct nvkm_device_pci *pdev = container_of(device, typeof(*pdev), device);
	struct {
		u64 gpuPhysAddr;
		u64 gpuPhysFbAddr;
		u64 gpuPhysInstAddr;
		u64 nvDomainBusDeviceFunc;
		u64 simAccessBufPhysAddr;
		u64 pcieAtomicsOpMask;
		u64 consoleMemSize;
		u32 pciConfigMirrorBase;
		u32 pciConfigMirrorSize;
		u8  oorArch;
		u64 clPdbProperties;
		u32 Chipset;
		struct {
			u16 deviceID;
			u16 vendorID;
			u16 subdeviceID;
			u16 subvendorID;
			u8  revisionID;
		} FHBBusInfo;
	} *info;

	if (WARN_ON(device->type == NVKM_DEVICE_TEGRA))
		return -ENOSYS;

	info = r515_gsp_rpc_get(gsp, 72, sizeof(*info));
	if (IS_ERR(info))
		return PTR_ERR(info);

	info->gpuPhysAddr = device->func->resource_addr(device, 0);
	info->gpuPhysFbAddr = device->func->resource_addr(device, 1);
	info->gpuPhysInstAddr = device->func->resource_addr(device, 3);
	info->nvDomainBusDeviceFunc = pci_dev_id(pdev->pdev);
	info->pciConfigMirrorBase = 0x088000;
	info->pciConfigMirrorSize = 0x001000;

	return r515_gsp_rpc_wr(gsp, info, false);
}

static int
r515_gsp_msg_run_cpu_sequencer(void *priv, u32 fn, void *repv, u32 repc)
{
	struct nvkm_gsp *gsp = priv;
	struct nvkm_subdev *subdev = &gsp->subdev;
	struct nvkm_device *device = subdev->device;
	struct {
		u32 bufferSizeDWord;
		u32 cmdIndex;
		u32 regSaveArea[8];
		u32 commandBuffer[];
	} *seq = repv;
	int ptr = 0, ret;

	nvkm_debug(subdev, "seq: %08x %08x\n", seq->bufferSizeDWord, seq->cmdIndex);

	while (ptr < seq->cmdIndex) {
		GSP_SEQUENCER_BUFFER_CMD *cmd = (void *)&seq->commandBuffer[ptr];

		ptr += 1;
		ptr += GSP_SEQUENCER_PAYLOAD_SIZE_DWORDS(cmd->opCode);

		switch (cmd->opCode) {
		case GSP_SEQ_BUF_OPCODE_REG_WRITE: {
			u32 addr = cmd->payload.regWrite.addr;
			u32 data = cmd->payload.regWrite.val;

			nvkm_trace(subdev, "seq wr32 %06x %08x\n", addr, data);
			nvkm_wr32(device, addr, data);
		}
			break;
		case GSP_SEQ_BUF_OPCODE_REG_MODIFY: {
			u32 addr = cmd->payload.regModify.addr;
			u32 mask = cmd->payload.regModify.mask;
			u32 data = cmd->payload.regModify.val;

			nvkm_trace(subdev, "seq mask %06x %08x %08x\n", addr, mask, data);
			nvkm_mask(device, addr, mask, data);
		}
			break;
		case GSP_SEQ_BUF_OPCODE_REG_POLL: {
			u32 addr = cmd->payload.regPoll.addr;
			u32 mask = cmd->payload.regPoll.mask;
			u32 data = cmd->payload.regPoll.val;
			u32 usec = cmd->payload.regPoll.timeout ?: 4000000;
			//u32 error = cmd->payload.regPoll.error;

			nvkm_trace(subdev, "seq poll %06x %08x %08x %d\n", addr, mask, data, usec);
			nvkm_rd32(device, addr);
			nvkm_usec(device, usec,
				if ((nvkm_rd32(device, addr) & mask) == data)
					break;
			);
		}
			break;
		case GSP_SEQ_BUF_OPCODE_DELAY_US: {
			u32 usec = cmd->payload.delayUs.val;

			nvkm_trace(subdev, "seq usec %d\n", usec);
			udelay(usec);
		}
			break;
		case GSP_SEQ_BUF_OPCODE_REG_STORE: {
			u32 addr = cmd->payload.regStore.addr;
			u32 slot = cmd->payload.regStore.index;

			seq->regSaveArea[slot] = nvkm_rd32(device, addr);
			nvkm_trace(subdev, "seq save %08x -> %d: %08x\n", addr, slot,
				   seq->regSaveArea[slot]);
		}
			break;
		case GSP_SEQ_BUF_OPCODE_CORE_RESET:
			nvkm_trace(subdev, "seq core reset\n");
			nvkm_falcon_reset(&gsp->falcon);
			nvkm_falcon_mask(&gsp->falcon, 0x624, 0x00000080, 0x00000080);
			nvkm_falcon_wr32(&gsp->falcon, 0x10c, 0x00000000);
			break;
		case GSP_SEQ_BUF_OPCODE_CORE_START:
			nvkm_trace(subdev, "seq core start\n");
			if (nvkm_falcon_rd32(&gsp->falcon, 0x100) & 0x00000040)
				nvkm_falcon_wr32(&gsp->falcon, 0x130, 0x00000002);
			else
				nvkm_falcon_wr32(&gsp->falcon, 0x100, 0x00000002);
			break;
		case GSP_SEQ_BUF_OPCODE_CORE_WAIT_FOR_HALT:
			nvkm_trace(subdev, "seq core wait halt\n");
			nvkm_msec(device, 2000,
				if (nvkm_falcon_rd32(&gsp->falcon, 0x100) & 0x00000010)
					break;
			);
			break;
		case GSP_SEQ_BUF_OPCODE_CORE_RESUME: {
			struct nvkm_sec2 *sec2 = device->sec2;
			u32 mbox0;

			nvkm_trace(subdev, "seq core resume\n");

			ret = gsp->func->reset(gsp);
			if (WARN_ON(ret))
				return ret;

			nvkm_falcon_wr32(&gsp->falcon, 0x040, lower_32_bits(gsp->libos.addr));
			nvkm_falcon_wr32(&gsp->falcon, 0x044, upper_32_bits(gsp->libos.addr));

			nvkm_falcon_start(&sec2->falcon);

			if (nvkm_msec(device, 2000,
				if (nvkm_rd32(device, 0x1180f8) & 0x04000000)
					break;
			) < 0)
				return -ETIMEDOUT;

			mbox0 = nvkm_falcon_rd32(&sec2->falcon, 0x040);
			if (WARN_ON(mbox0)) {
				nvkm_error(&gsp->subdev, "seq core resume sec2: 0x%x\n", mbox0);
				return -EIO;
			}

			nvkm_falcon_wr32(&gsp->falcon, 0x080, gsp->boot.app_version);

			if (WARN_ON(!nvkm_falcon_riscv_active(&gsp->falcon)))
				return -EIO;
		}
			break;
		default:
			nvkm_error(subdev, "unknown sequencer opcode %08x\n", cmd->opCode);
			return -EINVAL;
		}
	}

	return 0;
}

static void
nvkm_gsp_mem_dtor(struct nvkm_gsp *gsp, struct nvkm_gsp_mem *mem)
{
	if (mem->data) {
		dma_free_coherent(gsp->subdev.device->dev, mem->size, mem->data, mem->addr);
		mem->data = NULL;
	}
}

static int
nvkm_gsp_mem_ctor(struct nvkm_gsp *gsp, u32 size, struct nvkm_gsp_mem *mem)
{
	mem->size = size;
	mem->data = dma_alloc_coherent(gsp->subdev.device->dev, size, &mem->addr, GFP_KERNEL);
	if (WARN_ON(!mem->data))
		return -ENOMEM;

	return 0;
}


static int
r515_gsp_booter_boot(struct nvkm_gsp *gsp, const char *name,
		     const struct nvkm_gsp_booter_fw *booter, u32 mbox0, u32 mbox1,
		     struct nvkm_falcon *falcon)
{
	struct nvkm_subdev *subdev = &gsp->subdev;
	struct nvkm_falcon_fw fw = {};
	const struct nvkm_gsp_booter_fw_hdr *hdr = (void *)booter->hdr.data;
	int ret;

	nvkm_debug(subdev, "%s hdr:\n", name);
	nvkm_debug(subdev, "\tos_code_offset : %08x\n", hdr->os_code_offset);
	nvkm_debug(subdev, "\tos_code_size   : %08x\n", hdr->os_code_size);
	nvkm_debug(subdev, "\tos_data_offset : %08x\n", hdr->os_data_offset);
	nvkm_debug(subdev, "\tos_data_size   : %08x\n", hdr->os_data_size);
	nvkm_debug(subdev, "\tnum_apps       : %08x\n", hdr->num_apps);
	nvkm_debug(subdev, "\tapp_code_offset: %08x\n", hdr->app_code_offset);
	nvkm_debug(subdev, "\tapp_code_size  : %08x\n", hdr->app_code_size);
	nvkm_debug(subdev, "\tapp_data_offset: %08x\n", hdr->app_data_offset);
	nvkm_debug(subdev, "\tapp_data_size  : %08x\n", hdr->app_data_size);

	ret = gsp->func->booter.ctor(gsp, name, booter, falcon, &fw);
	if (ret)
		return ret;

	ret = nvkm_falcon_fw_sign(&fw, booter->patch_loc, booter->sig.size / booter->num_sigs,
				  booter->sig.data, booter->num_sigs, 0, 0, 0);
	if (WARN_ON(ret))
		return ret;

	ret = nvkm_falcon_fw_boot(&fw, subdev, true, &mbox0, &mbox1, 0, 0);
	nvkm_falcon_fw_dtor(&fw);
	return 0;
}

static int
r515_gsp_booter_unload(struct nvkm_gsp *gsp, const struct nvkm_gsp_booter_fw *booter)
{
	struct nvkm_subdev *subdev = &gsp->subdev;
	struct nvkm_device *device = subdev->device;
	u32 wpr2_hi;
	int ret;

	wpr2_hi = nvkm_rd32(device, 0x1fa828);
	if (!wpr2_hi) {
		nvkm_debug(subdev, "WPR2 not set - skipping booter unload\n");
		return 0;
	}

	ret = r515_gsp_booter_boot(gsp, "booter-unload", booter, 0xff, 0xff, &device->sec2->falcon);
	if (WARN_ON(ret))
		return ret;

	wpr2_hi = nvkm_rd32(device, 0x1fa828);
	if (WARN_ON(wpr2_hi))
		return -EIO;

	return 0;
}

static int
r515_gsp_booter_load(struct nvkm_gsp *gsp)
{
	struct {
	    u64 magic;
	    u64 revision;
	    u64 sysmemAddrOfRadix3Elf;
	    u64 sizeOfRadix3Elf;
	    u64 sysmemAddrOfBootloader;
	    u64 sizeOfBootloader;
	    u64 bootloaderCodeOffset;
	    u64 bootloaderDataOffset;
	    u64 bootloaderManifestOffset;
	    u64 sysmemAddrOfSignature;
	    u64 sizeOfSignature;
	    u64 gspFwRsvdStart;
	    u64 nonWprHeapOffset;
	    u64 nonWprHeapSize;
	    u64 gspFwWprStart;
	    u64 gspFwHeapOffset;
	    u64 gspFwHeapSize;
	    u64 gspFwOffset;
	    u64 bootBinOffset;
	    u64 frtsOffset;
	    u64 frtsSize;
	    u64 gspFwWprEnd;
	    u64 fbSize;
	    u64 vgaWorkspaceOffset;
	    u64 vgaWorkspaceSize;
	    u64 bootCount;
	    u64 partitionRpcAddr;
	    u16 partitionRpcRequestOffset;
	    u16 partitionRpcReplyOffset;
	    u32 padding[7];
	    u64 verified;
	} *meta;
	int ret;

	ret = nvkm_gsp_mem_ctor(gsp, 0x1000, &gsp->wpr_meta);
	if (ret)
		return ret;

	meta = gsp->wpr_meta.data;

	meta->magic = 0xdc3aae21371a60b3ULL;
	meta->revision = 1;

	meta->sysmemAddrOfRadix3Elf = gsp->radix3[0].addr;
	meta->sizeOfRadix3Elf = gsp->fb.wpr2.elf.size;

	meta->sysmemAddrOfBootloader = gsp->boot.fw.addr;
	meta->sizeOfBootloader = gsp->boot.fw.size;
	meta->bootloaderCodeOffset = gsp->boot.code_offset;
	meta->bootloaderDataOffset = gsp->boot.data_offset;
	meta->bootloaderManifestOffset = gsp->boot.manifest_offset;

	meta->sysmemAddrOfSignature = gsp->sig.addr;
	meta->sizeOfSignature = gsp->sig.size;

	meta->gspFwRsvdStart = gsp->fb.heap.addr;
	meta->nonWprHeapOffset = gsp->fb.heap.addr;
	meta->nonWprHeapSize = gsp->fb.heap.size;
	meta->gspFwWprStart = gsp->fb.wpr2.addr;
	meta->gspFwHeapOffset = ALIGN(gsp->fb.wpr2.heap.addr + sizeof(*meta), 0x1000);
	meta->gspFwHeapSize = gsp->fb.wpr2.elf.addr - meta->gspFwHeapOffset;
	meta->gspFwOffset = gsp->fb.wpr2.elf.addr;
	meta->bootBinOffset = gsp->fb.wpr2.boot.addr;
	meta->frtsOffset = gsp->fb.wpr2.frts.addr;
	meta->frtsSize = gsp->fb.wpr2.frts.size;
	meta->gspFwWprEnd = gsp->fb.wpr2.addr + gsp->fb.wpr2.size;
	meta->fbSize = gsp->fb.size;
	meta->vgaWorkspaceOffset = gsp->fb.bios.vga_workspace.addr;
	meta->vgaWorkspaceSize = gsp->fb.bios.vga_workspace.size;
	meta->bootCount = 0;
	meta->partitionRpcAddr = 0;
	meta->partitionRpcRequestOffset = 0;
	meta->partitionRpcReplyOffset = 0;
	meta->verified = 0;

	return r515_gsp_booter_boot(gsp, "booter-load", gsp->func->booter.load,
				    lower_32_bits(gsp->wpr_meta.addr),
				    upper_32_bits(gsp->wpr_meta.addr),
				    &gsp->subdev.device->sec2->falcon);
}

static int
r515_gsp_shared_init(struct nvkm_gsp *gsp)
{
	int ret, i;
	struct nvfw_gsp_msgq *cmdq;
	struct nvfw_gsp_msgq *msgq;

	gsp->shm.cmdq.size = 0x40000;
	gsp->shm.msgq.size = 0x40000;

	gsp->shm.ptes.nr  = (gsp->shm.cmdq.size + gsp->shm.msgq.size) >> GSP_PAGE_SHIFT;
	gsp->shm.ptes.nr += DIV_ROUND_UP(gsp->shm.ptes.nr * sizeof(u64), GSP_PAGE_SIZE);
	gsp->shm.ptes.size = ALIGN(gsp->shm.ptes.nr * sizeof(u64), GSP_PAGE_SIZE);

	ret = nvkm_gsp_mem_ctor(gsp, gsp->shm.ptes.size +
				     gsp->shm.cmdq.size +
				     gsp->shm.msgq.size,
				&gsp->shm.mem);
	if (ret)
		return ret;

	gsp->shm.ptes.ptr = gsp->shm.mem.data;
	gsp->shm.cmdq.ptr = (u8 *)gsp->shm.ptes.ptr + gsp->shm.ptes.size;
	gsp->shm.msgq.ptr = (u8 *)gsp->shm.cmdq.ptr + gsp->shm.cmdq.size;

	for (i = 0; i < gsp->shm.ptes.nr; i++)
		gsp->shm.ptes.ptr[i] = gsp->shm.mem.addr + (i << GSP_PAGE_SHIFT);

	cmdq = gsp->shm.cmdq.ptr;
	cmdq->tx.version = 0;
	cmdq->tx.size = gsp->shm.cmdq.size;
	cmdq->tx.entryOff = GSP_PAGE_SIZE;
	cmdq->tx.msgSize = GSP_PAGE_SIZE;
	cmdq->tx.msgCount = (cmdq->tx.size - cmdq->tx.entryOff) / cmdq->tx.msgSize;
	cmdq->tx.writePtr = 0;
	cmdq->tx.flags = 1;
	cmdq->tx.rxHdrOff = offsetof(typeof(*cmdq), rx.readPtr);

	msgq = gsp->shm.msgq.ptr;

	gsp->cmdq.cnt = cmdq->tx.msgCount;
	gsp->cmdq.wptr = &cmdq->tx.writePtr;
	gsp->cmdq.rptr = &msgq->rx.readPtr;
	gsp->msgq.cnt = cmdq->tx.msgCount;
	gsp->msgq.wptr = &msgq->tx.writePtr;
	gsp->msgq.rptr = &cmdq->rx.readPtr;
	return 0;
}

static int
r515_gsp_rmargs_init(struct nvkm_gsp *gsp)
{
	struct {
		struct {
		    u64 addr;
		    u32 ptes;
		    u64 cmdq_offset;
		    u64 msgq_offset;
		} mq;
		struct {
		    u32 old_level;
		    u32 flags;
		    u8  in_pm_transition;
		} sr;
	} *args;
	int ret;

	ret = r515_gsp_shared_init(gsp);
	if (ret)
		return ret;

	ret = nvkm_gsp_mem_ctor(gsp, 0x1000, &gsp->rmargs);
	if (ret)
		return ret;

	args = gsp->rmargs.data;
	args->mq.addr = gsp->shm.mem.addr;
	args->mq.ptes = gsp->shm.ptes.nr;
	args->mq.cmdq_offset = (u8 *)gsp->shm.cmdq.ptr - (u8 *)gsp->shm.mem.data;
	args->mq.msgq_offset = (u8 *)gsp->shm.msgq.ptr - (u8 *)gsp->shm.mem.data;
	args->sr.old_level = 0;
	args->sr.flags = 0;
	args->sr.in_pm_transition = 0;
	return 0;
}

static inline u64
r515_gsp_libos_id8(const char *name)
{
	u64 id = 0;

	for (int i = 0; i < sizeof(id) && *name; i++, name++)
		id = (id << 8) | *name;

	return id;
}

static int
r515_gsp_libos_init(struct nvkm_gsp *gsp)
{
	struct {
	    u64 id8;
	    u64 addr;
	    u64 size;
#define LIBOS_MEMORY_REGION_CONTIGUOUS 1
	    u8  kind;
#define LIBOS_MEMORY_REGION_LOC_SYSMEM 1
	    u8  loc;
	} *args;
	int ret;

	ret = nvkm_gsp_mem_ctor(gsp, 0x1000, &gsp->libos);
	if (ret)
		return ret;

	args = gsp->libos.data;

	ret = nvkm_gsp_mem_ctor(gsp, 0x10000, &gsp->loginit);
	if (ret)
		return ret;

	args[0].id8  = r515_gsp_libos_id8("LOGINIT");
	args[0].addr = gsp->loginit.addr;
	args[0].size = gsp->loginit.size;
	args[0].kind = LIBOS_MEMORY_REGION_CONTIGUOUS;
	args[0].loc  = LIBOS_MEMORY_REGION_LOC_SYSMEM;

	ret = nvkm_gsp_mem_ctor(gsp, 0x10000, &gsp->logrm);
	if (ret)
		return ret;

	args[1].id8  = r515_gsp_libos_id8("LOGRM");
	args[1].addr = gsp->logrm.addr;
	args[1].size = gsp->logrm.size;
	args[1].kind = LIBOS_MEMORY_REGION_CONTIGUOUS;
	args[1].loc  = LIBOS_MEMORY_REGION_LOC_SYSMEM;

	ret = r515_gsp_rmargs_init(gsp);
	if (ret)
		return ret;

	args[2].id8  = r515_gsp_libos_id8("RMARGS");
	args[2].addr = gsp->rmargs.addr;
	args[2].size = gsp->rmargs.size;
	args[2].kind = LIBOS_MEMORY_REGION_CONTIGUOUS;
	args[2].loc  = LIBOS_MEMORY_REGION_LOC_SYSMEM;
	return 0;
}

int
r515_gsp_fini(struct nvkm_gsp *gsp)
{
	int ret;

	ret = r515_gsp_rpc_unloading_guest_driver(gsp);
	if (WARN_ON(ret))
		return ret;

	nvkm_msec(gsp->subdev.device, 2000,
		if (nvkm_falcon_rd32(&gsp->falcon, 0x040) & 0x80000000)
			break;
	);

	nvkm_falcon_reset(&gsp->falcon);

	ret = nvkm_gsp_fwsec_sb(gsp);
	WARN_ON(ret);

	ret = r515_gsp_booter_unload(gsp, gsp->func->booter.unload);
	WARN_ON(ret);
	return ret;
}

int
r515_gsp_init(struct nvkm_gsp *gsp)
{
	int ret;

	ret = nvkm_gsp_fwsec_frts(gsp);
	if (WARN_ON(ret))
		return ret;

	ret = r515_gsp_libos_init(gsp);
	if (WARN_ON(ret))
		return ret;

	ret = r515_gsp_rpc_set_system_info(gsp);
	if (WARN_ON(ret))
		return ret;

	ret = r515_gsp_rpc_set_registry(gsp);
	if (WARN_ON(ret))
		return ret;

	/* Reset GSP into RISC-V mode. */
	ret = gsp->func->reset(gsp);
	if (WARN_ON(ret))
		return ret;

	nvkm_falcon_wr32(&gsp->falcon, 0x040, lower_32_bits(gsp->libos.addr));
	nvkm_falcon_wr32(&gsp->falcon, 0x044, upper_32_bits(gsp->libos.addr));

	/* Execute booter to handle (eventually...) booting GSP-RM. */
	ret = r515_gsp_booter_load(gsp);
	if (WARN_ON(ret))
		return ret;

	nvkm_falcon_wr32(&gsp->falcon, 0x080, gsp->boot.app_version);

	if (WARN_ON(!nvkm_falcon_riscv_active(&gsp->falcon)))
		return -EIO;

	ret = r515_gsp_rpc_poll(gsp, 0x1001 /* GSP_INIT_DONE. */);
	if (ret)
		return ret;

	gsp->running = true;

	ret = r515_gsp_rpc_get_gsp_static_info(gsp);
	if (WARN_ON(ret))
		return ret;

	return 0;
}

static int
r515_gsp_rm_boot_ctor(struct nvkm_gsp *gsp)
{
	struct {
		u32  version;
		u32  bootloaderOffset;
		u32  bootloaderSize;
		u32  bootloaderParamOffset;
		u32  bootloaderParamSize;
		u32  riscvElfOffset;
		u32  riscvElfSize;
		u32  appVersion;
		u32  manifestOffset;
		u32  manifestSize;
		u32  monitorDataOffset;
		u32  monitorDataSize;
		u32  monitorCodeOffset;
		u32  monitorCodeSize;
		u32  bIsMonitorEnabled;
		u32  swbromCodeOffset;
		u32  swbromCodeSize;
		u32  swbromDataOffset;
		u32  swbromDataSize;
	} *desc = (void *)gsp->func->boot->desc.data;
	int ret;

	ret = nvkm_gsp_mem_ctor(gsp, gsp->func->boot->img.size, &gsp->boot.fw);
	if (ret)
		return ret;

	memcpy(gsp->boot.fw.data, gsp->func->boot->img.data, gsp->func->boot->img.size);

	gsp->boot.code_offset = desc->monitorCodeOffset;
	gsp->boot.data_offset = desc->monitorDataOffset;
	gsp->boot.manifest_offset = desc->manifestOffset;
	gsp->boot.app_version = desc->appVersion;
	return 0;
}

int
r515_gsp_oneinit(struct nvkm_gsp *gsp)
{
	int ret;

	ret = r515_gsp_rm_boot_ctor(gsp);
	if (ret)
		return ret;

	gsp->fb.wpr2.frts.size = 0x100000;
	gsp->fb.wpr2.frts.addr = ALIGN_DOWN(gsp->fb.bios.addr, 0x20000) - gsp->fb.wpr2.frts.size;

	gsp->fb.wpr2.boot.size = gsp->boot.fw.size;
	gsp->fb.wpr2.boot.addr = ALIGN_DOWN(gsp->fb.wpr2.frts.addr - gsp->fb.wpr2.boot.size, 0x1000);

	gsp->fb.wpr2.elf.size = gsp->fw.len;
	gsp->fb.wpr2.elf.addr = ALIGN_DOWN(gsp->fb.wpr2.boot.addr - gsp->fb.wpr2.elf.size, 0x10000);

	gsp->fb.wpr2.heap.size = 64 * 1024 * 1024;
	gsp->fb.wpr2.heap.addr = ALIGN(gsp->fb.wpr2.elf.addr - gsp->fb.wpr2.heap.size, 0x20000);

	gsp->fb.wpr2.addr = gsp->fb.wpr2.heap.addr;
	gsp->fb.wpr2.size = gsp->fb.wpr2.frts.addr + gsp->fb.wpr2.frts.size - gsp->fb.wpr2.addr;

	gsp->fb.heap.size = 0x100000;
	gsp->fb.heap.addr = gsp->fb.wpr2.addr - gsp->fb.heap.size;
	return 0;
}

static const struct nvkm_firmware_func
r515_gsp_fw = {
	.type = NVKM_FIRMWARE_IMG_DMA,
};

void
r515_gsp_dtor(struct nvkm_gsp *gsp)
{
	mutex_destroy(&gsp->msgq.mutex);
	mutex_destroy(&gsp->cmdq.mutex);

	nvkm_gsp_mem_dtor(gsp, &gsp->radix3[2]);
	nvkm_gsp_mem_dtor(gsp, &gsp->radix3[1]);
	nvkm_gsp_mem_dtor(gsp, &gsp->radix3[0]);
	nvkm_gsp_mem_dtor(gsp, &gsp->sig);
	nvkm_firmware_dtor(&gsp->fw);
}

int
r515_gsp_load(struct nvkm_gsp *gsp, int ver, const struct nvkm_gsp_fwif *fwif)
{
	struct nvkm_subdev *subdev = &gsp->subdev;
	const struct firmware *fw;
	const struct elf64_hdr *ehdr;
	const struct elf64_shdr *shdr;
	const char *names;
	int ret, i, j;
	u64 addr, size;
	u64 *ptes;

	ret = nvkm_firmware_get(subdev, "gsp/gsp", ver, &fw);
	if (ret)
		return ret;

	/* Load ELF image into DMA-accessible memory. */
	ret = nvkm_firmware_ctor(&r515_gsp_fw, "gsp-rm", subdev->device,
				 fw->data, fw->size, &gsp->fw);
	nvkm_firmware_put(fw);
	if (ret)
		return ret;

	/* Load relevant signature, and zero out all signature sections. */
	ehdr = (const struct elf64_hdr *)gsp->fw.img;
	shdr = (const struct elf64_shdr *)&gsp->fw.img[ehdr->e_shoff];
	names = &gsp->fw.img[shdr[ehdr->e_shstrndx].sh_offset];

	for (ret = -ENOENT, i = 0; i < ehdr->e_shnum; i++, shdr++) {
		if (!strncmp(&names[shdr->sh_name], ".fwsignature_", 13)) {
			if (ret && !strcmp(&names[shdr->sh_name], fwif->sig_section)) {
				ret = nvkm_gsp_mem_ctor(gsp, ALIGN(shdr->sh_size, 256), &gsp->sig);
				if (ret)
					return ret;

				memcpy(gsp->sig.data, &gsp->fw.img[shdr->sh_offset], shdr->sh_size);
				ret = 0;
			}

			memset(&gsp->fw.img[shdr->sh_offset], 0, shdr->sh_size);
		}
	}

	if (ret) {
		nvkm_firmware_dtor(&gsp->fw);
		return ret;
	}

	/* Build radix3 page table for ELF image. */
	addr = gsp->fw.phys;
	size = sg_dma_len(&gsp->fw.mem.sgl);

	for (i = ARRAY_SIZE(gsp->radix3) - 1; i >= 0; i--) {
		gsp->radix3[i].size = ALIGN((size / GSP_PAGE_SIZE) * sizeof(u64), GSP_PAGE_SIZE);
		gsp->radix3[i].data = dma_alloc_coherent(subdev->device->dev, gsp->radix3[i].size,
							 &gsp->radix3[i].addr, GFP_KERNEL);
		if (WARN_ON(!gsp->radix3[i].data))
			return -ENOMEM;

		ptes = gsp->radix3[i].data;
		for (j = 0; j < size / GSP_PAGE_SIZE; j++)
			*ptes++ = addr + GSP_PAGE_SIZE * j;

		size = gsp->radix3[i].size;
		addr = gsp->radix3[i].addr;
	}

	r515_gsp_msg_ntfy_add(gsp, 0x00001002, r515_gsp_msg_run_cpu_sequencer, gsp);
	return 0;
}

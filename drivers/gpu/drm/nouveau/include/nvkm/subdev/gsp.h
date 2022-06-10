#ifndef __NVKM_GSP_H__
#define __NVKM_GSP_H__
#define nvkm_gsp(p) container_of((p), struct nvkm_gsp, subdev)
#include <core/subdev.h>
#include <core/falcon.h>
#include <core/firmware.h>

struct nvkm_gsp_mem {
	u32 size;
	void *data;
	u64 addr;
};

typedef int (*nvkm_gsp_msg_ntfy_func)(void *priv, u32 fn, void *repv, u32 repc);

struct nvkm_gsp {
	const struct nvkm_gsp_func *func;
	struct nvkm_subdev subdev;

	struct nvkm_falcon falcon;

	struct nvkm_firmware fw;
	struct nvkm_gsp_mem sig;
	struct nvkm_gsp_mem radix3[3];

	struct {
		struct {
			struct {
				u64 addr;
				u64 size;
			} vga_workspace;
			u64 addr;
			u64 size;
		} bios;
		struct {
			struct {
				u64 addr;
				u64 size;
			} frts, boot, elf, heap;
			u64 addr;
			u64 size;
		} wpr2;
		struct {
			u64 addr;
			u64 size;
		} heap;
		u64 addr;
		u64 size;
	} fb;

	struct {
		struct nvkm_gsp_mem fw;
		u32 code_offset;
		u32 data_offset;
		u32 manifest_offset;
		u32 app_version;
	} boot;

	struct nvkm_gsp_mem wpr_meta;

	struct nvkm_gsp_mem libos;
	struct nvkm_gsp_mem loginit;
	struct nvkm_gsp_mem logrm;
	struct nvkm_gsp_mem rmargs;

	struct {
		struct nvkm_gsp_mem mem;

		struct {
			int   nr;
			u32 size;
			u64 *ptr;
		} ptes;

		struct {
			u32  size;
			void *ptr;
		} cmdq, msgq;
	} shm;

	struct nvkm_gsp_cmdq {
		struct mutex mutex;
		u32 cnt;
		u32 seq;
		u32 *wptr;
		u32 *rptr;
	} cmdq;

	struct nvkm_gsp_msgq {
		struct mutex mutex;
		u32 cnt;
		u32 *wptr;
		u32 *rptr;
		struct nvkm_gsp_msgq_ntfy {
			u32 fn;
			nvkm_gsp_msg_ntfy_func func;
			void *priv;
		} ntfy[16];
		int ntfy_nr;
	} msgq;

	bool running;

	/* Internal GSP-RM control handles. */
	u32 client;
	u32 device;
	u32 subdevice;
};

static inline bool
nvkm_gsp_rm(struct nvkm_gsp *gsp)
{
	return gsp && gsp->fw.img != NULL;
}

int gv100_gsp_new(struct nvkm_device *, enum nvkm_subdev_type, int, struct nvkm_gsp **);
int tu102_gsp_new(struct nvkm_device *, enum nvkm_subdev_type, int, struct nvkm_gsp **);
int tu116_gsp_new(struct nvkm_device *, enum nvkm_subdev_type, int, struct nvkm_gsp **);
int ga100_gsp_new(struct nvkm_device *, enum nvkm_subdev_type, int, struct nvkm_gsp **);
int ga102_gsp_new(struct nvkm_device *, enum nvkm_subdev_type, int, struct nvkm_gsp **);
#endif

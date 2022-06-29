#ifndef __NVKM_GSP_H__
#define __NVKM_GSP_H__
#define nvkm_gsp(p) container_of((p), struct nvkm_gsp, subdev)
#include <core/subdev.h>
#include <core/falcon.h>
#include <core/firmware.h>

#define GPC_MAX 32
#define GSP_MAX_ENGINES 0x34

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

	u32 gpc_mask;

	struct {
		u32 gpc_id;
		u32 tpc_mask;
	} tpc[GPC_MAX];
	u32 tpc_max;
	u32 tpc_total;

	u32 num_engines;
	u32 engine_ids[GSP_MAX_ENGINES];

	struct {
		u64 rm_bar1_pdb;
		u64 rm_bar2_pdb;
	} bar;

	const struct nvkm_gsp_rpc {
		int (*update_bar_pde)(struct nvkm_gsp *, int bar, u64 addr);

		void *(*rm_alloc_get)(struct nvkm_gsp *, u32 client, u32 parent, u32 object,
				      u32 oclass, u32 argc);
		void *(*rm_alloc_push)(struct nvkm_gsp *, void *argv, bool wait, u32 repc);
		void (*rm_alloc_done)(struct nvkm_gsp *, void *repv);

		void *(*rm_ctrl_get)(struct nvkm_gsp *, u32 client, u32 object, u32 cmd, u32 argc);
		void *(*rm_ctrl_push)(struct nvkm_gsp *, void *argv, bool wait, u32 repc);
		void (*rm_ctrl_done)(struct nvkm_gsp *, void *repv);
	} *rpc;

	atomic_t client_id; /*XXX: allocator */
};

static inline bool
nvkm_gsp_rm(struct nvkm_gsp *gsp)
{
	return gsp && gsp->fw.img != NULL;
}

static inline void *
nvkm_gsp_rm_alloc_get(struct nvkm_gsp *gsp, u32 client, u32 parent, u32 object,
		      u32 oclass, u32 argc)
{
	return gsp->rpc->rm_alloc_get(gsp, client, parent, object, oclass, argc);
}

static inline void *
nvkm_gsp_rm_alloc_push(struct nvkm_gsp *gsp, void *argv, bool wait, u32 repc)
{
	return gsp->rpc->rm_alloc_push(gsp, argv, wait, repc);
}

static inline int
nvkm_gsp_rm_alloc_wr(struct nvkm_gsp *gsp, void *argv, bool wait)
{
	void *repv = gsp->rpc->rm_alloc_push(gsp, argv, wait, 0);

	if (IS_ERR(repv))
		return PTR_ERR(repv);

	return 0;
}

static inline void
nvkm_gsp_rm_alloc_done(struct nvkm_gsp *gsp, void *repv)
{
	gsp->rpc->rm_alloc_done(gsp, repv);
}

static inline int
nvkm_gsp_rm_alloc(struct nvkm_gsp *gsp, u32 client, u32 parent, u32 object, u32 oclass, u32 argc)
{
	void *argv = gsp->rpc->rm_alloc_get(gsp, client, parent, object, oclass, argc);

	if (IS_ERR_OR_NULL(argv))
		return argv ? PTR_ERR(argv) : -EIO;

	return nvkm_gsp_rm_alloc_wr(gsp, argv, true);
}

static inline void *
nvkm_gsp_rm_ctrl_get(struct nvkm_gsp *gsp, u32 client, u32 object, u32 cmd, u32 argc)
{
	return gsp->rpc->rm_ctrl_get(gsp, client, object, cmd, argc);
}

static inline void *
nvkm_gsp_rm_ctrl_push(struct nvkm_gsp *gsp, void *argv, bool wait, u32 repc)
{
	return gsp->rpc->rm_ctrl_push(gsp, argv, wait, repc);
}

static inline int
nvkm_gsp_rm_ctrl_wr(struct nvkm_gsp *gsp, void *argv, bool wait)
{
	void *repv = gsp->rpc->rm_ctrl_push(gsp, argv, wait, 0);

	if (IS_ERR(repv))
		return PTR_ERR(repv);

	return 0;
}

static inline void
nvkm_gsp_rm_ctrl_done(struct nvkm_gsp *gsp, void *repv)
{
	gsp->rpc->rm_ctrl_done(gsp, repv);
}

int gv100_gsp_new(struct nvkm_device *, enum nvkm_subdev_type, int, struct nvkm_gsp **);
int tu102_gsp_new(struct nvkm_device *, enum nvkm_subdev_type, int, struct nvkm_gsp **);
int tu116_gsp_new(struct nvkm_device *, enum nvkm_subdev_type, int, struct nvkm_gsp **);
int ga100_gsp_new(struct nvkm_device *, enum nvkm_subdev_type, int, struct nvkm_gsp **);
int ga102_gsp_new(struct nvkm_device *, enum nvkm_subdev_type, int, struct nvkm_gsp **);

u64 nvkm_gsp_units(struct nvkm_gsp *gsp);
#endif

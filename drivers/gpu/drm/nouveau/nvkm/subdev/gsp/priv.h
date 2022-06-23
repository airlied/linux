/* SPDX-License-Identifier: MIT */
#ifndef __NVKM_GSP_PRIV_H__
#define __NVKM_GSP_PRIV_H__
#include <subdev/gsp.h>
enum nvkm_acr_lsf_id;

int nvkm_gsp_fwsec_frts(struct nvkm_gsp *);
int nvkm_gsp_fwsec_sb(struct nvkm_gsp *);

struct nvkm_gsp_fwif {
	int version;
	int (*load)(struct nvkm_gsp *, int ver, const struct nvkm_gsp_fwif *);
	const struct nvkm_gsp_func *func;
	char *sig_section;
};

int gv100_gsp_nofw(struct nvkm_gsp *, int, const struct nvkm_gsp_fwif *);
int  r515_gsp_load(struct nvkm_gsp *, int, const struct nvkm_gsp_fwif *);

struct nvkm_gsp_booter_fw {
	struct {
		const u8 *data;
		u32 size;
	} img, hdr, sig;
	u32 patch_loc;
	u32 patch_sig;
	u32 fuse_ver;
	u32 engine_id;
	u32 ucode_id;
	u32 num_sigs;
};

struct nvkm_gsp_booter_fw_hdr {
	u32 os_code_offset;
	u32 os_code_size;
	u32 os_data_offset;
	u32 os_data_size;
	u32 num_apps;
	u32 app_code_offset;
	u32 app_code_size;
	u32 app_data_offset;
	u32 app_data_size;
};

struct nvkm_gsp_func {
	const struct nvkm_falcon_func *flcn;
	const struct nvkm_falcon_fw_func *fwsec;

	struct {
		int (*ctor)(struct nvkm_gsp *, const char *name, const struct nvkm_gsp_booter_fw *,
			    struct nvkm_falcon *, struct nvkm_falcon_fw *);
		const struct nvkm_gsp_booter_fw *load;
		const struct nvkm_gsp_booter_fw *unload;
	} booter;

	const struct nvkm_gsp_fw_boot {
		struct {
			const u8 *data;
			u32 size;
		} desc, img;
	} *boot;

	void (*dtor)(struct nvkm_gsp *);
	int (*oneinit)(struct nvkm_gsp *);
	int (*init)(struct nvkm_gsp *);
	int (*fini)(struct nvkm_gsp *);
	int (*reset)(struct nvkm_gsp *);

	const struct nvkm_gsp_rpc *rpc;
};

extern const struct nvkm_gsp_func gv100_gsp;

extern const struct nvkm_falcon_func tu102_gsp_flcn;
extern const struct nvkm_falcon_fw_func tu102_gsp_fwsec;
int tu102_gsp_booter_ctor(struct nvkm_gsp *, const char *, const struct nvkm_gsp_booter_fw *,
			  struct nvkm_falcon *, struct nvkm_falcon_fw *);
int tu102_gsp_oneinit(struct nvkm_gsp *);
int tu102_gsp_reset(struct nvkm_gsp *);

void r515_gsp_dtor(struct nvkm_gsp *);
int r515_gsp_oneinit(struct nvkm_gsp *);
int r515_gsp_init(struct nvkm_gsp *);
int r515_gsp_fini(struct nvkm_gsp *);
extern const struct nvkm_gsp_rpc r515_gsp_rpc;

int nvkm_gsp_new_(const struct nvkm_gsp_fwif *, struct nvkm_device *, enum nvkm_subdev_type, int,
		  struct nvkm_gsp **);

extern const struct nvkm_gsp_func gv100_gsp;
#endif

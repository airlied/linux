/* SPDX-License-Identifier: MIT */
#ifndef __NVKM_GSP_FW_MSGQ_H__
#define __NVKM_GSP_FW_MSGQ_H__

struct nvfw_gsp_msgq_tx {
    u32 version;   // queue version
    u32 size;      // bytes, page aligned
    u32 msgSize;   // entry size, bytes, must be power-of-2, 16 is minimum
    u32 msgCount;  // number of entries in queue
    u32 writePtr;  // message id of next slot
    u32 flags;     // if set it means "i want to swap RX"
    u32 rxHdrOff;  // Offset of msgqRxHeader from start of backing store.
    u32 entryOff;  // Offset of entries from start of backing store.
};

struct nvfw_gsp_msgq_rx {
    u32 readPtr;   // message id of last message read
};

struct nvfw_gsp_msgq {
	struct nvfw_gsp_msgq_tx tx;
	struct nvfw_gsp_msgq_rx rx;
};
#endif
